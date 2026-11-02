# -*- coding: utf-8 -*-
"""倒立摆位置控制上位机。

对应固件：jiangxiedaolibai_pure_pd（USB 115200 8N1，遥测 v3 100 Hz）
新增命令：op4 设置位置目标 / op5 设置斜坡步长（见 protocol.py 与 README）。

用法:
    python host_app.py
    python host_app.py --port COM5
"""

import argparse
import collections
import csv
import datetime as _dt
import json
import math
import os
import queue
import random
import re
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P


def source_pid_defaults():
    """从 src/pendulum_ctrl.v 读编译期 PID 参数，作为参数页初值。

    这样“写入 PID”默认发的就是源码里的值，不会把已调好的增益改坏。
    """
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), 'src', 'pendulum_ctrl.v')
    out = {}
    try:
        with open(path, encoding='utf-8') as fh:
            text = fh.read()
    except OSError:
        return out
    for name in ('KP_A_MAN', 'KI_A_MAN', 'KD_A_MAN',
                 'KP_P_MAN', 'KI_P_MAN', 'KD_P_MAN',
                 'KP_A_AUTO', 'KI_A_AUTO', 'KD_A_AUTO',
                 'KP_P_AUTO', 'KI_P_AUTO', 'KD_P_AUTO'):
        m = re.search(r"%s\s*=\s*16'sd(-?\d+)" % name, text)
        if m:
            out[name] = int(m.group(1))
    return out

try:
    import serial
    from serial.tools import list_ports
except ImportError:                                    # pragma: no cover
    serial = None
    list_ports = None

APP_TITLE = '倒立摆位置控制上位机'
APP_VERSION = '1.3 (10-18 17:56)'
BAUD = 115200
POLL_MS = 20
DRAW_MS = 50
LOG_MAX_LINES = 400
UNDO_DEPTH = 30
DATA_MAX = 12000

COL_BG = '#f6f8fa'
COL_PANEL = '#ffffff'
COL_TEXT = '#1f2328'
COL_MUTED = '#57606a'
COL_OK = '#1a7f37'
COL_WARN = '#9a6700'
COL_ACCENT = '#0969da'
COL_GRID = '#d8dee4'
COL_THETA = '#cf222e'
COL_LOC = '#0969da'
COL_TGT = '#1a7f37'
COL_PWM = '#8250df'


def _fmt(value, digits=1):
    return ('%+.' + str(digits) + 'f') % value


def clean_rows(rows, max_theta=60.0, teleport_deg=60.0):
    """把“起摆/倒摆”帧和“通信/复位跳变”分开统计。

    - 落在 |θ|>60° 或单帧位置变化过大（>60°）→ 起摆阶段，正常，剔除但不报警；
    - 单帧位置瞬移（>60° 且 dt<0.05 s）→ 真正的跳变（复位/掉线），计入 fault。

    返回 (清洗后的 rows, 起摆帧数, 跳变帧数)。
    """
    out = []
    fallen = 0
    fault = 0
    prev = None
    for r in rows:
        try:
            t, th, loc = float(r[0]), float(r[1]), float(r[2])
        except (TypeError, ValueError):
            fallen += 1
            continue
        if abs(th) > max_theta:
            fallen += 1
            prev = None
            continue
        if prev is not None:
            dt = t - prev[0]
            jump = abs(loc - prev[1])
            if 0 < dt < 0.05 and jump > teleport_deg:
                fault += 1
                prev = None
                continue
            if jump > max_theta and dt > 0:
                fallen += 1
                prev = (t, loc)
                continue
        out.append(r)
        prev = (t, loc)
    return out, fallen, fault


def _median_filter_rows(rows, k=5):
    """对位置/摆杆做中值滤波（只用于报告指标，不改原始数据）。"""
    if len(rows) <= k:
        return rows
    half = k // 2
    out = []
    for i, r in enumerate(rows):
        lo = max(0, i - half)
        hi = min(len(rows), i + half + 1)
        loc = sorted(x[2] for x in rows[lo:hi])[(hi - lo) // 2]
        th = sorted(x[1] for x in rows[lo:hi])[(hi - lo) // 2]
        out.append((r[0], th, loc, r[3], r[4], r[5]))
    return out


def _speed_series(seg, idx, window=0.06):
    """按 ~60 ms 窗口做差分，得到较平滑的速度（单位/秒）。"""
    out = []
    j = 0
    for i in range(len(seg)):
        while j < i and seg[i][0] - seg[j][0] > window:
            j += 1
        dt = seg[i][0] - seg[j][0]
        out.append(0.0 if dt < 0.02 else (seg[i][idx] - seg[j][idx]) / dt)
    return out


def analyze_move(rows, start_t=None, end_t=None):
    """从波形数据里量一次定位的性能指标。

    rows: [(t, theta_deg, loc_deg, tgt_deg, pwm, state), ...]，t 为秒。
    以最后一次目标变化为起点；没有目标变化时按“稳态”统计。
    返回 dict（含 seg 供画图）；样本不足返回 None。
    """
    if len(rows) < 30:
        return None
    rows, fallen, fault = clean_rows(rows)
    if len(rows) < 30:
        return None
    rows = _median_filter_rows(rows)
    # 有明确的“本次定位下发时间”就用它，最准；没有才靠启发式。
    if end_t is not None:
        rows = [r for r in rows if r[0] <= end_t]
        if len(rows) < 30:
            return None
    used_start = False
    if start_t is not None:
        picked = [r for r in rows if r[0] >= start_t - 0.05]
        if len(picked) >= 30:
            rows = picked
            used_start = True
    # 选“最后一次真正的定位”：从末尾往回走——
    # 目标单调变化的那一段就是一次定位；走过 0.8 s 的“平台”说明到上一条之前了。
    if used_start:
        start = 0
    else:
        n = len(rows)
        tgt_end = rows[-1][3]
        i = n - 1
        while i > 0 and abs(rows[i - 1][3] - tgt_end) <= 1.0:
            i -= 1
        settle_from = i
        look = max(0, i - 20)
        direction = 0.0
        if abs(rows[i][3] - rows[look][3]) > 2.0:
            direction = 1.0 if rows[i][3] > rows[look][3] else -1.0
        j = i
        still = 0.0
        while j > 1:
            d = rows[j][3] - rows[j - 1][3]
            if direction > 0 and d < -1.5:
                break
            if direction < 0 and d > 1.5:
                break
            if direction == 0 and abs(d) > 1.5:
                break
            if abs(d) <= 0.5:
                still += rows[j][0] - rows[j - 1][0]
                if still > 0.8:
                    break
            else:
                still = 0.0
            j -= 1
        start = max(0, j - 1)
        if settle_from - start < 5 or abs(rows[settle_from][3] - rows[start][3]) < 5.0:
            changes = []
            for k in range(1, n):
                d = abs(rows[k][3] - rows[k - 1][3])
                if d > 1.0:
                    changes.append((k, d))
            big = [c for c in changes if c[1] >= 5.0]
            start = big[-1][0] if big else (max(changes, key=lambda c: c[1])[0]
                                            if changes else 0)
    seg = rows[start:]
    t0 = seg[0][0]
    tgt = seg[-1][3]
    loc0 = seg[0][2]
    step = tgt - loc0
    out = {'t0': t0, 'step': step, 'target': tgt, 'loc0': loc0,
           't_end': seg[-1][0] - t0, 'frames': len(seg), 'seg': seg,
           'theta_max': max(abs(r[1]) for r in seg),
           'dropped': fallen + fault, 'fallen': fallen, 'fault': fault}

    # 速度与速度误差（参考 = 目标的变化率，实际 = 位置的变化率）
    v_ref = _speed_series(seg, 3)
    v_act = _speed_series(seg, 2)
    v_err = [a - b for a, b in zip(v_act, v_ref)]
    out['v_ref'] = v_ref
    out['v_act'] = v_act
    out['v_err'] = v_err

    # 静止段（最后 2 s）：位置/摆杆峰峰值、摆动频率、电机 RMS
    tail_no = [r for r in seg if r[0] >= seg[-1][0] - 2.0]
    if len(tail_no) >= 10:
        lp = [r[2] for r in tail_no]
        tp = [r[1] for r in tail_no]
        pw = [r[4] for r in tail_no]
        tail_f = [r for r in seg if r[0] >= seg[-1][0] - 4.0] or tail_no
        lf = [r[2] for r in tail_f]
        mean = sum(lf) / len(lf)
        dur = max(0.5, tail_f[-1][0] - tail_f[0][0])
        cross = sum(1 for a, b in zip(lf, lf[1:])
                    if (a - mean) * (b - mean) < 0)
        tail_half = [r for r in seg if r[0] >= seg[-1][0] - 0.5]
        moving_tail = (len(tail_half) > 2 and
                       abs(tail_half[-1][2] - tail_half[0][2]) > 1.5)
        out.update({'rest_pkpk': max(lp) - min(lp),
                    'rest_theta': max(tp) - min(tp),
                    'rest_valid': not moving_tail,
                    # 偏移量 = 静止段的平均值（相对目标 / 相对摆杆零位）
                    'rest_off': sum(r[2] - r[3] for r in tail_no) / len(tail_no),
                    'rest_th_off': sum(tp) / len(tp),
                    'rest_freq': cross / 2.0 / dur,
                    'rest_pwm': (sum(v * v for v in pw) / len(pw)) ** 0.5})
    else:
        out.update({'rest_pkpk': 0.0, 'rest_theta': 0.0,
                    'rest_off': 0.0, 'rest_th_off': 0.0,
                    'rest_freq': 0.0, 'rest_pwm': 0.0, 'rest_valid': False})

    if abs(step) < 1.0:                      # 没有移动：只算稳态
        tail = [r for r in seg if r[0] >= seg[-1][0] - 0.5]
        out.update({'moved': False, 'overshoot': 0.0, 'peak_t': 0.0,
                    'settle_t': 0.0, 'band': 0.0, 'peak_v': 0.0,
                    'v_err_rms': 0.0, 'v_err_max': 0.0,
                    'ss_err': sum(r[2] - r[3] for r in tail) / max(1, len(tail))})
        return out
    sign = 1.0 if step > 0 else -1.0
    over, peak_t = 0.0, 0.0
    for r in seg:
        e = (r[2] - tgt) * sign
        if e > over:
            over, peak_t = e, r[0] - t0
    band = max(0.5, abs(step) * 0.02)        # ±2%，最小 ±0.5°
    idx = len(seg) - 1
    while idx > 0 and abs(seg[idx][2] - seg[idx][3]) <= band:
        idx -= 1
    settle_t = seg[min(len(seg) - 1, idx + 1)][0] - t0
    tail = [r for r in seg if r[0] >= seg[-1][0] - 0.5]
    ss = sum(r[2] - r[3] for r in tail) / max(1, len(tail))
    pv = 0.0
    for a, b in zip(seg, seg[1:]):
        dt = b[0] - a[0]
        if 0 < dt < 0.2:
            pv = max(pv, abs(b[2] - a[2]) / dt)
    # 速度误差只统计“运动段”（起点到调节完成 + 0.3 s）
    move_err = [e for r, e in zip(seg, v_err) if r[0] - t0 <= settle_t + 0.3]
    if move_err:
        vrms = (sum(e * e for e in move_err) / len(move_err)) ** 0.5
        vmax = max(abs(e) for e in move_err)
    else:
        vrms = vmax = 0.0
    out.update({'moved': True, 'overshoot': over, 'peak_t': peak_t,
                'settle_t': settle_t, 'band': band, 'peak_v': pv, 'ss_err': ss,
                'v_err_rms': vrms, 'v_err_max': vmax})
    return out


class SerialWorker(threading.Thread):
    """串口读线程：只负责收字节、解帧、把事件塞进队列。"""

    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.queue = out_queue
        self.stop_event = threading.Event()
        self.ser = None
        self.lock = threading.Lock()
        self.decoder = P.Decoder()

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05,
                                     write_timeout=1.0)
        except Exception as exc:
            text = str(exc)
            low = text.lower()
            if 'permission' in low or 'access' in low or '拒绝访问' in text:
                hint = ('%s 被别的程序占用了（常见：还开着一个旧上位机 / 串口助手 / '
                        'Arduino 监视器）。把那些窗口全关掉再点“连接”。' % self.port)
            elif 'could not open' in low or 'filenotfound' in low:
                hint = ('系统里没有 %s 这个串口：拔插一下板子的 USB，点“刷新”看'
                        '列表里有没有新的 COM 口；换根数据线（有的线只能充电）。'
                        % self.port)
            else:
                hint = '打开 %s 失败：%s' % (self.port, text)
            self.queue.put(('error', hint))
            return
        self.queue.put(('open', self.port))
        try:
            while not self.stop_event.is_set():
                waiting = self.ser.in_waiting
                data = self.ser.read(waiting if waiting else 1)
                if not data:
                    continue
                for frame in self.decoder.feed(data):
                    self.queue.put(('frame', frame))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.queue.put(('error', '串口读取中断：%s' % exc))
        finally:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.queue.put(('closed', None))

    def send(self, data):
        with self.lock:
            if self.ser is None:
                raise IOError('串口未打开')
            if self.ser.write(data) != len(data):
                raise IOError('串口未完整写出')

    def stop(self):
        self.stop_event.set()


class TrackView:
    """速度曲线页的实时跟踪图：上=位置/目标，下=位置误差（放大看间隙）。"""

    NICE = (1, 2, 5, 10, 20, 30, 45, 60, 90, 120, 180, 240,
            360, 540, 720, 1080, 1440, 2160, 3600)

    def __init__(self, canvas, app):
        self.canvas = canvas
        self.app = app
        self.span = 12.0

    @classmethod
    def _nice(cls, value, minimum=1.0):
        for n in cls.NICE:
            if n >= max(minimum, value):
                return float(n)
        return float(cls.NICE[-1])

    def draw(self):
        c = self.canvas
        c.delete('all')
        wave = getattr(self.app, 'wave', None)
        if wave is None:
            return
        data = list(wave.frozen) if wave.paused else list(wave.data)
        w, h = c.winfo_width(), c.winfo_height()
        if w <= 1:
            w = c.winfo_reqwidth()
        if h <= 1:
            h = c.winfo_reqheight()
        if w < 160 or h < 120:
            return
        if len(data) < 2:
            c.create_text(w / 2, h / 2, text='等待数据：连上板子走一次定位就能看到',
                          fill=COL_MUTED, font=('Microsoft YaHei UI', 9))
            return
        now = data[-1][0]
        t0 = now - self.span
        vis = [r for r in data if r[0] >= t0]
        if len(vis) < 2:
            return
        left, right, top, bottom, gap = 58, 66, 12, 22, 12
        avail = h - top - bottom - gap
        lane_a = avail * 0.56
        lane_b = avail * 0.44

        def x_of(t):
            return left + (t - t0) / self.span * (w - left - right)

        # ---------------- 上：位置 / 目标 ----------------
        ya0, ya1 = top, top + lane_a
        ref = max([abs(r[2]) for r in vis] + [abs(r[3]) for r in vis] + [10.0])
        span_a = self._nice(ref * 1.12)

        def y_a(v):
            return (ya0 + ya1) / 2.0 - v / span_a * (lane_a / 2.0 - 4)

        c.create_rectangle(left, ya0, w - right, ya1, outline=COL_GRID)
        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
            yy = y_a(frac * span_a)
            c.create_line(left, yy, w - right, yy,
                          fill=COL_GRID if frac == 0 else '#eef1f4',
                          dash=() if frac == 0 else (2, 4))
            c.create_text(left - 5, yy, text='%g' % (frac * span_a), anchor='e',
                          fill=COL_MUTED, font=('Consolas', 8))
        c.create_text(left + 4, ya0 + 3, text='转臂位置 / 目标 (°)', anchor='nw',
                      fill=COL_MUTED, font=('Microsoft YaHei UI', 9))

        pts = []
        for r in vis:
            pts.extend((x_of(r[0]), y_a(r[2])))
        if len(pts) >= 4:
            c.create_line(*pts, fill=COL_LOC, width=2)
        pts = []
        for r in vis:
            pts.extend((x_of(r[0]), y_a(r[3])))
        if len(pts) >= 4:
            c.create_line(*pts, fill=COL_TGT, dash=(6, 4), width=2)
        last = vis[-1]
        c.create_text(w - right + 4, y_a(last[2]), anchor='w',
                      text='%+.1f°' % last[2], fill=COL_LOC,
                      font=('Consolas', 9, 'bold'))

        # ---------------- 下：位置误差（放大） ----------------
        yb0, yb1 = top + lane_a + gap, top + lane_a + gap + lane_b
        errs = [r[2] - r[3] for r in vis]
        span_b = self._nice(max(1.0, max(abs(e) for e in errs) * 1.3), 1.0)

        def y_b(v):
            return (yb0 + yb1) / 2.0 - v / span_b * (lane_b / 2.0 - 4)

        c.create_rectangle(left, yb0, w - right, yb1, outline=COL_GRID)
        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
            yy = y_b(frac * span_b)
            c.create_line(left, yy, w - right, yy,
                          fill=COL_GRID if frac == 0 else '#eef1f4',
                          dash=() if frac == 0 else (2, 4))
            c.create_text(left - 5, yy, text='%g' % (frac * span_b), anchor='e',
                          fill=COL_MUTED, font=('Consolas', 8))
        c.create_text(left + 4, yb0 + 3, text='位置误差 = 实际 − 目标 (°)，纵轴已放大', anchor='nw',
                      fill=COL_MUTED, font=('Microsoft YaHei UI', 9))

        pts = []
        for r, e in zip(vis, errs):
            pts.extend((x_of(r[0]), y_b(e)))
        if len(pts) >= 4:
            c.create_line(*pts, fill='#bc4c00', width=2)
        c.create_text(w - right + 4, y_b(errs[-1]), anchor='w',
                      text='%+.2f°' % errs[-1], fill='#bc4c00',
                      font=('Consolas', 9, 'bold'))

        for frac in (-1, -0.5, 0, 0.5, 1):
            tt = t0 + (frac + 1) / 2.0 * self.span
            c.create_text(x_of(tt), h - 8, text='%.1fs' % (tt - now), anchor='s',
                          fill=COL_MUTED, font=('Consolas', 8))


class WaveView:
    """三通道实时波形：角度偏差 / 转臂位置与目标 / 电机指令。"""

    def __init__(self, canvas):
        self.canvas = canvas
        self.data = collections.deque(maxlen=DATA_MAX)
        self.span = 15.0
        self.paused = False
        self.frozen = []
        self.frozen_end = 0.0

    def add(self, frame):
        center = frame.get('angle_center') or 2971
        theta = (frame['theta'] - center) * P.DEG_PER_COUNT
        loc = P.counts_to_deg(frame['location'])
        tgt = P.counts_to_deg(frame['target'])
        self.data.append((frame['host_time'], theta, loc, tgt,
                          frame['pwm'], frame['state']))

    def clear(self):
        self.data.clear()
        self.canvas.delete('all')

    def latest(self):
        return self.data[-1] if self.data else None

    def draw(self):
        if self.paused:
            self._render(self.frozen, self.frozen_end)
            return
        if not self.data:
            self.canvas.delete('all')
            return
        self._render(list(self.data), self.data[-1][0])

    def _render(self, data, now):
        c = self.canvas
        c.delete('all')
        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            w = c.winfo_reqwidth()
        if h <= 1:
            h = c.winfo_reqheight()
        if w < 120 or h < 120 or not data:
            return
        left, right, top, bottom, gap = 62, 78, 16, 24, 16
        lane_h = (h - top - bottom - 2 * gap) / 3.0
        t_end = now
        t_start = t_end - self.span
        visible = [row for row in data if row[0] >= t_start]
        if len(visible) > 1500:
            step = len(visible) // 1500 + 1
            visible = visible[::step]
        if not visible:
            return

        def x_of(t):
            return left + (t - t_start) / self.span * (w - left - right)

        lanes = (
            ('摆杆角度偏差 (°)', 1, 20.0, COL_THETA, 1),
            ('转臂位置 / 目标 (°)', 2, 20.0, COL_LOC, 2),
            ('电机指令 PWM (±255)', 4, 255.0, COL_PWM, 3),
        )
        nice = (10, 20, 30, 45, 60, 90, 120, 180, 240, 360, 540, 720,
                1080, 1440, 2160, 3600)

        def nice_span(v):
            for n in nice:
                if n >= v:
                    return float(n)
            return float(nice[-1])

        for title, idx, default_range, color, order in lanes:
            y0 = top + (order - 1) * (lane_h + gap)
            y1 = y0 + lane_h
            values = [row[idx] for row in visible]
            if idx == 2:
                ref = max([abs(v) for v in values] + [abs(row[3]) for row in visible] + [10.0])
                span_y = nice_span(max(10.0, ref * 1.15))
            elif idx == 1:
                span_y = min(180.0, max(default_range, max(abs(v) for v in values) * 1.15))
            else:
                span_y = 255.0

            def y_of(v, y0=y0, y1=y1, span_y=span_y):
                return (y0 + y1) / 2.0 - v / span_y * (lane_h / 2.0 - 2)

            c.create_rectangle(0, y0 - 3, w, y1 + 3, fill=COL_PANEL, outline='')
            for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
                yy = y_of(frac * span_y)
                c.create_line(left, yy, w - right, yy,
                              fill=COL_GRID if frac == 0 else '#eaeef2',
                              dash=() if frac == 0 else (2, 4))
                c.create_text(left - 6, yy, text='%g' % (frac * span_y),
                              anchor='e', fill=COL_MUTED,
                              font=('Consolas', 8))
            c.create_text(left, y0 + 2, text=title, anchor='nw',
                          fill=COL_MUTED, font=('Microsoft YaHei UI', 8))

            if idx == 2:
                # 先画实际，再把目标画在上面：两者重合时目标虚线仍然可见
                pts = []
                for row in visible:
                    pts.extend((x_of(row[0]), y_of(row[2])))
                if len(pts) >= 4:
                    c.create_line(*pts, fill=color, width=2)
                pts = []
                for row in visible:
                    pts.extend((x_of(row[0]), y_of(row[3])))
                if len(pts) >= 4:
                    c.create_line(*pts, fill=COL_TGT, dash=(6, 4), width=2)
            else:
                pts = []
                for row in visible:
                    pts.extend((x_of(row[0]), y_of(row[idx])))
                if len(pts) >= 4:
                    c.create_line(*pts, fill=color, width=2)

            last = visible[-1]
            label = ('%+.1f°' % last[idx]) if idx != 4 else ('%+d' % int(last[4]))
            # 右上角白底数值，避免和波形线重叠
            lx1, ly1 = w - right - 6, y0 + 4
            c.create_rectangle(lx1 - 76, ly1, lx1, ly1 + 17,
                               fill=COL_PANEL, outline='')
            c.create_text(lx1 - 4, ly1 + 8, text=label, anchor='e',
                          fill=color, font=('Consolas', 10, 'bold'))

        axis_y = h - bottom + 10
        c.create_line(left, h - bottom, w - right, h - bottom, fill=COL_GRID)
        for i in range(6):
            frac = i / 5.0
            xx = left + frac * (w - left - right)
            age = self.span * (1 - frac)
            c.create_text(xx, axis_y, text='-%.0fs' % age if age > 0.5 else '现在',
                          fill=COL_MUTED, font=('Consolas', 8))


class DemoBoard:
    """没有硬件时用的假板子：一阶位置响应 + 16 点曲线 + 100 Hz 遥测 + 命令回执。"""

    def __init__(self):
        self.location = 0.0
        self.target = P.deg_to_counts(90)
        self.step = 2
        self.pwm = 0
        self.frames = 0
        self.center = 2971
        self.cmd_seq = 0
        self.result = 0
        self.active_seq = 0
        self.pending_seq = 0
        self.pending_ticks = 0
        self.profile = list(P.CURVE_FLAT)
        self.move_start = 0.0
        self.jog = 0

    def apply(self, packet):
        self.cmd_seq = struct.unpack('<H', packet[4:6])[0]
        operation = packet[6]
        value = struct.unpack('<i', packet[8:12])[0]
        if operation == 4:
            self.target = value
            self.move_start = self.location
        elif operation == 5:
            self.step = max(P.STEP_MIN, value)
        elif operation == 6:
            index = max(0, min(P.PROFILE_POINTS - 1, value))
            point = max(P.CURVE_MIN, min(P.CURVE_MAX,
                                         struct.unpack('<i', packet[12:16])[0]))
            self.profile[index] = point
        elif operation == 7:
            self.jog = max(P.JOG_MIN, min(P.JOG_MAX, value))
        self.result = 1
        self.pending_seq = self.cmd_seq
        self.pending_ticks = 5

    def frame(self):
        if self.pending_ticks:
            self.pending_ticks -= 1
            if not self.pending_ticks:
                self.result = 2
                self.active_seq = self.pending_seq
        if self.jog:
            self.location += self.jog * 0.4
            self.target = int(round(self.location))
            self.move_start = self.location
        # 每 4 秒左右自动换向，方便只看界面时也有波形。
        elif self.frames and self.frames % 300 == 0:
            self.target = P.deg_to_counts(90) if self.target <= 0 else -P.deg_to_counts(90)
            self.move_start = self.location
        err = self.target - self.location
        span = self.target - self.move_start
        if abs(span) > 1:
            progress = (self.location - self.move_start) / span
            index = max(0, min(P.PROFILE_POINTS - 1, int(progress * P.PROFILE_POINTS)))
            scale = max(0.04, self.profile[index] / float(P.CURVE_MAX))
        else:
            scale = 1.0
        per_tick = self.step * 0.4 * max(0.04, scale)
        if abs(err) <= per_tick:
            self.location = float(self.target)
        else:
            self.location += math.copysign(per_tick, err)
        self.pwm = int(max(-220, min(220, err * 0.8)))
        self.frames += 1
        theta = int(self.center + 4 * math.sin(self.frames * 0.21)
                    + random.uniform(-1.5, 1.5))
        return {
            'format': 'trace_v3_100hz',
            'sequence': self.frames & 0xFFFF,
            'firmware_id': 0x26101801,
            'fpga_ms': (self.frames * 10) & 0xFFFFFFFF,
            'state': 4, 'manual_mode': 1, 'flags': 0x0001,
            'encoder_absolute': 10000,
            'arm_relative_counts': int(round(self.location)),
            'position_target_counts': self.target,
            'theta_raw': theta, 'theta_filtered': theta,
            'theta_min': theta - 3, 'theta_max': theta + 3,
            'angle_center': self.center,
            'pwm': self.pwm, 'requested_pwm': self.pwm,
            'pid_command_sequence': self.cmd_seq,
            'pid_active_sequence': self.active_seq,
            'pid_command_result': self.result,
            'pid_command_bank': 1,
            'host_time': time.monotonic(),
            'theta': theta,
            'location': int(round(self.location)),
            'target': self.target,
        }


class DemoWorker:
    def __init__(self, board):
        self.board = board

    def send(self, data):
        self.board.apply(bytes(data))

    def stop(self):
        pass


class Joystick(tk.Canvas):
    """摇杆：左右拖动给速度，松手回中。"""

    def __init__(self, master, on_change, radius=78, **kw):
        super().__init__(master, width=radius * 2 + 40, height=radius * 2 + 40,
                         bg=COL_PANEL, highlightthickness=0, **kw)
        self.on_change = on_change
        self.R = radius
        self.cx = radius + 20
        self.cy = radius + 20
        self.dx = 0.0
        self.dy = 0.0
        self._dragging = False
        self._redraw()
        self.bind('<Button-1>', self._press)
        self.bind('<B1-Motion>', self._move)
        self.bind('<ButtonRelease-1>', self._release)

    def _redraw(self):
        self.delete('all')
        r = self.R
        self.create_oval(self.cx - r, self.cy - r, self.cx + r, self.cy + r,
                         outline=COL_GRID, fill='#ffffff', width=2)
        self.create_line(self.cx - r, self.cy, self.cx + r, self.cy, fill='#eaeef2')
        self.create_line(self.cx, self.cy - r, self.cx, self.cy + r, fill='#eaeef2')
        self.create_oval(self.cx - 3, self.cy - 3, self.cx + 3, self.cy + 3,
                         fill='#8b949e', outline='')
        kr = 15
        kx = self.cx + self.dx * (r - kr)
        ky = self.cy + self.dy * (r - kr)
        self.create_oval(kx - kr, ky - kr, kx + kr, ky + kr,
                         fill=COL_ACCENT, outline='#0550ae', width=2)

    def _set(self, event):
        x = event.x - self.cx
        y = event.y - self.cy
        d = (x * x + y * y) ** 0.5
        if d == 0:
            self.dx = self.dy = 0.0
        else:
            s = min(d, self.R) / self.R
            self.dx = x / d * s
            self.dy = y / d * s
        self._redraw()
        self.on_change(self.dx, self.dy)

    def _press(self, event):
        self._dragging = True
        self._set(event)

    def _move(self, event):
        if self._dragging:
            self._set(event)

    def _release(self, _event):
        self._dragging = False
        self.reset()

    def reset(self):
        self.dx = self.dy = 0.0
        self._redraw()
        self.on_change(0.0, 0.0)


class CurveEditor(tk.Canvas):
    """16 点速度曲线：折线 + 圆点 + 面积，值 -256~256（负数 = 反向制动）。"""

    def __init__(self, master, on_change=None, **kw):
        super().__init__(master, bg='white', bd=0, highlightthickness=1,
                         highlightbackground=COL_GRID, **kw)
        self.on_change = on_change
        self.vals = list(P.CURVE_FLAT)
        self._drag = None
        self.bind('<Configure>', lambda _e: self._redraw())
        self.bind('<Button-1>', self._press)
        self.bind('<B1-Motion>', self._move)
        self.bind('<ButtonRelease-1>', self._release)
        self._redraw()

    def _geom(self):
        w = max(self.winfo_width(), 320)
        h = max(self.winfo_height(), 220)
        return 52, w - 18, 16, h - 28

    def _x_at(self, i, x1, x2):
        return x1 + (x2 - x1) * i / float(P.PROFILE_POINTS - 1)

    def _y_at(self, value, y1, y2):
        mid = (y1 + y2) / 2.0
        return mid - value / float(P.CURVE_MAX) * (y2 - y1) / 2.0

    def _index_at(self, x, x1, x2):
        if x2 <= x1:
            return 0
        i = int(round((x - x1) / float(x2 - x1) * (P.PROFILE_POINTS - 1)))
        return max(0, min(P.PROFILE_POINTS - 1, i))

    def _value_at(self, y, y1, y2):
        if y2 <= y1:
            return 0
        mid = (y1 + y2) / 2.0
        value = (mid - y) / ((y2 - y1) / 2.0) * P.CURVE_MAX
        return max(P.CURVE_MIN, min(P.CURVE_MAX, int(round(value))))

    def _redraw(self):
        self.delete('all')
        x1, x2, y1, y2 = self._geom()
        mid = (y1 + y2) / 2.0
        for value in (P.CURVE_MAX, P.CURVE_MAX // 2, 0, P.CURVE_MIN // 2, P.CURVE_MIN):
            yy = self._y_at(value, y1, y2)
            self.create_line(x1, yy, x2, yy,
                             fill='#d0d7de' if value == 0 else '#eaeef2')
            self.create_text(x1 - 6, yy, text='%d' % value, anchor='e',
                             fill=COL_MUTED, font=('Consolas', 8))
        for i in range(P.PROFILE_POINTS):
            xx = self._x_at(i, x1, x2)
            self.create_line(xx, y1, xx, y2, fill='#f3f5f7')
        pts = []
        for i, value in enumerate(self.vals):
            pts.extend((self._x_at(i, x1, x2), self._y_at(value, y1, y2)))
        if len(pts) >= 4:
            self.create_polygon(pts[0], mid, *pts, pts[-2], mid,
                                fill='#ddf4ff', outline='')
            self.create_line(*pts, fill=COL_ACCENT, width=2)
        for i, value in enumerate(self.vals):
            xx = self._x_at(i, x1, x2)
            yy = self._y_at(value, y1, y2)
            r = 5 if self._drag == i else 3.5
            self.create_oval(xx - r, yy - r, xx + r, yy + r,
                             fill=COL_ACCENT if self._drag == i else 'white',
                             outline=COL_ACCENT, width=2)
        self.create_text((x1 + x2) / 2.0, y2 + 12,
                         text='行程 0% → 100%（负数 = 反向制动）',
                         fill=COL_MUTED, font=('Microsoft YaHei UI', 8))
        if self._drag is not None:
            self.create_text(x1, y1 - 4, anchor='sw',
                             text='点 %d = %+d' % (self._drag, self.vals[self._drag]),
                             fill=COL_ACCENT, font=('Consolas', 9, 'bold'))

    def _press(self, event):
        x1, x2, y1, y2 = self._geom()
        self._drag = self._index_at(event.x, x1, x2)
        self.vals[self._drag] = self._value_at(event.y, y1, y2)
        self._redraw()
        self._notify()

    def _move(self, event):
        if self._drag is None:
            return
        x1, x2, y1, y2 = self._geom()
        self._drag = self._index_at(event.x, x1, x2)
        self.vals[self._drag] = self._value_at(event.y, y1, y2)
        self._redraw()
        self._notify()

    def _release(self, _event):
        self._drag = None
        self._redraw()

    def _notify(self):
        if self.on_change:
            self.on_change(list(self.vals))

    def get_vals(self):
        return list(self.vals)

    def set_vals(self, values):
        self.vals = P.clamp_curve(values)
        self._redraw()
        self._notify()

    def smooth(self):
        v = list(self.vals)
        out = list(v)
        for i in range(1, P.PROFILE_POINTS - 1):
            out[i] = int(round((v[i - 1] + 2 * v[i] + v[i + 1]) / 4.0))
        out[0] = int(round((v[0] + v[1]) / 2.0))
        out[-1] = int(round((v[-2] + v[-1]) / 2.0))
        self.set_vals(out)


class ScrollableFrame(ttk.Frame):
    """左侧控制栏：窗口较矮时可滚动，避免按钮被裁掉。"""

    def __init__(self, parent):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, bg=COL_BG, highlightthickness=0, bd=0)
        self.vsb = ttk.Scrollbar(self, orient='vertical', command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)
        self.vsb.pack(side='right', fill='y')
        self.canvas.pack(side='left', fill='both', expand=True)
        self.inner = ttk.Frame(self.canvas)
        self._win = self.canvas.create_window((0, 0), window=self.inner, anchor='nw')
        self.inner.bind('<Configure>', self._on_inner)
        self.canvas.bind('<Configure>', self._on_canvas)

    def _on_inner(self, _event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))

    def _on_canvas(self, event):
        self.canvas.itemconfigure(self._win, width=event.width)

    def _bind_wheel(self, _event=None):
        self.canvas.bind_all('<MouseWheel>', self._wheel)

    def _unbind_wheel(self, _event=None):
        self.canvas.unbind_all('<MouseWheel>')

    def _wheel(self, event):
        self.canvas.yview_scroll(int(-event.delta / 120) or 0, 'units')


class App:
    def __init__(self, root, port=None, demo=False):
        self.root = root
        self.root.title('%s v%s' % (APP_TITLE, APP_VERSION))
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        width = min(1360, int(sw * 0.94))
        height = min(940, int(sh * 0.92))
        root.geometry('%dx%d+%d+%d' % (width, height,
                                       max(0, (sw - width) // 2),
                                       max(0, (sh - height) // 3)))
        self.root.minsize(980, 620)
        self.root.configure(bg=COL_BG)

        self.queue = queue.Queue()
        self.worker = None
        self.link = P.Link()
        self.connected = False
        self.last_state = None
        self.frame_count = 0
        self.fps = 0.0
        self._fps_mark = time.monotonic()
        self._fps_frames = 0
        self.recording = False
        self.rec_file = None
        self.rec_writer = None
        self.rec_started = None
        self.rec_rows = 0
        self._curve_queue = collections.deque()
        self._curve_total = 0
        self._curve_active = False
        self._curve_synced = False
        self._curve_save_after = None

        self.var_port = tk.StringVar()
        self.var_status = tk.StringVar(value='未连接')
        self.var_fw = tk.StringVar(value='固件 —')
        self.var_state = tk.StringVar(value='—')
        self.var_mode = tk.StringVar(value='—')
        self.var_theta = tk.StringVar(value='—')
        self.var_pwm = tk.StringVar(value='—')
        self.var_pos_out = tk.StringVar(value='—')
        self.var_cur_deg = tk.StringVar(value='—')
        self.var_cur_cnt = tk.StringVar(value='')
        self.var_tgt_deg = tk.StringVar(value='—')
        self.var_tgt_cnt = tk.StringVar(value='')
        self.var_err_deg = tk.StringVar(value='')
        self.var_entry = tk.StringVar(value='0')
        self.var_speed = tk.StringVar(value='8')
        self.var_speed_desc = tk.StringVar(value='')
        self.applied_step = 8
        self.var_hint = tk.StringVar(value='连接后开始')
        self.var_rec = tk.StringVar(value='未记录')
        self.var_msg = tk.StringVar(value='就绪：选择串口并连接')
        self.var_wave_stats = tk.StringVar(value='')
        self.var_curve_status = tk.StringVar(value='未下载')
        self.var_curve_info = tk.StringVar(value='')
        self.var_curve_points = tk.StringVar(value='')
        self.var_jog_max = tk.StringVar(value='16')
        self.var_jog_invert = tk.BooleanVar(value=False)
        self.var_jog_desc = tk.StringVar(value='松手停在原地；仅平衡时有效')
        self.var_auto_rec = tk.BooleanVar(value=True)
        self.var_ff = tk.StringVar(value='0')
        self.var_ff_status = tk.StringVar(value='0% = 关闭')
        self.var_ff_cmd = tk.StringVar(value='0')
        self.var_ff_cmd_status = tk.StringVar(value='0% = 关闭')
        self.var_step90 = tk.StringVar(value='16')
        self.var_step360 = tk.StringVar(value='24')
        self.var_auto_step = tk.BooleanVar(value=True)
        self.var_curve_exec = tk.BooleanVar(value=True)
        self.var_curve_stream = tk.BooleanVar(value=True)
        self.var_curve_auto = tk.BooleanVar(value=True)
        self.var_curve_target = tk.StringVar(value='')
        self.var_curve_info2 = tk.StringVar(value='选一种曲线，填目标角度后点“执行曲线”')
        self._curve_plan = None
        self._curve_job = None
        self._batch = None
        self._curve_undo = []
        self.var_deadzone = tk.StringVar(value='5')
        self.var_deadzone_status = tk.StringVar(value='默认 5')
        self.var_vp = tk.StringVar(value='0')
        self.var_vp_status = tk.StringVar(value='0% = 关闭')
        # PID / 中值 / 摩擦前馈（参数页）
        # 初值直接取源码 pendulum_ctrl.v 的编译期参数，避免“写入”把调好的增益改坏。
        self.src_pid = source_pid_defaults()
        src = self.src_pid
        self.var_pid_bank = tk.IntVar(value=0)
        self.var_pid_a_kp = tk.StringVar(value=str(src.get('KP_A_AUTO', 1750)))
        self.var_pid_a_ki = tk.StringVar(value=str(src.get('KI_A_AUTO', 60)))
        self.var_pid_a_kd = tk.StringVar(value=str(src.get('KD_A_AUTO', 27000)))
        self.var_pid_p_kp = tk.StringVar(value=str(src.get('KP_P_AUTO', 1400)))
        self.var_pid_p_ki = tk.StringVar(value=str(src.get('KI_P_AUTO', 10)))
        self.var_pid_p_kd = tk.StringVar(value=str(src.get('KD_P_AUTO', 5509)))
        self.var_pid_status = tk.StringVar(
            value='初值 = 源码参数（未写回）' if src else '没找到源码，初值可能过期')
        self.var_pid_diff = tk.StringVar(value='')
        self.var_preset_name = tk.StringVar(value='')
        self.var_preset_info = tk.StringVar(value='')
        self._presets = []
        self.var_center = tk.StringVar(value='2957')
        self.var_vp = tk.StringVar(value='0')
        self.var_ff_gain = tk.StringVar(value='0')
        self.var_ff_gain_status = tk.StringVar(value='0 = 关闭')
        self.var_vp_status = tk.StringVar(value='0% = 关闭')
        self.var_center = tk.StringVar(value='2965')
        self.var_center_status = tk.StringVar(value='')
        self.var_ks = tk.StringVar(value='0')
        self.var_ks_status = tk.StringVar(value='0 = 关闭')
        self.var_param_now = tk.StringVar(value='等待遥测…')
        self.last_frame = None
        self.var_damp = tk.StringVar(value='100')
        self._curve_history = []
        self._curve_last = None
        self._undoing = False
        self._jog_pending = 0
        self._jog_sent = 0
        self._jog_timer = None
        self._jog_target = None
        self._jog_moved = False
        self._jog_after = None
        self._move_queue = []
        self._board_step = 8
        self._restore_step = False
        self._settle_count = 0
        self.last_location = None
        self._arm_hist = collections.deque(maxlen=40)
        self._move_wait_start = None

        self._build_style()
        self._build_ui()
        self._presets = self._load_presets()
        self._refresh_preset_list()
        self._refresh_ports(select=port)
        self._load_preset_file()          # 把调好的参数填进界面（不自动下发）
        self._bind_keys()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._update_speed_desc()
        self._poll_id = self.root.after(POLL_MS, self._poll)
        self._draw_id = self.root.after(DRAW_MS, self._draw)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.demo_board = None
        self._restore_session(keep_port=port is not None)
        self._refresh_saved_curves()
        if demo:
            self._start_demo()

    # ------------------------------------------------------------ 演示模式
    def _start_demo(self):
        self.demo_board = DemoBoard()
        self.worker = DemoWorker(self.demo_board)
        self.connected = True
        self.var_status.set('演示模式（无硬件）')
        self.btn_conn.configure(state='disabled', text='演示中')
        self._log('演示模式：模拟 100 Hz 遥测、位置斜坡与命令回执')
        self.root.after(10, self._demo_tick)
        # 演示里直接摆一条梯形曲线并走一遍下载流程，方便看效果。

    def _demo_tick(self):
        if self.demo_board is None or self.worker is None:
            return
        self._on_frame(self.demo_board.frame())
        self.root.after(10, self._demo_tick)

    # ------------------------------------------------------------ 外观
    def _build_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use('vista')
        except tk.TclError:
            pass
        base = ('Microsoft YaHei UI', 9)
        style.configure('.', font=base)
        style.configure('TFrame', background=COL_BG)
        style.configure('Card.TFrame', background=COL_PANEL)
        style.configure('TLabel', background=COL_BG, foreground=COL_TEXT)
        style.configure('Card.TLabel', background=COL_PANEL, foreground=COL_TEXT)
        style.configure('Key.TButton', font=('Microsoft YaHei UI', 14), padding=6)
        style.configure('Go.TButton', font=('Microsoft YaHei UI', 14, 'bold'),
                        padding=6)
        style.configure('Muted.TLabel', background=COL_PANEL, foreground=COL_MUTED,
                        font=('Microsoft YaHei UI', 8))
        style.configure('Big.TLabel', background=COL_PANEL, foreground=COL_TEXT,
                        font=('Consolas', 26, 'bold'))
        style.configure('Good.TLabel', background=COL_PANEL, foreground=COL_OK)
        style.configure('Warn.TLabel', background=COL_PANEL, foreground=COL_WARN)
        style.configure('TLabelframe', background=COL_PANEL)
        style.configure('TLabelframe.Label', background=COL_PANEL,
                        foreground=COL_TEXT, font=('Microsoft YaHei UI', 9, 'bold'))

    # ------------------------------------------------------------ 界面
    def _draw(self):
        try:
            self.wave.draw()
            try:
                if (hasattr(self, 'track')
                        and self.notebook.index(self.notebook.select()) == 2):
                    self.track.draw()
            except Exception:
                pass
        finally:
            self._draw_id = self.root.after(DRAW_MS, self._draw)

    def _toggle_pause(self):
        self.wave.paused = not self.wave.paused
        if self.wave.paused:
            self.wave.frozen = list(self.wave.data)
            self.wave.frozen_end = (self.wave.data[-1][0] if self.wave.data
                                    else time.monotonic())
            self.btn_pause.configure(text='恢复显示')
        else:
            self.btn_pause.configure(text='暂停显示')

    def _on_span(self, _evt=None):
        try:
            self.wave.span = float(self.cmb_span.get())
        except ValueError:
            self.wave.span = 15.0

    def _clear_wave(self):
        self.wave.clear()
        self.var_wave_stats.set('已清空')

    def _rec_dir(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rec')

    def _open_rec_dir(self):
        directory = self._rec_dir()
        os.makedirs(directory, exist_ok=True)
        try:
            os.startfile(directory)
        except Exception as exc:
            messagebox.showerror('无法打开', str(exc))

    # ------------------------------------------------------------ 实验报告
    def _toggle_record(self):
        if self.recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        directory = self._rec_dir()
        os.makedirs(directory, exist_ok=True)
        name = 'pos_%s.csv' % _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(directory, name)
        self.rec_file = open(path, 'w', newline='', encoding='utf-8-sig')
        self.rec_writer = csv.writer(self.rec_file)
        self.rec_writer.writerow(['host_time', 'elapsed_s', 'state', 'mode',
                                  'theta_deg', 'location', 'target', 'pwm',
                                  'angle_center', 'note', 'pos_output'])
        self.rec_started = time.monotonic()
        self.rec_rows = 0
        self.rec_path = path
        self.recording = True
        self.btn_rec.configure(text='停止记录')
        self.var_rec.set('记录中 ' + name.replace('pos_', '').replace('.csv', ''))
        self._log('开始记录 %s' % path)

    def _stop_record(self):
        self.recording = False
        if self.rec_file is not None:
            try:
                self.rec_file.close()
            except Exception:
                pass
        self.rec_file = None
        self.rec_writer = None
        self.btn_rec.configure(text='开始记录')
        self.var_rec.set('已记录 %d 行' % self.rec_rows)
        self._log('记录结束，共 %d 行' % self.rec_rows)

    def _write_row(self, frame):
        if self.rec_writer is None:
            return
        center = frame.get('angle_center') or 2971
        self.rec_writer.writerow([
            _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3],
            '%.3f' % (time.monotonic() - self.rec_started),
            frame['state'], frame['manual_mode'],
            '%.3f' % ((frame['theta'] - center) * P.DEG_PER_COUNT),
            frame['location'], frame['target'], frame['pwm'], center, '',
            frame.get('position_output_q8', '')])
        self.rec_rows += 1
        if self.rec_rows % 100 == 0:
            self.rec_file.flush()

    def _rec_note(self, text):
        if not self.recording or self.rec_writer is None:
            return
        self.rec_writer.writerow([
            _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3],
            '%.3f' % (time.monotonic() - self.rec_started),
            '', '', '', '', '', '', '', text, ''])
        self.rec_file.flush()

    def _log(self, text):
        line = '%s  %s\n' % (time.strftime('%H:%M:%S'), text)
        self.txt_log.configure(state='normal')
        self.txt_log.insert('end', line)
        if int(self.txt_log.index('end-1c').split('.')[0]) > LOG_MAX_LINES:
            self.txt_log.delete('1.0', '200.0')
        self.txt_log.see('end')
        self.txt_log.configure(state='disabled')

    def _set_msg(self, text):
        self.var_msg.set('%s  %s' % (time.strftime('%H:%M:%S'), text))

    def _bind_keys(self):
        for sequence in ('<Left>', '<Right>', '<Shift-Left>', '<Shift-Right>',
                         '<Prior>', '<Next>', '<space>', '<Escape>', 'z', 'Z'):
            self.root.bind(sequence, self._on_shortcut)

    def _on_shortcut(self, event):
        # 输入框/下拉框里按键是编辑文本，不能触发运动命令。
        if isinstance(event.widget, (tk.Entry, tk.Text, ttk.Entry,
                                     ttk.Spinbox, ttk.Combobox)):
            return
        key = event.keysym
        shift = bool(event.state & 0x0001)
        if key == 'Left':
            self._nudge(-15 if shift else -5)
        elif key == 'Right':
            self._nudge(15 if shift else 5)
        elif key == 'Prior':
            self._nudge(45)
        elif key == 'Next':
            self._nudge(-45)
        elif key in ('space', 'Escape'):
            self._hold_here()
        elif key.lower() == 'z':
            self._goto_deg(0.0)

    def _report_dir(self):
        directory = os.path.join(self._rec_dir(), 'reports')
        os.makedirs(directory, exist_ok=True)
        return directory

    def _make_report(self):
        """把最近一次定位量成指标，连同波形一起导出成一页报告（PNG + PDF）。"""
        rows = list(self.wave.frozen) if self.wave.paused else list(self.wave.data)
        times = getattr(self, '_move_times', [])
        start_t = times[-2] if len(times) >= 2 else (times[-1] if times else None)
        end_t = times[-1] if len(times) >= 2 else None
        metrics = analyze_move(rows, start_t, end_t)
        if metrics is None:
            messagebox.showinfo('还没有数据',
                                '先连上板子、走一次定位，再生成报告。')
            return
        dropped = metrics.get('fault', 0)
        if dropped > 50:
            self._log('报告取消：剔除 %d 个跳变样本（板子复位/掉线？）' % dropped)
            messagebox.showerror(
                '这次数据不能用',
                '记录里有 %d 个跳变样本（转臂位置瞬间跳几千度）。\n\n'
                '通常是：\n'
                '1) 板子中途复位（电机大电流把电源拉低）；\n'
                '2) USB 线松动/掉线；\n'
                '3) 编码器或 ADC 排线接触不良。\n\n'
                '先查这几样，再跑一次（只发一条 +90，别同时点别的）。' % dropped)
            return
        try:
            png, pdf = self._render_report(metrics, rows)
        except Exception as exc:                       # noqa: BLE001
            self._log('生成报告失败：%s' % exc)
            messagebox.showerror('生成失败', str(exc))
            return
        self._log('报告已生成：%s' % png)
        self._set_msg('报告已保存到 rec/reports')
        self._preview_image(png, pdf)

    def _render_report(self, m, rows):
        import matplotlib
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.patches import FancyBboxPatch

        matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
        matplotlib.rcParams['axes.unicode_minus'] = False

        fig = Figure(figsize=(12.8, 7.6), dpi=150)
        FigureCanvasAgg(fig)
        fig.patch.set_facecolor('white')

        now = _dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        fig.text(0.04, 0.965, '倒立摆位置控制 · 定位实验报告',
                 fontsize=20, weight='bold', color='#1f2328')
        fig.text(0.04, 0.935,
                 '%s    %s    %s' % (now, self.var_port.get(),
                                     self.var_fw.get() or '固件未知'),
                 fontsize=9.5, color='#57606a')

        # ---- 指标卡 ----
        moved = m.get('moved', False)
        cards = [
            ('目标阶跃', '%+.1f°' % m['step']),
            ('超调', '%.1f°' % m['overshoot']),
            ('调节时间(±2%)', ('%.2f s' % m['settle_t'])
             if moved and m['settle_t'] < m['t_end'] - 0.2 else '未稳定'),
            ('稳态误差', '%+.2f°' % m['ss_err']),
            ('峰值速度', '%.0f °/s' % m['peak_v']),
            ('摆杆最大偏差', '%.1f°' % m['theta_max']),
        ]
        ax_m = fig.add_axes([0.04, 0.79, 0.92, 0.115])
        ax_m.set_xlim(0, len(cards)); ax_m.set_ylim(0, 1); ax_m.axis('off')
        for i, (title, value) in enumerate(cards):
            ax_m.add_patch(FancyBboxPatch((i + 0.02, 0.06), 0.96, 0.88,
                                          boxstyle='round,pad=0.01,rounding_size=0.06',
                                          fc='#f6f8fa', ec='#d0d7de', lw=0.8))
            ax_m.text(i + 0.5, 0.68, value, ha='center', va='center',
                      fontsize=17, weight='bold', color='#0969da')
            ax_m.text(i + 0.5, 0.24, title, ha='center', va='center',
                      fontsize=8.8, color='#57606a')

        # ---- 一行评价（评委最爱看） ----
        if moved:
            if m['overshoot'] <= 2.0 and m['settle_t'] <= 3.0:
                level, color = '优秀', '#1a7f37'
            elif m['overshoot'] <= 5.0 and m['settle_t'] <= 4.0:
                level, color = '良好', '#1a7f37'
            elif m['overshoot'] <= 10.0:
                level, color = '一般', '#9a6700'
            else:
                level, color = '偏大', '#cf222e'
            tips = []
            if m['overshoot'] > 5.0:
                tips.append('降低定位峰值速度或加长减速段')
            if abs(m['ss_err']) > 0.5:
                tips.append('检查位置环积分（静差偏大）')
            if m['theta_max'] > 8.0:
                tips.append('摆杆摆动偏大，可略增角度环阻尼')
            verdict = ('定位质量：%s   （超调 %.1f°，调节 %s，稳态 %+.2f°）'
                       % (level, m['overshoot'],
                          ('未稳定' if m['settle_t'] >= m['t_end'] - 0.2
                           else '%.2fs' % m['settle_t']), m['ss_err']))
            if tips:
                verdict += '    建议：' + '、'.join(tips)
        else:
            color = '#57606a'
            verdict = '稳态运行：未检测到定位动作（目标未变化）'
        fig.text(0.04, 0.752, verdict, fontsize=10.2, color=color, weight='bold')

        # ---- 两行细指标：速度误差 + 静止段抖动 ----
        vrms = m.get('v_err_rms', 0.0)
        vmax = m.get('v_err_max', 0.0)
        rest = m.get('rest_pkpk', 0.0)
        rest_th = m.get('rest_theta', 0.0)
        rest_f = m.get('rest_freq', 0.0)
        rest_pwm = m.get('rest_pwm', 0.0)
        rest_off = m.get('rest_off', 0.0)
        rest_th_off = m.get('rest_th_off', 0.0)
        line1 = ('速度误差：RMS %.0f °/s · 峰值 %.0f °/s（越小越贴曲线）'
                 % (vrms, vmax))
        fig.text(0.04, 0.723, line1, fontsize=9.2, color='#57606a')
        if m.get('rest_valid', True):
            line2 = ('静止段（最后 2 s）：位置峰峰 %.2f°（偏移 %+.2f°）· '
                     '摆杆 %.2f°（偏移 %+.2f°）· 摆动 %.2f Hz · 电机 RMS %.0f'
                     % (rest, rest_off, rest_th, rest_th_off, rest_f, rest_pwm))
        else:
            line2 = ('静止段：记录末尾转臂还在动（最后 0.5 s 位移 >1.5°），'
                     '本次静止指标不作数——停稳 3 s 再生成报告')
        rest_bad = rest > 1.5 or rest_th > 4.0
        fig.text(0.04, 0.700, line2, fontsize=8.8,
                 color='#cf222e' if rest_bad else '#1a7f37',
                 weight='bold' if rest_bad else 'normal')
        if rest_bad:
            fig.text(0.04, 0.672,
                     '静止还在来回摆：先把“阻尼”恢复到 100%，再把步长/前馈降一档；'
                     '摩擦前馈从 10~20 起小量加，加大会变成左右顶。',
                     fontsize=8.6, color='#cf222e')

        # ---- 三段曲线 ----
        seg = m['seg']
        t = [r[0] - m['t0'] for r in seg]
        loc = [r[2] for r in seg]
        tgt = [r[3] for r in seg]
        theta = [r[1] for r in seg]
        pwm = [r[4] for r in seg]

        ax1 = fig.add_axes([0.07, 0.505, 0.89, 0.150])
        ax1.plot(t, loc, '-', color='#0969da', lw=1.7, label='转臂位置')
        ax1.plot(t, tgt, '--', color='#1a7f37', lw=1.6, label='目标')
        if moved and m['band'] > 0:
            ax1.fill_between([t[0], t[-1]], m['target'] - m['band'],
                             m['target'] + m['band'], color='#1a7f37',
                             alpha=0.10, lw=0)
            ax1.axvline(m['settle_t'], color='#1a7f37', ls=':', lw=1.1)
            ax1.text(0.995, 0.95, '调节完成 %.2fs' % m['settle_t'],
                     transform=ax1.transAxes, va='top', ha='right',
                     fontsize=8.2, color='#1a7f37')
        if moved and m['overshoot'] > 0.05:
            sign = 1.0 if m['step'] > 0 else -1.0
            peak_y = m['target'] + sign * m['overshoot']
            ax1.annotate('超调 %.1f°' % m['overshoot'],
                         xy=(m['peak_t'], peak_y),
                         xytext=(m['peak_t'] + 0.35,
                                 m['target'] + sign * m['overshoot'] * 0.45),
                         fontsize=9, color='#cf222e', ha='left', va='center',
                         arrowprops=dict(arrowstyle='->', color='#cf222e', lw=0.9))
        ax1.set_ylabel('转臂 (°)', fontsize=9)
        ax1.margins(x=0.01, y=0.16)
        ax1.grid(True, color='#eaeef2', lw=0.8)
        ax1.legend(loc='lower right', fontsize=8, frameon=False)

        ax2 = fig.add_axes([0.07, 0.350, 0.89, 0.110], sharex=ax1)
        v_ref = m.get('v_ref') or [0.0] * len(t)
        v_act = m.get('v_act') or [0.0] * len(t)
        ax2.plot(t, v_ref, '--', color='#1a7f37', lw=1.5, label='理想速度（曲线）')
        ax2.plot(t, v_act, '-', color='#0969da', lw=1.5, label='实际速度')
        ax2.axhline(0, color='#d0d7de', lw=0.8)
        if vrms > 0:
            ax2.fill_between([t[0], t[-1]], -vrms, vrms, color='#bc4c00',
                             alpha=0.07, lw=0)
        ax2.text(0.995, 0.06, '速度误差 RMS %.0f °/s' % vrms,
                 transform=ax2.transAxes, ha='right', va='bottom',
                 fontsize=8.4, color='#bc4c00')
        ax2.set_ylabel('速度 (°/s)', fontsize=9)
        ax2.legend(loc='upper right', fontsize=7.6, frameon=False, ncol=2)
        ax2.grid(True, color='#eaeef2', lw=0.8)

        ax3 = fig.add_axes([0.07, 0.220, 0.89, 0.090], sharex=ax1)
        ax3.plot(t, theta, color='#cf222e', lw=1.3)
        ax3.axhline(0, color='#d0d7de', lw=0.8)
        ax3.set_ylabel('摆杆 (°)', fontsize=9)
        ax3.grid(True, color='#eaeef2', lw=0.8)

        ax4 = fig.add_axes([0.07, 0.125, 0.89, 0.070], sharex=ax1)
        ax4.plot(t, pwm, color='#8250df', lw=1.1)
        ax4.axhline(0, color='#d0d7de', lw=0.8)
        ax4.set_ylabel('电机指令', fontsize=9)
        ax4.set_xlabel('时间 (s，以本次定位起点为 0)', fontsize=9)
        ax4.grid(True, color='#eaeef2', lw=0.8)
        for ax in (ax1, ax2, ax3, ax4):
            ax.tick_params(labelsize=8, colors='#57606a')
            for side in ('top', 'right'):
                ax.spines[side].set_visible(False)

        # ---- 页脚：参数与数据来源 ----
        try:
            speed = self.var_speed.get()
        except Exception:                                  # noqa: BLE001
            speed = '?'
        try:
            kp_a = self.var_pid_a_kp.get()
            kd_a = self.var_pid_a_kd.get()
            kp_p = self.var_pid_p_kp.get()
            ks = self.var_ks.get()
        except Exception:                                  # noqa: BLE001
            kp_a = kd_a = kp_p = ks = '?'
        foot = ('步长 %s 计/50ms · 角度 KP/KD %s/%s · 位置 KP %s · 摩擦 KS %s'
                ' · 帧率 %.0f Hz · 样本 %d 点'
                % (speed, kp_a, kd_a, kp_p, ks, self.fps, len(seg)))
        if m.get('fallen'):
            foot += '  ·  已剔除 %d 个起摆/倒摆帧' % m['fallen']
        if m.get('fault'):
            foot += '  ·  跳变 %d 帧（建议重跑）' % m['fault']
        foot += '  ·  0° = 切入平衡时的转臂位置'
        if getattr(self, 'rec_path', None):
            foot += '  ·  数据 %s' % os.path.basename(self.rec_path)
        fig.text(0.04, 0.028, foot, fontsize=7.8, color='#57606a')

        name = 'report_%s' % _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        png = os.path.join(self._report_dir(), name + '.png')
        pdf = os.path.join(self._report_dir(), name + '.pdf')
        png2 = os.path.join(self._report_dir(), name + '_vel.png')

        # ---- 第二页：理想速度 / 实际速度 两幅对照波形 ----
        fig2, (bx1, bx2) = plt.subplots(2, 1, figsize=(11.2, 5.6), sharex=True)
        bx1.plot(t, v_ref, '-', color='#1a7f37', lw=2.0)
        bx1.fill_between(t, 0, v_ref, color='#1a7f37', alpha=0.12, lw=0)
        bx1.axhline(0, color='#d0d7de', lw=0.8)
        bx1.set_ylabel('理想速度 (°/s)', fontsize=10)
        bx1.set_title('① 速度曲线（上位机规划）', fontsize=11, loc='left')
        bx1.grid(True, color='#eaeef2', lw=0.8)

        bx2.plot(t, v_act, '-', color='#0969da', lw=2.0)
        bx2.fill_between(t, 0, v_act, color='#0969da', alpha=0.12, lw=0)
        bx2.axhline(0, color='#d0d7de', lw=0.8)
        bx2.set_ylabel('实际速度 (°/s)', fontsize=10)
        bx2.set_xlabel('时间 (s，以本次定位起点为 0)', fontsize=9)
        bx2.set_title('② 实测速度（编码器采集）', fontsize=11, loc='left')
        bx2.grid(True, color='#eaeef2', lw=0.8)
        if vrms > 0:
            bx2.text(0.995, 0.06, '与①的误差 RMS %.0f °/s' % vrms,
                     transform=bx2.transAxes, ha='right', va='bottom',
                     fontsize=9, color='#bc4c00')
        for ax in (bx1, bx2):
            ax.tick_params(labelsize=8.5, colors='#57606a')
            for side in ('top', 'right'):
                ax.spines[side].set_visible(False)
        fig2.tight_layout()

        fig.savefig(png, facecolor='white')
        fig2.savefig(png2, facecolor='white')
        try:
            from matplotlib.backends.backend_pdf import PdfPages
            with PdfPages(pdf) as pp:
                pp.savefig(fig)
                pp.savefig(fig2)
        except Exception:                                  # noqa: BLE001
            fig.savefig(pdf, facecolor='white')
        plt.close(fig2)
        self._last_vel_png = png2
        return png, pdf

    def _preview_image(self, png, pdf):
        """弹窗预览报告；没有 PIL 就只提示路径。"""
        try:
            from PIL import Image, ImageTk
        except ImportError:
            messagebox.showinfo('报告已生成', png)
            return
        try:
            win = tk.Toplevel(self.root)
            win.title('本次实验报告')
            win.transient(self.root)
            img = Image.open(png)
            img.thumbnail((1180, 720))
            photo = ImageTk.PhotoImage(img)
            label = ttk.Label(win, image=photo)
            label.image = photo
            label.pack(padx=8, pady=8)
            bar = ttk.Frame(win)
            bar.pack(fill='x', pady=(0, 8))
            ttk.Button(bar, text='打开图片',
                       command=lambda: os.startfile(png)).pack(side='left', padx=8)
            ttk.Button(bar, text='打开 PDF',
                       command=lambda: os.startfile(pdf)).pack(side='left')
            ttk.Button(bar, text='关闭', command=win.destroy).pack(side='right', padx=8)
        except Exception as exc:                           # noqa: BLE001
            messagebox.showinfo('报告已生成', '%s\n%s' % (png, exc))

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=(10, 8, 10, 4))
        top.pack(fill='x')
        ttk.Label(top, text='串口').pack(side='left')
        self.cmb_port = ttk.Combobox(top, textvariable=self.var_port, width=22,
                                     state='readonly')
        self.cmb_port.pack(side='left', padx=(6, 4))
        ttk.Button(top, text='刷新', width=6, command=self._refresh_ports).pack(side='left')
        self.btn_conn = ttk.Button(top, text='连接', width=8, command=self._toggle_conn)
        self.btn_conn.pack(side='left', padx=6)
        ttk.Label(top, textvariable=self.var_status).pack(side='left', padx=(10, 16))
        ttk.Label(top, textvariable=self.var_fw, foreground=COL_MUTED).pack(side='left', padx=8)
        ttk.Label(top, textvariable=self.var_state,
                  font=('Microsoft YaHei UI', 10, 'bold')).pack(side='left', padx=8)
        ttk.Label(top, textvariable=self.var_mode).pack(side='left', padx=8)

        rec_box = ttk.Frame(top)
        rec_box.pack(side='right')
        self.btn_rec = ttk.Button(rec_box, text='开始记录', command=self._toggle_record)
        self.btn_rec.pack(side='left')
        ttk.Button(rec_box, text='速度对照', command=self._open_vel_window).pack(
            side='left', padx=(6, 0))
        ttk.Button(rec_box, text='软件键盘', command=self._open_keypad).pack(
            side='left', padx=(6, 0))
        ttk.Button(rec_box, text='生成报告', command=self._make_report).pack(
            side='left', padx=(6, 0))
        ttk.Button(rec_box, text='目录', command=self._open_rec_dir).pack(
            side='left', padx=4)

        body = ttk.Frame(self.root, padding=(10, 4, 10, 8))
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, minsize=430, weight=0)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        self._build_left(body)
        self._build_right(body)

        bar = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        bar.pack(fill='x')
        ttk.Label(bar,
                  text='0° = 切入平衡时的转臂位置',
                  foreground=COL_MUTED).pack(side='left')
        ttk.Checkbutton(bar, text='连接后自动记录',
                        variable=self.var_auto_rec).pack(side='left', padx=12)
        ttk.Label(bar, textvariable=self.var_msg,
                  foreground=COL_MUTED).pack(side='right')
        ttk.Label(bar, textvariable=self.var_rec,
                  foreground=COL_MUTED).pack(side='right', padx=12)

        # 全局滚轮：把滚轮事件转给鼠标所在的可滚动页面（左栏 / 速度曲线页）
        self.root.bind_all('<MouseWheel>', self._on_wheel)

    def _on_wheel(self, event):
        widget = event.widget
        for page in (getattr(self, 'left_page', None),
                     getattr(self, 'curve_page', None)):
            if page is None:
                continue
            inner = getattr(page, 'inner', None)
            w = widget
            while w is not None:
                if w is inner:
                    page.canvas.yview_scroll(int(-event.delta / 120) or 0, 'units')
                    return
                w = getattr(w, 'master', None)

    def _build_left(self, parent):
        scroll = ScrollableFrame(parent)
        scroll.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        self.left_page = scroll
        left = scroll.inner

        pos = ttk.LabelFrame(left, text='位置控制', padding=10)
        pos.pack(fill='x')
        pos.columnconfigure(0, weight=1)
        pos.columnconfigure(1, weight=1)

        ttk.Label(pos, text='当前角度', style='Card.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(pos, text='目标角度', style='Card.TLabel').grid(row=0, column=1, sticky='w')
        ttk.Label(pos, textvariable=self.var_cur_deg, style='Big.TLabel').grid(
            row=1, column=0, sticky='w')
        ttk.Label(pos, textvariable=self.var_tgt_deg, style='Big.TLabel',
                  foreground=COL_TGT).grid(row=1, column=1, sticky='w')
        ttk.Label(pos, textvariable=self.var_cur_cnt, style='Muted.TLabel').grid(
            row=2, column=0, sticky='w')
        ttk.Label(pos, textvariable=self.var_tgt_cnt, style='Muted.TLabel').grid(
            row=2, column=1, sticky='w')
        ttk.Label(pos, textvariable=self.var_err_deg, style='Muted.TLabel').grid(
            row=3, column=0, columnspan=2, sticky='w', pady=(2, 0))

        entry_row = ttk.Frame(pos, style='Card.TFrame')
        entry_row.grid(row=4, column=0, columnspan=2, sticky='ew', pady=(10, 4))
        ttk.Label(entry_row, text='转到', style='Card.TLabel').pack(side='left')
        ent = ttk.Entry(entry_row, textvariable=self.var_entry, width=10,
                        font=('Consolas', 11))
        ent.pack(side='left', padx=6)
        ent.bind('<Return>', lambda _e: self._goto_entry())
        ttk.Label(entry_row, text='°', style='Card.TLabel').pack(side='left')
        ttk.Button(entry_row, text='执行', width=7, command=self._goto_entry).pack(
            side='left', padx=6)
        ttk.Button(entry_row, text='停住', width=7, command=self._hold_here).pack(side='left')

        grid = ttk.Frame(pos, style='Card.TFrame')
        grid.grid(row=5, column=0, columnspan=2, sticky='ew', pady=(6, 2))
        # 调试只用 90/180/360，按钮就只留这几个（相对板上当前目标累加，不会越点越偏）。
        for i, (text, delta) in enumerate((('-90°', -90), ('+90°', 90),
                                           ('+180°', 180), ('+360°', 360),
                                           ('-360°', -360))):
            ttk.Button(grid, text=text, width=6,
                       command=lambda d=delta: self._nudge(d)).grid(
                row=0, column=i, padx=1, pady=2, sticky='ew')
            grid.columnconfigure(i, weight=1)

        quick = ttk.Frame(pos, style='Card.TFrame')
        quick.grid(row=6, column=0, columnspan=2, sticky='ew', pady=(4, 0))
        ttk.Button(quick, text='回 0°', command=lambda: self._goto_deg(0.0)).pack(
            side='left', expand=True, fill='x', padx=(0, 3))
        ttk.Button(quick, text='急停（目标=当前位置）',
                   command=self._hold_here).pack(side='left', expand=True, fill='x', padx=(3, 0))

        spd = ttk.LabelFrame(left, text='逼近步长（每次 op4 一起下发）', padding=10)
        spd.pack(fill='x', pady=(8, 0))
        row = ttk.Frame(spd, style='Card.TFrame')
        row.pack(fill='x')
        for text, value in (('慢', 3), ('中', 8), ('快', 16),
                            ('快 24', 24), ('极快', 32)):
            ttk.Radiobutton(row, text='%s (%d)' % (text, value), value=str(value),
                            variable=self.var_speed, command=self._pick_speed).pack(
                side='left', padx=(0, 6))
        row2 = ttk.Frame(spd, style='Card.TFrame')
        row2.pack(fill='x', pady=(6, 0))
        ttk.Label(row2, text='自定义步长(1~64 计数/50ms)', style='Card.TLabel').pack(side='left')
        spin = ttk.Spinbox(row2, from_=P.STEP_MIN, to=P.STEP_MAX, width=5,
                           textvariable=self.var_speed, command=self._update_speed_desc)
        spin.pack(side='left', padx=6)
        spin.bind('<KeyRelease>', lambda _e: self._update_speed_desc())
        spin.bind('<Return>', lambda _e: self._apply_speed())
        ttk.Button(row2, text='应用到板', command=self._apply_speed).pack(side='left')
        ttk.Label(spd, textvariable=self.var_speed_desc, style='Muted.TLabel').pack(
            anchor='w', pady=(6, 0))
        ttk.Separator(spd).pack(fill='x', pady=8)
        ttk.Checkbutton(spd, text='自动选档（按角度用下面的预设）',
                        variable=self.var_auto_step).pack(anchor='w')
        ttk.Checkbutton(spd, text='按轨迹曲线分段执行（曲线高低=该段步长）',
                        variable=self.var_curve_exec).pack(anchor='w')
        ttk.Checkbutton(spd, text='按角度自动选曲线（90°/360°/任意）',
                        variable=self.var_curve_auto).pack(anchor='w')
        ttk.Checkbutton(spd, text='全程按曲线走（目标不变，只调节速度上限）',
                        variable=self.var_curve_stream).pack(anchor='w')
        pre = ttk.Frame(spd, style='Card.TFrame')
        pre.pack(fill='x', pady=(6, 0))
        ttk.Label(pre, text='90° 步长', style='Card.TLabel').pack(side='left')
        ttk.Spinbox(pre, from_=3, to=64, width=4,
                    textvariable=self.var_step90).pack(side='left', padx=(4, 12))
        ttk.Label(pre, text='360° 步长', style='Card.TLabel').pack(side='left')
        ttk.Spinbox(pre, from_=3, to=64, width=4,
                    textvariable=self.var_step360).pack(side='left', padx=4)
        ttk.Label(spd, text='刹车增益随距离自动（90°≈1.25、360°≈1.5），收尾 3~5 档',
                  style='Muted.TLabel').pack(anchor='w', pady=(6, 0))

        info = ttk.LabelFrame(left, text='实时状态', padding=10)
        info.pack(fill='x', pady=(8, 0))
        info.columnconfigure(1, weight=1)
        ttk.Label(info, text='摆杆角度', style='Card.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(info, textvariable=self.var_theta, style='Card.TLabel').grid(row=0, column=1, sticky='e')
        ttk.Label(info, text='电机指令', style='Card.TLabel').grid(row=1, column=0, sticky='w')
        ttk.Label(info, textvariable=self.var_pwm, style='Card.TLabel').grid(row=1, column=1, sticky='e')
        ttk.Label(info, text='位置环输出', style='Card.TLabel').grid(row=2, column=0, sticky='w')
        ttk.Label(info, textvariable=self.var_pos_out, style='Card.TLabel').grid(
            row=2, column=1, sticky='e')
        ttk.Label(info, text='允许下发', style='Card.TLabel').grid(row=3, column=0, sticky='w')
        self.lbl_gate = ttk.Label(info, text='—', style='Warn.TLabel')
        self.lbl_gate.grid(row=3, column=1, sticky='e')
        ttk.Label(info, textvariable=self.var_hint, style='Muted.TLabel',
                  wraplength=320, justify='left').grid(
            row=4, column=0, columnspan=2, sticky='w', pady=(6, 0))
        ttk.Frame(left, height=8).pack()

    def _build_param_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        page = ScrollableFrame(parent)
        page.grid(row=0, column=0, sticky='nsew')
        self.param_page = page
        inner = page.inner
        inner.columnconfigure(0, weight=1)

        cur = ttk.LabelFrame(inner, text='当前板上的值（遥测刷新）', padding=10)
        cur.pack(fill='x', padx=10, pady=(10, 4))
        ttk.Label(cur, textvariable=self.var_param_now, style='Card.TLabel',
                  wraplength=700, justify='left').pack(anchor='w')
        ttk.Button(cur, text='把当前值读到这里', command=self._load_current_params).pack(
            anchor='w', pady=(6, 0))

        pre = ttk.LabelFrame(inner, text='参数预设（保存/载入整套参数）', padding=10)
        pre.pack(fill='x', padx=10, pady=4)
        prow = ttk.Frame(pre)
        prow.pack(fill='x')
        ttk.Label(prow, text='预设:', style='Card.TLabel').pack(side='left')
        self.cmb_preset = ttk.Combobox(prow, textvariable=self.var_preset_name, width=16)
        self.cmb_preset.pack(side='left', padx=6)
        self.cmb_preset.bind('<<ComboboxSelected>>',
                             lambda _e: self._preset_fill())
        ttk.Button(prow, text='保存为预设', width=11,
                   command=self._preset_save).pack(side='left', padx=(6, 4))
        ttk.Button(prow, text='载入到界面', width=11,
                   command=self._preset_fill).pack(side='left', padx=4)
        ttk.Button(prow, text='载入并一键写入', width=15,
                   command=self._preset_apply).pack(side='left', padx=4)
        ttk.Button(prow, text='删除', width=6,
                   command=self._preset_delete).pack(side='left', padx=(4, 0))
        ttk.Label(pre, textvariable=self.var_preset_info,
                  style='Muted.TLabel', wraplength=760, justify='left').pack(
            anchor='w', pady=(6, 0))

        edit = ttk.LabelFrame(inner, text='PID 参数（按参数组写入）', padding=10)
        edit.pack(fill='x', padx=10, pady=4)
        row = ttk.Frame(edit)
        row.pack(fill='x')
        ttk.Label(row, text='参数组:', style='Card.TLabel').pack(side='left')
        ttk.Radiobutton(row, text='自动', value=0, variable=self.var_pid_bank).pack(
            side='left', padx=4)
        ttk.Radiobutton(row, text='手动', value=1, variable=self.var_pid_bank).pack(
            side='left', padx=4)
        ttk.Label(row, text='（KPA/KIA/KDA + KPP/KIP/KDP 两套各 6 个）',
                  style='Muted.TLabel').pack(side='left', padx=8)

        for title, prefix in (('角度环', 'a'), ('位置环', 'p')):
            r = ttk.Frame(edit)
            r.pack(fill='x', pady=(6, 0))
            ttk.Label(r, text=title, width=7, style='Card.TLabel').pack(side='left')
            for label, key in (('KP', 'kp'), ('KI', 'ki'), ('KD', 'kd')):
                ttk.Label(r, text=label, style='Card.TLabel').pack(side='left', padx=(10, 2))
                ent = ttk.Entry(r, textvariable=getattr(self, 'var_pid_%s_%s' % (prefix, key)),
                                width=9)
                ent.pack(side='left')
                ent.bind('<Return>', lambda _e, p=prefix: self._write_pid(p))

        btnrow = ttk.Frame(edit)
        btnrow.pack(fill='x', pady=(10, 0))
        ttk.Button(btnrow, text='写入角度环', width=12,
                   command=lambda: self._write_pid('a')).pack(side='left')
        ttk.Button(btnrow, text='写入位置环', width=12,
                   command=lambda: self._write_pid('p')).pack(side='left', padx=6)
        ttk.Button(btnrow, text='整套写入', width=10,
                   command=lambda: self._write_pid('all')).pack(side='left')
        ttk.Label(edit, textvariable=self.var_pid_status,
                  style='Muted.TLabel').pack(anchor='w', pady=(6, 0))
        ttk.Label(edit, textvariable=self.var_pid_diff,
                  style='Warn.TLabel').pack(anchor='w')
        ttk.Label(edit, text='提示：只改一个环就点对应的按钮，另一个环用板子当前值，不会被覆盖。',
                  style='Muted.TLabel', wraplength=760).pack(anchor='w', pady=(4, 0))

        misc = ttk.LabelFrame(inner, text='中值 / 摩擦前馈 / 默认值', padding=10)
        misc.pack(fill='x', padx=10, pady=4)
        r0 = ttk.Frame(misc)
        r0.pack(fill='x')
        ttk.Label(r0, text='平衡点中值(0~4095):', style='Card.TLabel').pack(side='left')
        spin_c = ttk.Spinbox(r0, from_=0, to=4095, width=7,
                             textvariable=self.var_center, command=self._write_center)
        spin_c.pack(side='left', padx=6)
        spin_c.bind('<Return>', lambda _e: self._write_center())
        ttk.Button(r0, text='写入', command=self._write_center).pack(side='left')
        ttk.Label(r0, textvariable=self.var_center_status,
                  style='Muted.TLabel').pack(side='left', padx=8)
        ttk.Label(r0, text='（写当前参数组；改动 ±5 慢慢试）',
                  style='Muted.TLabel').pack(side='left')
        r0b = ttk.Frame(misc)
        r0b.pack(fill='x', pady=(4, 0))
        r1 = ttk.Frame(misc)
        r1.pack(fill='x')
        ttk.Label(r1, text='摩擦前馈 KS(0~12):', style='Card.TLabel').pack(side='left')
        spin_ks = ttk.Spinbox(r1, from_=0, to=12, width=5, textvariable=self.var_ks,
                              command=self._write_ks)
        spin_ks.pack(side='left', padx=6)
        spin_ks.bind('<Return>', lambda _e: self._write_ks())
        ttk.Button(r1, text='写入', command=self._write_ks).pack(side='left')
        ttk.Button(r1, text='读板上值', width=9,
                   command=self._read_ks).pack(side='left', padx=(2, 6))
        ttk.Label(r1, textvariable=self.var_ks_status,
                  style='Muted.TLabel').pack(side='left', padx=8)
        ttk.Label(r1, text='（0=关；越大补得越狠）', style='Muted.TLabel').pack(
            side='left', padx=8)
        r_ff = ttk.Frame(misc)
        r_ff.pack(fill='x', pady=(4, 0))
        ttk.Label(r_ff, text='加速度前馈(Q8):', style='Card.TLabel').pack(side='left')
        ent_ff = ttk.Entry(r_ff, textvariable=self.var_ff_gain, width=7)
        ent_ff.pack(side='left', padx=6)
        ent_ff.bind('<Return>', lambda _e: self._apply_ff())
        ttk.Button(r_ff, text='写入', command=self._apply_ff).pack(side='left')
        ttk.Label(r_ff, textvariable=self.var_ff_gain_status,
                  style='Muted.TLabel').pack(side='left', padx=8)
        ttk.Label(r_ff, text='（256=1.0；0=关；加减速由前馈直接给倾斜）',
                  style='Muted.TLabel').pack(side='left')
        r_vp = ttk.Frame(misc)
        r_vp.pack(fill='x', pady=(8, 0))
        ttk.Label(r_vp, text='速度环 P %:', style='Card.TLabel').pack(side='left')
        spin_vp = ttk.Spinbox(r_vp, from_=0, to=400, increment=5, width=5,
                              textvariable=self.var_vp, command=self._apply_vp)
        spin_vp.pack(side='left', padx=6)
        spin_vp.bind('<Return>', lambda _e: self._apply_vp())
        ttk.Button(r_vp, text='写入', command=self._apply_vp).pack(side='left')
        ttk.Label(r_vp, textvariable=self.var_vp_status,
                  style='Muted.TLabel').pack(side='left', padx=8)
        ttk.Label(r_vp, text='（参考速度−实测速度，补到倾斜；0=关）',
                  style='Muted.TLabel').pack(side='left', padx=4)
        one = ttk.Frame(misc)
        one.pack(fill="x", pady=(10, 0))
        ttk.Button(one, text='一键写入（PID + 摩擦 KS + 速度环 P）',
                   command=self._write_all).pack(side="left")
        ttk.Label(one, textvariable=self.var_pid_status,
                  style="Muted.TLabel").pack(side="left", padx=10)
        r2 = ttk.Frame(misc)
        r2.pack(fill="x", pady=(6, 0))
        r2 = ttk.Frame(misc)
        r2.pack(fill='x', pady=(6, 0))
        ttk.Button(r2, text='恢复固件默认增益（op3）',
                   command=self._write_defaults).pack(side='left')
        ttk.Label(r2, text='把当前参数组的 PID 和摩擦 KS 恢复到源码默认值',
                  style='Muted.TLabel').pack(side='left', padx=8)

    def _build_joystick_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        holder = ttk.Frame(parent, style='Card.TFrame')
        holder.grid(row=0, column=0)
        self.joystick = Joystick(holder, self._on_joystick, radius=110)
        self.joystick.pack(pady=(20, 16))

        ctrl = ttk.Frame(parent, style='Card.TFrame')
        ctrl.grid(row=1, column=0, pady=(0, 6))
        ttk.Label(ctrl, text='最大速度', style='Card.TLabel').pack(side='left')
        spin = ttk.Spinbox(ctrl, from_=1, to=64, width=6,
                           textvariable=self.var_jog_max, command=self._update_jog_desc)
        spin.pack(side='left', padx=6)
        spin.bind('<KeyRelease>', lambda _e: self._update_jog_desc())
        ttk.Label(ctrl, text='计/51ms', style='Muted.TLabel').pack(side='left')
        ttk.Checkbutton(ctrl, text='反向', variable=self.var_jog_invert,
                        command=self._update_jog_desc).pack(side='left', padx=(16, 6))
        ttk.Button(ctrl, text='停', width=5, command=self._jog_stop).pack(side='left')

        ttk.Label(parent, textvariable=self.var_jog_desc, style='Muted.TLabel',
                  wraplength=760, justify='center').grid(row=2, column=0, pady=(0, 14))

    def _build_curve_tab2(self, parent):
        """轨迹曲线：曲线只决定“每 50 ms 走多少”，由上位机按曲线流式发 op4。"""
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        self.curve_editor = CurveEditor(parent, on_change=self._on_curve_change,
                                        height=240)
        self.curve_editor.grid(row=0, column=0, sticky='nsew', padx=10, pady=(10, 4))

        presets = ttk.Frame(parent, style='Card.TFrame')
        presets.grid(row=1, column=0, sticky='ew', padx=10)
        for name in ('匀速', '梯形', 'S 形', '慢起慢停', '软停'):
            ttk.Button(presets, text=name, width=9,
                       command=lambda n=name: self._apply_curve_preset(n)).pack(
                side='left', padx=(0, 4))
        ttk.Button(presets, text='平滑一次', width=8,
                   command=lambda: self._apply_curve_preset('平滑')).pack(
            side='left', padx=(8, 0))
        ttk.Button(presets, text='撤销', width=6,
                   command=lambda: self._undo_curve()).pack(side='left', padx=(8, 0))

        saved = ttk.Frame(parent, style='Card.TFrame')
        saved.grid(row=4, column=0, sticky='ew', padx=10, pady=(6, 0))
        ttk.Label(saved, text='曲线名称', style='Card.TLabel').pack(side='left')
        self.var_curve_name = getattr(self, 'var_curve_name',
                                      tk.StringVar(value='我的软停'))
        ttk.Entry(saved, textvariable=self.var_curve_name, width=12).pack(
            side='left', padx=6)
        ttk.Button(saved, text='保存曲线', command=self._save_curve).pack(side='left')
        self.var_saved_curve = getattr(self, 'var_saved_curve', tk.StringVar())
        self.cmb_saved = ttk.Combobox(saved, textvariable=self.var_saved_curve,
                                      width=14, state='readonly')
        self.cmb_saved.pack(side='left', padx=(10, 4))
        ttk.Button(saved, text='载入', command=self._load_curve).pack(side='left')
        ttk.Button(saved, text='删除', width=6,
                   command=self._delete_curve).pack(side='left', padx=(4, 0))
        self.var_curve_saved_info = getattr(self, 'var_curve_saved_info',
                                            tk.StringVar(value=''))
        ttk.Label(saved, textvariable=self.var_curve_saved_info,
                  style='Muted.TLabel').pack(side='left', padx=8)

        ctrl = ttk.Frame(parent, style='Card.TFrame')
        ctrl.grid(row=2, column=0, sticky='ew', padx=10, pady=(6, 0))
        ttk.Label(ctrl, text='目标', style='Card.TLabel').pack(side='left')
        ent = ttk.Entry(ctrl, textvariable=self.var_curve_target, width=8)
        ent.pack(side='left', padx=6)
        ent.bind('<Return>', lambda _e: self._run_curve())
        ent.bind('<KeyRelease>', lambda _e: self._update_curve_info2())
        ttk.Button(ctrl, text='执行曲线', command=self._run_curve).pack(
            side='left', padx=(4, 6))
        ttk.Button(ctrl, text='停止', width=6, command=self._stop_curve).pack(side='left')
        ttk.Label(parent, textvariable=self.var_curve_info2, style='Muted.TLabel',
                  wraplength=840, justify='left').grid(
            row=3, column=0, sticky='w', padx=12, pady=(6, 8))
        self._update_curve_info2()

    # ---------------- 曲线保存/载入 ----------------
    def _curves_path(self):
        import os
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'config', 'curves.json')

    def _read_curves(self):
        import json
        import os
        path = self._curves_path()
        if not os.path.exists(path):
            return {}
        try:
            with open(path, encoding='utf-8') as fh:
                data = json.load(fh)
            return data if isinstance(data, dict) else {}
        except Exception:                                  # noqa: BLE001
            return {}

    def _write_curves(self, data):
        import json
        import os
        path = self._curves_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)

    def _refresh_saved_curves(self):
        names = sorted(self._read_curves().keys())
        try:
            self.cmb_saved['values'] = names
        except Exception:                                  # noqa: BLE001
            pass
        self.var_curve_saved_info.set('已存 %d 条' % len(names) if names else '还没有保存的曲线')

    def _save_curve(self):
        name = (self.var_curve_name.get() or '').strip()
        if not name:
            self._set_msg('先给曲线起个名字')
            return
        vals = [max(0, min(256, int(v))) for v in self.curve_editor.get_vals()]
        data = self._read_curves()
        data[name] = vals
        try:
            self._write_curves(data)
        except Exception as exc:                           # noqa: BLE001
            self._set_msg('保存曲线失败：%s' % exc)
            return
        self._refresh_saved_curves()
        self.var_curve_saved_info.set('已保存「%s」' % name)
        self._log('保存曲线「%s」：%s' % (name, vals))
        self._set_msg('曲线「%s」已保存' % name)

    def _load_curve(self):
        name = (self.var_saved_curve.get() or '').strip()
        if not name:
            self._set_msg('先在下拉框选一条曲线')
            return
        data = self._read_curves()
        vals = data.get(name)
        if not vals:
            self._set_msg('没找到曲线「%s」' % name)
            return
        self.curve_editor.set_vals(vals)
        self.var_curve_name.set(name)
        self.var_curve_saved_info.set('已载入「%s」' % name)
        self._log('载入曲线「%s」' % name)

    def _delete_curve(self):
        name = (self.var_saved_curve.get() or '').strip()
        if not name:
            return
        data = self._read_curves()
        if name in data:
            del data[name]
            self._write_curves(data)
            self._refresh_saved_curves()
            self.var_curve_saved_info.set('已删除「%s」' % name)

    def _on_curve_change(self, _vals=None):
        try:
            self._curve_undo.append(list(self.curve_editor.get_vals()))
            if len(self._curve_undo) > 30:
                self._curve_undo.pop(0)
        except Exception:                                  # noqa: BLE001
            pass
        self._update_curve_info2()

    def _apply_curve_preset(self, name):
        presets = P.curve_presets()
        if name == '平滑':
            vals = list(self.curve_editor.get_vals())
            for _ in range(2):
                vals = [(vals[max(0, i - 1)] + vals[i] + vals[min(15, i + 1)]) // 3
                        for i in range(16)]
            self.curve_editor.set_vals(vals)
            return
        vals = presets.get(name)
        if vals is None:
            self._set_msg('没有这个预设：%s' % name)
            return
        self.curve_editor.set_vals(list(vals))

    def _undo_curve(self):
        if not self._curve_undo:
            self._set_msg('没有可撤销的曲线改动')
            return
        self.curve_editor.set_vals(self._curve_undo.pop())

    def _update_curve_info2(self):
        try:
            vals = self.curve_editor.get_vals()
        except Exception:                                  # noqa: BLE001
            return
        try:
            deg = float(self.var_curve_target.get().replace('°', ''))
        except ValueError:
            deg = 0.0
        try:
            step = P.clamp_step(int(float(self.var_speed.get())))
        except ValueError:
            step = 8
        mean = max(0.05, sum(vals) / 256.0 / P.PROFILE_POINTS)
        seconds = abs(P.deg_to_counts(deg)) / (step * mean) * 0.05 if step else 0
        self.var_curve_info2.set(
            '16 点：%s\n峰值倍率 %.0f%%，平均 %.0f%%　→　转 %.0f° 约 %.1f s（步长 %d）'
            % (','.join(str(v) for v in vals), max(vals) / 2.56, mean * 100,
               deg, seconds, step))

    def _run_curve(self):
        """轨迹曲线页：按曲线页的目标启动一次定位（和左侧按钮走同一条路径）。"""
        text = self.var_curve_target.get().strip().replace('°', '')
        if not text:
            self._set_msg('先在“轨迹曲线”页的目标框里填一个角度')
            self.var_curve_info2.set('先填目标角度（例如 90），再点“执行曲线”')
            return
        try:
            deg = float(text)
        except ValueError:
            messagebox.showerror('输入错误', '目标角度不是数字：%r' % text)
            return
        self.var_entry.set('%.1f' % deg)
        self._goto_deg(deg)

    def _curve_step(self):
        plan = self._curve_plan
        self._curve_job = None
        if not plan:
            return
        loc = self.last_location
        if loc is None:
            self._curve_plan = None
            return
        now = time.monotonic()
        if plan.get('stream'):
            self._curve_stream_step(plan, loc, now)
            return
        stages = plan['stages']
        phase = plan['phase']
        if phase >= len(stages):
            if abs(loc - plan['target']) <= 4 or now - plan['t0'] > 12.0:
                err = int(loc - plan['target'])
                self._log('定位完成：误差 %+d 计数（%.1f°）' % (err, err * 0.3383))
                self.var_curve_info2.set('定位完成，误差 %+d 计数' % err)
                self._curve_plan = None
                return
            self._curve_job = self.root.after(100, self._curve_step)
            return
        if phase > 0:
            prev = stages[phase - 1][0]
            before = stages[phase - 2][0] if phase >= 2 else plan['start']
            span = abs(prev - before)
            lead = max(12, int(span * 0.35))     # 提前 35% 段长就切，速度不掉
            stalled = now - plan['t0'] > 2.5
            if abs(loc - prev) > lead and not stalled:
                self._curve_job = self.root.after(80, self._curve_step)
                return
            if stalled:
                self._log('第 %d 段等待超时，继续下一段' % phase)
        target, step, label = stages[phase]
        try:
            _, packet = self.link.build(4, target, step)
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._curve_plan = None
            self._set_msg('定位中止：%s' % exc)
            return
        self._rec_note('CURVE %s %d step %d' % (label, target, step))
        plan['phase'] = phase + 1
        plan['t0'] = now
        self.var_curve_info2.set('%s → %s°（步长 %d）'
                                 % (label, _fmt(P.counts_to_deg(target)), step))
        self._curve_job = self.root.after(100, self._curve_step)

    def _curve_stream_step(self, plan, loc, now):
        """全程曲线：目标不动，按行程进度把步长按曲线改小/改大。"""
        vals = plan['vals']
        base = plan['base']
        span = plan['dist']
        prog = 0.0 if span == 0 else abs(loc - plan['start']) / float(abs(span))
        prog = max(0.0, min(1.0, prog))
        idx = prog * (len(vals) - 1)
        i0 = int(idx)
        i1 = min(len(vals) - 1, i0 + 1)
        val = vals[i0] + (vals[i1] - vals[i0]) * (idx - i0)
        step = max(3, min(base, int(round(base * val / 256.0))))
        try:
            _, packet = self.link.build(4, plan['target'], step)
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._curve_plan = None
            self._set_msg('曲线中止：%s' % exc)
            return
        plan['sends'] = plan.get('sends', 0) + 1
        if plan['sends'] % 5 == 1:
            self._rec_note('CURVE %d%% step %d' % (int(prog * 100), step))
            self.var_curve_info2.set('进度 %d%%  曲线 %d/256 → 步长 %d  剩余 %d 计数'
                                     % (int(prog * 100), int(val), step,
                                        int(abs(plan['target'] - loc))))
        remain = abs(plan['target'] - loc)
        if remain <= 4 or now - plan['t0'] > 30.0:
            err = int(loc - plan['target'])
            self._log('定位%s：误差 %+d 计数（%.1f°）'
                      % ('完成' if remain <= 4 else '超时', err, err * 0.3383))
            self.var_curve_info2.set('定位完成，误差 %+d 计数' % err)
            self._curve_plan = None
            return
        self._curve_job = self.root.after(150, self._curve_step)

    def _stop_curve(self):
        """取消曲线并用 op5 冻结在当前位置。"""
        if self._curve_job is not None:
            try:
                self.root.after_cancel(self._curve_job)
            except Exception:                          # noqa: BLE001
                pass
            self._curve_job = None
        if self._curve_plan is None:
            self._set_msg('没有正在执行的曲线')
            return
        self._curve_plan = None
        self._hold_here()
        self.var_curve_info2.set('曲线已停止（已冻结在当前位置）')

    def _build_track_tab(self, parent):
        """定位跟踪：上栏 位置/目标，下栏 位置误差（自动放大）。"""
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)
        bar = ttk.Frame(parent, style='Card.TFrame')
        bar.grid(row=0, column=0, sticky='ew', padx=10, pady=(8, 4))
        ttk.Label(bar, text='滚轮上下滚动 · 下栏误差自动放大',
                  style='Muted.TLabel').pack(side='left')
        self.track_canvas = tk.Canvas(parent, bg='#ffffff',
                                      highlightthickness=1,
                                      highlightbackground=COL_GRID)
        self.track_canvas.grid(row=1, column=0, sticky='nsew', padx=10, pady=(0, 10))
        self.track = TrackView(self.track_canvas, self)

    def _build_right(self, parent):
        right = ttk.Frame(parent)
        right.grid(row=0, column=1, sticky='nsew')
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self.notebook = ttk.Notebook(right)
        self.notebook.grid(row=0, column=0, sticky='nsew')
        wave_tab = ttk.Frame(self.notebook, style='Card.TFrame')
        curve_tab = ttk.Frame(self.notebook, style='Card.TFrame')
        track_tab = ttk.Frame(self.notebook, style='Card.TFrame')
        param_tab = ttk.Frame(self.notebook, style='Card.TFrame')
        self.notebook.add(wave_tab, text='实时波形')
        self.notebook.add(curve_tab, text='轨迹曲线')
        self.notebook.add(track_tab, text='定位跟踪')
        self.notebook.add(param_tab, text='参数')
        joy_tab = ttk.Frame(self.notebook, style='Card.TFrame')
        self.notebook.add(joy_tab, text='摇杆')
        self._build_joystick_tab(joy_tab)

        wave_tab.rowconfigure(1, weight=1)
        wave_tab.columnconfigure(0, weight=1)
        bar = ttk.Frame(wave_tab, style='Card.TFrame')
        bar.grid(row=0, column=0, sticky='ew', padx=8, pady=(6, 4))
        ttk.Label(bar, text='时间窗', style='Card.TLabel').pack(side='left')
        self.cmb_span = ttk.Combobox(bar, values=('5', '10', '15', '30', '60'),
                                     width=5, state='readonly')
        self.cmb_span.set('15')
        self.cmb_span.pack(side='left', padx=6)
        self.cmb_span.bind('<<ComboboxSelected>>', self._on_span)
        self.btn_pause = ttk.Button(bar, text='暂停显示', width=9, command=self._toggle_pause)
        self.btn_pause.pack(side='left', padx=6)
        ttk.Button(bar, text='清空', width=6, command=self._clear_wave).pack(side='left')
        ttk.Label(bar, text='实线=实际  虚线=板上目标', style='Muted.TLabel').pack(
            side='left', padx=12)
        ttk.Label(bar, textvariable=self.var_wave_stats,
                  style='Muted.TLabel').pack(side='right')

        self.canvas = tk.Canvas(wave_tab, bg=COL_PANEL, highlightthickness=1,
                                highlightbackground=COL_GRID)
        self.canvas.grid(row=1, column=0, sticky='nsew', padx=8, pady=(0, 8))
        self.wave = WaveView(self.canvas)

        self._build_curve_tab2(curve_tab)
        self._build_track_tab(track_tab)
        self._build_param_tab(param_tab)

        logf = ttk.LabelFrame(right, text='命令与事件', padding=6)
        logf.grid(row=1, column=0, sticky='nsew', pady=(8, 0))
        logf.rowconfigure(0, weight=1)
        logf.columnconfigure(0, weight=1)
        self.txt_log = tk.Text(logf, height=8, wrap='none', font=('Consolas', 9),
                               bg='#ffffff', fg=COL_TEXT, relief='flat',
                               highlightthickness=1, highlightbackground=COL_GRID)
        self.txt_log.grid(row=0, column=0, sticky='nsew')
        scroll = ttk.Scrollbar(logf, orient='vertical', command=self.txt_log.yview)
        scroll.grid(row=0, column=1, sticky='ns')
        self.txt_log.configure(yscrollcommand=scroll.set, state='disabled')

    # ------------------------------------------------------------ 串口
    def _refresh_ports(self, select=None):
        if list_ports is None:
            self.var_status.set('缺少 pyserial：pip install pyserial')
            return
        ports = []
        for info in list_ports.comports():
            ports.append('%s — %s' % (info.device, info.description or ''))
        self.cmb_port['values'] = ports
        if not ports:
            self.var_port.set('')
            self.var_status.set('未发现串口')
            return
        wanted = None
        if select:
            wanted = next((p for p in ports if p.upper().startswith(select.upper())), None)
        if wanted is None:
            current = self.var_port.get()
            wanted = current if current in ports else ports[0]
        self.var_port.set(wanted)

    def _toggle_conn(self):
        if self.connected:
            self._disconnect('用户断开')
        else:
            self._connect()

    def _connect(self):
        if serial is None:
            messagebox.showerror('缺少依赖', '需要 pyserial：\n\npip install pyserial')
            return
        label = self.var_port.get()
        if not label:
            messagebox.showwarning('没有串口', '请先插入 USB 转串口并点“刷新”')
            return
        port = label.split(' ')[0]
        self.queue = queue.Queue()
        self.worker = SerialWorker(port, BAUD, self.queue)
        self.worker.start()
        self.var_status.set('正在打开 %s …' % port)
        self.btn_conn.configure(text='正在打开…', state='disabled')
        self.root.after(3000, self._open_watchdog, port)

    def _open_watchdog(self, port):
        """3 s 还没连上就当打开卡住了，重置按钮并提示。"""
        if self.connected:
            return
        if self.worker is not None and self.worker.is_alive():
            # 线程还卡在 serial.Serial()：多半是驱动/端口状态坏了
            self.worker.stop()
            self.worker = None
        self.var_status.set('打开 %s 超时' % port)
        self.btn_conn.configure(text='连接', state='normal')
        self._log('打开 %s 超时（3 s 没有回应）' % port)
        messagebox.showerror(
            '打开串口超时',
            '%s 3 秒没有打开成功。\n\n'
            '通常是：\n'
            '1) 另一个程序还占着这个口（关掉旧上位机/串口助手）；\n'
            '2) USB 刚拔插过，驱动还没稳定（等 3 秒点刷新再连）；\n'
            '3) 换一根数据线/换一个 USB 口。' % port)
        self._refresh_ports()

    def _disconnect(self, reason=''):
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.connected = False
        self.link.reset()
        self.last_state = None
        self.var_status.set('未连接')
        self.btn_conn.configure(text='连接', state='normal')
        self._abort_curve('串口断开')
        self._curve_synced = False
        self.var_curve_status.set('未下载')
        self._jog_pending = 0
        self._jog_sent = 0
        self._rec_note('DISCONNECT %s' % (reason or ''))
        if self.recording:
            self._stop_record()
        self._log('断开：%s' % (reason or '—'))
        self._update_gate()

    # ------------------------------------------------------------ 事件循环
    def _poll(self):
        """轮询串口事件。单个事件出错不能把整个循环弄死（否则界面会永远卡在
        “正在打开”，而且串口还被这个进程占着）。"""
        for _ in range(400):
            try:
                kind, payload = self.queue.get_nowait()
            except queue.Empty:
                break
            try:
                self._handle(kind, payload)
            except Exception as exc:                       # noqa: BLE001
                import traceback
                traceback.print_exc()
                self._log('处理 %s 事件出错：%s' % (kind, exc))
        try:
            lost = self.link.check_timeout()
            if lost is not None:
                self._log('命令 %d 超时未确认，请读取板上目标再操作' % lost.sequence)
                self._set_msg(self.link.message)
                if lost.operation == 6:
                    self._abort_curve('确认超时')
            self._update_gate()
            self._maybe_restore_step()
            if self.connected and not self.link.fresh:
                self.var_status.set('已连接，等待遥测…')
        except Exception:                                  # noqa: BLE001
            import traceback
            traceback.print_exc()
        finally:
            self._poll_id = self.root.after(POLL_MS, self._poll)

    def _handle(self, kind, payload):
        if kind == 'open':
            self.connected = True
            self.var_status.set('%s 已连接 @115200' % payload)
            self.btn_conn.configure(text='断开', state='normal')
            self._log('串口打开：%s 115200 8N1' % payload)
            self._set_msg('等待遥测帧（约 100 Hz）')
            if self.var_auto_rec.get() and not self.recording:
                self._start_record()
            self._rec_note('CONNECT %s' % payload)
            # 曲线/前馈都是易失的：连上后自动补发一次，避免“忘了下载”导致硬停超调。
            self.root.after(900, self._auto_sync)
            # 连上后把板子的实际 PID 读进参数页，避免用过期值覆盖
            self.root.after(1300, self._load_current_params)
        elif kind == 'closed':
            if self.connected:
                self.connected = False
                self.var_status.set('串口已关闭')
                self.btn_conn.configure(text='连接')
                self._log('串口关闭')
                self._update_gate()
        elif kind == 'error':
            self._log('错误：%s' % payload)
            self._set_msg(payload)
            self._refresh_ports()
            messagebox.showerror('打不开串口', payload)
            if not self.connected:
                self.btn_conn.configure(text='连接', state='normal')
        elif kind == 'frame':
            self._on_frame(payload)

    def _on_frame(self, frame):
        event, detail = self.link.observe(frame)
        self.last_frame = frame
        self._pid_write_check()
        if getattr(self, '_ks_read_pending', False) and frame.get('pid_command_bank') == 0x80:
            self._ks_read_pending = False
            ks = int(frame.get('pid_position_kd', 0)) & 0x0F
            self.var_ks.set(str(ks))
            self.var_ks_status.set('板上 %d' % ks)
            self._log('板上摩擦 KS = %d' % ks)
        self.frame_count += 1
        self._fps_frames += 1
        now = time.monotonic()
        if now - self._fps_mark >= 1.0:
            self.fps = self._fps_frames / (now - self._fps_mark)
            self._fps_mark = now
            self._fps_frames = 0

        if event == 'reset':
            self._log('检测到 FPGA 复位：%s' % self.link.message)
            self._set_msg(self.link.message)
            self._rec_note('FPGA_RESET')
            self._abort_curve('FPGA 复位')
            self._curve_synced = False
            self.var_curve_status.set('未下载（FPGA 复位）')
            if self.var_ff.get() not in ('', '0'):
                self._log('FPGA 复位：速度前馈已回到 0，需要重新下发')
                self.var_ff_status.set('已归零，需重发')
            if self.var_ff_cmd.get() not in ('', '0'):
                self._log('FPGA 复位：指令前馈已回到 0，需要重新下发')
                self.var_ff_cmd_status.set('已归零，需重发')
            if self.var_deadzone.get() not in ('', '5'):
                self._log('FPGA 复位：角度死区已回到默认 5，需要重新下发')
                self.var_deadzone_status.set('已回默认，需重发')
            if self.var_vp.get() not in ('', '0'):
                self._log('FPGA 复位：速度环 P 已回到 0，需要重新下发')
                self.var_vp_status.set('已归零，需重发')
            self.root.after(900, self._auto_sync)
        elif event == 'ack':
            extra = ''
            if getattr(detail, 'operation', 0) in (10, 11, 12, 14, 15):
                extra = '（写入值 %d）' % getattr(detail, 'value', 0)
            self._log('确认 %d：%s%s' % (detail.sequence, self.link.message, extra))
            self._set_msg(self.link.message)
            self._rec_note('ACK %d' % detail.sequence)
            if detail.operation in (5, 6, 8, 9, 10, 11, 12, 14, 15) and self._curve_active:
                self._send_next_curve()
            elif self._move_queue:
                self._pump_move()
            elif detail.operation == 5:
                self._log('确认 %d：斜坡步长 = %s' %
                          (detail.sequence, getattr(detail, 'value', '')))
                self.applied_step = P.clamp_step(int(getattr(detail, 'value', 0) or 0))
                self._update_speed_desc()
                # 步长变了，曲线的时间基准跟着重算，避免节拍对不上。
                self._send_scale()
        elif event == 'reject':
            pending, result = detail
            self._log('拒绝 %d：%s' % (pending.sequence, P.RESULTS.get(result, result)))
            self._set_msg('命令被拒绝：%s' % P.RESULTS.get(result, result))
            self._rec_note('REJ %d/%s' % (pending.sequence, result))
            if pending.operation in (5, 6, 8, 9, 10, 11, 12, 14, 15) and self._curve_active:
                self._abort_curve('第 %d 点被拒绝' % pending.value)
            if self._move_queue:
                self._move_queue = []

        if frame.get('format') != 'trace_v3_100hz':
            self.var_fw.set('旧固件（仅观测）')
            self._set_msg('当前码流不是 v3 遥测：位置控制需要 0x26101801 及以上固件')
            self.var_state.set(P.STATES.get(frame.get('state'), '—'))
            return

        state = frame['state']
        self.var_fw.set('固件 0x%08X' % frame['firmware_id'])
        self.var_state.set(P.STATES.get(state, str(state)))
        self.var_mode.set('手动' if frame['manual_mode'] else '自动')
        center = frame.get('angle_center') or 2971
        theta_deg = (frame['theta'] - center) * P.DEG_PER_COUNT
        self.var_theta.set('%+.1f°  (%d)' % (theta_deg, frame['theta']))
        self.var_pwm.set('%+d' % frame['pwm'])
        self.var_pos_out.set('%+d' % frame.get('position_output_q8', 0))
        self.var_cur_deg.set(_fmt(P.counts_to_deg(frame['location'])))
        self.var_cur_cnt.set('%d 计数' % frame['location'])
        self.var_tgt_deg.set(_fmt(P.counts_to_deg(frame['target'])))
        self.var_tgt_cnt.set('%d 计数' % frame['target'])
        err = P.counts_to_deg(frame['location'] - frame['target'])
        self.var_err_deg.set('位置误差 %s°  ·  状态 %s' % (
            _fmt(err), P.STATES.get(state, state)))
        self.var_status.set('已连接 · %.0f 帧/秒' % self.fps)
        self.var_wave_stats.set('%d 帧  ·  %.0f Hz' % (len(self.wave.data), self.fps))
        if now - getattr(self, '_param_mark', 0.0) >= 0.5:
            self._param_mark = now
            self._update_param_now(frame)

        if self.last_state == 4 and state != 4:
            self._log('离开平衡态（%s）' % P.STATES.get(state, state))
            self._rec_note('LEAVE_BAL %s' % state)
            if self._jog_sent != 0:
                self._jog_pending = 0
                self._flush_jog()
                self.joystick.reset()
        elif self.last_state != 4 and state == 4:
            self._log('进入平衡态')
            self._rec_note('ENTER_BAL')
        self.last_state = state
        self.last_location = frame.get('location')
        if self.last_location is not None:
            self._arm_hist.append((time.monotonic(), self.last_location))

        self.wave.add(frame)
        if self.recording:
            self._write_row(frame)

    # ------------------------------------------------------------ 位置命令
    def _link_ready(self):
        if not self.connected:
            return False, '未连接串口'
        if not self.link.fresh:
            return False, '超过 0.5 s 没有遥测（检查串口/波特率）'
        if self.link.format != 'trace_v3_100hz':
            return False, '固件不支持位置命令（需要 v3 遥测）'
        if self.link.busy:
            return False, '上一条命令还在等待确认'
        return True, '可以下发'

    def _gate(self):
        ok, reason = self._link_ready()
        if not ok:
            return ok, reason
        if self.last_state != 4:
            return False, '未处于平衡态（%s）' % P.STATES.get(
                self.last_state, self.last_state)
        return True, '可以下发'

    def _update_gate(self):
        ok, reason = self._gate()
        self.lbl_gate.configure(text=reason if not ok else '可以下发',
                                style='Good.TLabel' if ok else 'Warn.TLabel')
        self.var_hint.set('平衡后可直接输入角度，或按 ←/→ 微调；空格=停住，z=回零。')

    def _base_target(self):
        """当前目标的基准：优先用板上斜坡目标，其次用实测位置。"""
        if self.link.board_target is not None:
            return self.link.board_target
        if self.wave.data:
            return int(round(self.wave.data[-1][2] / P.DEG_PER_COUNT))
        return 0

    # 离散分档限速（v1.3 行为）：短移动自动降速，长移动才用用户设的最大速度。
    # 超调大致随速度平方增长：步长 5≈2°、8≈6°、18≈30°，所以短移动必须压得很低。
    # 单位是编码器计数（1064/圈）：340≈115°，700≈237°，1200≈406°。
    # 最大速度 32 时：90°→10、180°→16、270°→27、360°→27。
    SPEED_TIERS = ((340, 0.32, 5), (700, 0.50, 6), (1200, 0.85, 8))

    def _tier_step(self, dist, user_step):
        for limit, frac, floor in self.SPEED_TIERS:
            if dist <= limit:
                return min(user_step, max(floor, int(round(user_step * frac))))
        return user_step

    def _auto_sync(self):
        """这个固件没有曲线/前馈等易失参数，连上后不需要补发。"""
        return

    def _maybe_restore_step(self):
        """步长随每次 op4 一起下发，不需要事后恢复。"""
        return

    def _arm_speed(self):
        """最近约 0.2 秒的转臂速度（计数/秒）；样本不够时返回 None。"""
        hist = getattr(self, '_arm_hist', None)
        if not hist or len(hist) < 5:
            return None
        t1, l1 = hist[-1]
        t0, l0 = hist[0]
        dt = t1 - t0
        if dt < 0.12:
            return None
        return (l1 - l0) / dt

    def _send_position(self, counts, why, note=''):
        """定位。勾选“按曲线分段执行”时按曲线分 4 段，否则一条 op4 直达。"""
        ok, reason = self._gate()
        if not ok:
            self._log('未下发：%s' % reason)
            self._set_msg(reason)
            return False
        if self._curve_plan is not None:
            self._set_msg('上一条定位还在走，等它结束或点“停止”')
            return False
        target = P.clamp_position(counts)
        start = self.last_location
        if start is None:
            self._set_msg('还没收到位置读数')
            return False
        dist = target - start
        if abs(dist) < 10:
            self._set_msg('目标与当前位置只差 %d 计数，不执行' % dist)
            return False

        try:
            base = P.clamp_step(int(float(self.var_speed.get())))
        except ValueError:
            base = 8
        if self.var_auto_step.get():
            try:
                big = abs(dist) >= P.deg_to_counts(180)
                base = int(float(self.var_step360.get() if big else self.var_step90.get()))
            except ValueError:
                pass
        base = max(3, min(base, int(max(6, abs(dist) / 20.0))))

        sign = 1 if dist > 0 else -1
        gain = 1.25 + 0.25 * min(1.0, max(0.0, (abs(dist) - 300.0) / 700.0))
        brake = int(max(abs(dist) * 0.10,
                        (0.45 * base * base + 0.08 * abs(dist)) * gain))
        brake = min(brake, int(abs(dist) * 0.85))
        step_final = max(3, min(5, int(round(brake / 70.0))))

        vals = None
        curve_name = '手动'
        if self.var_curve_auto.get():
            vals, curve_name = self._auto_curve_vals(abs(dist) * P.DEG_PER_COUNT)
        if not vals:
            try:
                vals = [max(0, int(v)) for v in self.curve_editor.get_vals()]
            except Exception:                              # noqa: BLE001
                vals = [256] * 16

        # 实测：<=120° 用 4 段法更准（90° 3.05° vs 19.3°），>120° 用全程曲线更准（180° 5.4° vs 12.5°）
        use_stream = self.var_curve_stream.get() and abs(dist) * P.DEG_PER_COUNT > 120
        if use_stream:
            # 全程曲线：目标固定，按行程进度实时改步长（靠固件"同目标续发不重置速度"）
            plan = {
                'target': target, 'start': start, 'dist': dist,
                'stream': True, 'vals': vals, 'base': base,
                't0': time.monotonic(), 't_last': time.monotonic(),
                'sends': 0, 'why': why, 'note': note,
            }
            self._curve_plan = plan
            self._log('定位 %+d 计数：全程曲线（基准 %d 档，曲线 %s）  %s'
                      % (dist, base, curve_name, why))
            self._rec_note(note or ('MOVE %d curve %s' % (target, curve_name)))
            self._last_move_t = time.monotonic()
            self._move_times = getattr(self, '_move_times', [])
            self._move_times.append(self._last_move_t)
            if self._curve_job is not None:
                try:
                    self.root.after_cancel(self._curve_job)
                except Exception:                      # noqa: BLE001
                    pass
            self._curve_step()
            return True

        stages = []
        if self.var_curve_exec.get() and abs(dist) >= 60:
            seg = max(1, len(vals) // 4)
            for k in range(4):
                part = vals[k * seg:(k + 1) * seg] or [256]
                ratio = max(0.08, (sum(part) / len(part)) / 256.0)
                st = max(3, min(base, int(round(base * ratio))))
                tgt = start + int(round(dist * (k + 1) / 4.0))
                stages.append((tgt, st, '段%d' % (k + 1)))
            stages[-1] = (target, step_final, '收尾')
        else:
            stages = [(target, base, '直达')]

        self._curve_plan = {
            'target': target, 'start': start, 'dist': dist, 'phase': 0,
            'stages': stages, 'base': base, 'step_final': step_final,
            't0': time.monotonic(), 'why': why, 'note': note,
        }
        self._log('定位 %+d 计数：%s  %s'
                  % (dist,
                     ' → '.join('%s %d@步长%d' % (l, t_, s_) for t_, s_, l in stages),
                     why))
        self._rec_note(note or ('MOVE %d steps %s'
                                % (target, '/'.join(str(s[1]) for s in stages))))
        self._last_move_t = time.monotonic()
        self._move_times = getattr(self, '_move_times', [])
        self._move_times.append(self._last_move_t)
        if len(self._move_times) > 20:
            self._move_times.pop(0)
        if self._curve_job is not None:
            try:
                self.root.after_cancel(self._curve_job)
            except Exception:                          # noqa: BLE001
                pass
        self._curve_step()
        return True

    def _goto_entry(self):
        text = self.var_entry.get().strip().replace('°', '')
        try:
            deg = float(text)
        except ValueError:
            messagebox.showerror('输入错误', '目标角度不是数字：%r' % text)
            return
        self._goto_deg(deg)

    def _goto_deg(self, deg):
        counts = P.deg_to_counts(deg)
        if abs(counts) > P.POSITION_LIMIT:
            self._set_msg('目标超出 ±%d°（±10 圈），已限幅' %
                          int(P.counts_to_deg(P.POSITION_LIMIT)))
        self._send_position(counts, '转到 %s°' % _fmt(deg))

    def _nudge(self, delta_deg):
        base = self._base_target()
        target = base + P.deg_to_counts(delta_deg)
        self.var_entry.set('%.1f' % P.counts_to_deg(P.clamp_position(target)))
        self._send_position(target, '相对 %+d°' % delta_deg)

    def _hold_here(self):
        """op5：冻结在当前位置（这是固件自带的“停”）。"""
        ok, reason = self._gate()
        if not ok:
            self._log('未下发：%s' % reason)
            self._set_msg(reason)
            return
        try:
            seq, packet = self.link.build(5, 0)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self._log('→ 冻结（停在当前位置）序号 %d' % seq)
        self._rec_note('HOLD')
        self._set_msg('冻结命令已发送')

    def _pick_speed(self):
        """档位按钮：先更新说明，连上板就直接下发，避免只改了界面。"""
        self._update_speed_desc()
        if self.connected and self.link.fresh:
            self._apply_speed()

    def _apply_speed(self):
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能下发速度')
            return
        try:
            step = P.clamp_step(int(float(self.var_speed.get())))
        except ValueError:
            messagebox.showerror('输入错误', '步长必须是 1~64 的整数')
            return
        self.var_speed.set(str(step))
        target = self.link.board_target
        if target is None:
            target = int(round((self.last_location or 0)))
        try:
            seq, packet = self.link.build(4, int(target), step)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self._log('→ 步长 %d 计数/50ms (≈%.0f°/s)，已用当前目标重发 序号 %d' %
                  (step, P.step_to_deg_s(step), seq))
        self._rec_note('STEP %d' % step)
        self._set_msg('速度命令已发送')
        self.applied_step = step
        self._board_step = step
        self._update_speed_desc()

    def _apply_ff(self):
        """op9：参考速度前馈系数，按百分比下发（100% = Q8 256）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能下发前馈')
            return
        try:
            pct = int(float(self.var_ff.get()))
        except ValueError:
            messagebox.showerror('输入错误', '前馈必须是 0~200 的百分比')
            return
        pct = max(0, min(200, pct))
        self.var_ff.set(str(pct))
        # 100% ≈ 25 个位置环输出单位 / (计数/拍)；固件再按 (gain × 步长) >> 8
        # 并限幅到位置环自己的输出上限，所以前馈不会压过闭环。
        gain = int(round(pct * 64.0))
        try:
            seq, packet = self.link.build(9, gain)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_ff_status.set('%d%%' % pct)
        self._log('→ 速度前馈 %d%%（Q8=%d）序号 %d' % (pct, gain, seq))
        self._rec_note('FF %d' % pct)
        self._set_msg('前馈命令已发送')

    def _apply_ff_cmd(self):
        """op11：指令前馈系数（0 关闭，256 = 100%）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能下发指令前馈')
            return
        try:
            pct = int(float(self.var_ff_cmd.get()))
        except ValueError:
            messagebox.showerror('输入错误', '指令前馈必须是 0~200 的百分比')
            return
        pct = max(0, min(200, pct))
        self.var_ff_cmd.set(str(pct))
        q8 = int(round(pct * 2.56))
        try:
            seq, packet = self.link.build(11, q8)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_ff_cmd_status.set('%d%%' % pct)
        self._log('→ 指令前馈 %d%%（Q8=%d）序号 %d' % (pct, q8, seq))
        self._rec_note('FFC %d' % pct)
        self._set_msg('指令前馈命令已发送')

    def _load_current_params(self):
        """把遥测里的当前值填到输入框（当前参数组的实际生效值）。"""
        d = self.last_frame
        if not d:
            self._set_msg('还没有遥测，先连上板子')
            return
        self.var_pid_a_kp.set(str(d.get('angle_kp', 0)))
        self.var_pid_a_ki.set(str(d.get('angle_ki', 0)))
        self.var_pid_a_kd.set(str(d.get('angle_kd', 0)))
        self.var_pid_p_kp.set(str(d.get('position_kp', 0)))
        self.var_pid_p_ki.set(str(d.get('position_ki', 0)))
        self.var_pid_p_kd.set(str(d.get('position_kd', 0)))
        if 'angle_center' in d:
            self.var_center.set(str(int(d['angle_center'])))
            self.var_center_status.set('板上 %d' % int(d['angle_center']))
        if d.get('manual_mode') in (0, 1):
            self.var_pid_bank.set(int(d['manual_mode']))
        self.var_pid_status.set('已从板子读取当前生效值')
        self._update_pid_diff()
        self._set_msg('已把板上的当前值读入输入框')

    def _update_param_now(self, frame):
        self.var_param_now.set(
            '角度 KP/KI/KD %d / %d / %d  ｜  位置 KP/KI/KD %d / %d / %d  ｜  中值 %d'
            % (frame.get('angle_kp', 0), frame.get('angle_ki', 0),
               frame.get('angle_kd', 0),
               frame.get('position_kp', 0), frame.get('position_ki', 0),
               frame.get('position_kd', 0),
               frame.get('angle_center', 0)))

    def _pid_inputs(self):
        return (int(float(self.var_pid_a_kp.get())),
                int(float(self.var_pid_a_ki.get())),
                int(float(self.var_pid_a_kd.get())),
                int(float(self.var_pid_p_kp.get())),
                int(float(self.var_pid_p_ki.get())),
                int(float(self.var_pid_p_kd.get())))

    def _open_vel_window(self):
        """实时速度对照：理想速度 vs 实测速度，边跑边更新。"""
        import matplotlib
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
        matplotlib.rcParams['axes.unicode_minus'] = False
        win = getattr(self, '_vel_win', None)
        if win is not None:
            try:
                win.deiconify()
                win.lift()
                return
            except Exception:                              # noqa: BLE001
                self._vel_win = None
        win = tk.Toplevel(self.root)
        win.title('实时速度对照：理想曲线 vs 实测')
        win.geometry('820x560')
        self._vel_win = win
        fig = Figure(figsize=(7.8, 5.2), dpi=100)
        ax1 = fig.add_subplot(211)
        ax2 = fig.add_subplot(212, sharex=ax1)
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill='both', expand=True)
        self._vel_fig, self._vel_ax1, self._vel_ax2 = fig, ax1, ax2
        self._vel_canvas = canvas

        def tick():
            if not win.winfo_exists():
                self._vel_win = None
                return
            try:
                self._vel_draw()
            except Exception:                              # noqa: BLE001
                pass
            win.after(200, tick)

        win.protocol('WM_DELETE_WINDOW',
                     lambda: (setattr(self, '_vel_win', None), win.destroy()))
        tick()

    def _auto_curve_vals(self, deg):
        """按目标角度挑内置曲线：<=120°→90°，>=270°→360°，其余→任意角度。"""
        try:
            data = self._read_curves()
        except Exception:                                  # noqa: BLE001
            return None
        if deg >= 270:
            name = '360° 定位'
        elif deg <= 120:
            name = '90° 定位'
        else:
            name = '任意角度'
        vals = data.get(name)
        if not vals:
            return None, name
        return [max(0, min(256, int(v))) for v in vals], name

    def _vel_stage_step(self):
        plan = self._curve_plan
        if not plan or not plan.get('stages'):
            return 0
        idx = min(max(0, plan.get('phase', 0) - 1), len(plan['stages']) - 1)
        return int(plan['stages'][idx][1])

    def _vel_draw(self):
        import numpy as np
        data = list(self.wave.data)
        ax1, ax2 = self._vel_ax1, self._vel_ax2
        ax1.clear()
        ax2.clear()
        span = getattr(self.wave, 'span', 15.0) or 15.0
        step_now = self._vel_stage_step()
        v_ref = step_now * 20.0 * P.DEG_PER_COUNT
        if len(data) > 10:
            t_all = np.array([d[0] for d in data])
            loc = np.array([d[2] for d in data])
            now = t_all[-1]
            keep = t_all >= now - span
            tt = t_all[keep] - (now - span)
            ll = loc[keep]
            v = np.zeros_like(ll)
            if len(ll) > 3:
                v[1:-1] = (ll[2:] - ll[:-2]) / np.maximum(1e-3, tt[2:] - tt[:-2])
                v = np.convolve(v, np.ones(3) / 3.0, mode='same')
            ymax = max(60.0, max(v_ref, float(np.abs(v).max())) * 1.3 + 10)
            ax1.plot(tt, np.full_like(tt, v_ref), '-', color='#1a7f37', lw=2)
            ax1.fill_between(tt, 0, v_ref, color='#1a7f37', alpha=0.12, lw=0)
            ax1.set_ylim(-20, ymax)
            ax1.set_ylabel('理想速度 (°/s)')
            ax1.set_title('① 上位机规划（当前段步长 %d ≈ %.0f°/s）'
                          % (step_now, v_ref))
            ax2.plot(tt, v, '-', color='#0969da', lw=1.8)
            ax2.fill_between(tt, 0, v, color='#0969da', alpha=0.12, lw=0)
            ax2.axhline(0, color='#d0d7de', lw=0.8)
            ax2.set_ylim(-20, ymax)
            ax2.set_ylabel('实测速度 (°/s)')
            ax2.set_xlabel('最近 %.0f 秒' % span)
            ax2.set_title('② 编码器实测')
        else:
            ax1.text(0.5, 0.5, '等待数据…', ha='center', va='center',
                     transform=ax1.transAxes)
        for ax in (ax1, ax2):
            ax.grid(True, color='#eaeef2', lw=0.8)
            for side in ('top', 'right'):
                ax.spines[side].set_visible(False)
        self._vel_fig.tight_layout()
        self._vel_canvas.draw_idle()

    def _open_keypad(self):
        """软件键盘：给触摸屏/演示用，直接输入目标角度并执行位置控制。"""
        win = getattr(self, '_keypad', None)
        if win is not None:
            try:
                win.deiconify()
                win.lift()
                return
            except Exception:                              # noqa: BLE001
                self._keypad = None
        win = tk.Toplevel(self.root)
        win.title('软件键盘 · 位置控制')
        win.transient(self.root)
        win.resizable(False, False)
        self._keypad = win
        big = ('Microsoft YaHei UI', 15)
        card = ttk.Frame(win, padding=12)
        card.pack(fill='both', expand=True)

        ttk.Label(card, textvariable=self.var_cur_deg,
                  font=('Microsoft YaHei UI', 12)).grid(
            row=0, column=0, columnspan=3, sticky='w')
        disp = ttk.Label(card, textvariable=self.var_entry,
                         font=('Consolas', 26, 'bold'), width=10,
                         anchor='e', relief='sunken', padding=4)
        disp.grid(row=1, column=0, columnspan=3, sticky='ew', pady=(4, 8))

        def key(ch):
            text = self.var_entry.get().strip()
            if ch == 'C':
                text = ''
            elif ch == 'B':
                text = text[:-1]
            elif ch == 'S':
                text = text[1:] if text.startswith('-') else '-' + text
            else:
                if ch == '.' and '.' in text:
                    return
                text += ch
            self.var_entry.set(text)

        keys = [('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
                ('4', 3, 0), ('5', 3, 1), ('6', 3, 2),
                ('1', 4, 0), ('2', 4, 1), ('3', 4, 2),
                ('±', 5, 0), ('0', 5, 1), ('.', 5, 2)]
        for txt, r, c in keys:
            ch = {'±': 'S', '.': '.'}.get(txt, txt)
            ttk.Button(card, text=txt, width=5, style='Key.TButton',
                       command=lambda k=ch: key(k)).grid(
                row=r, column=c, padx=3, pady=3, sticky='nsew')
        ttk.Button(card, text='⌫', width=5, style='Key.TButton',
                   command=lambda: key('B')).grid(row=6, column=0, padx=3, pady=3,
                                                  sticky='nsew')
        ttk.Button(card, text='清空', width=5, style='Key.TButton',
                   command=lambda: key('C')).grid(row=6, column=1, padx=3, pady=3,
                                                  sticky='nsew')
        ttk.Button(card, text='执行', width=5, style='Go.TButton',
                   command=self._keypad_go).grid(row=6, column=2, padx=3, pady=3,
                                                 sticky='nsew')

        quick = ttk.Frame(card)
        quick.grid(row=7, column=0, columnspan=3, pady=(10, 0), sticky='ew')
        for i, (txt, deg) in enumerate((('+90', 90), ('-90', -90),
                                        ('+180', 180), ('-180', -180))):
            ttk.Button(quick, text=txt, width=6, style='Key.TButton',
                       command=lambda d=deg: self._keypad_nudge(d)).grid(
                row=0, column=i, padx=3, sticky='ew')

        acts = ttk.Frame(card)
        acts.grid(row=8, column=0, columnspan=3, pady=(8, 0), sticky='ew')
        ttk.Button(acts, text='回 0°', width=8, style='Key.TButton',
                   command=lambda: (self.var_entry.set('0'), self._goto_entry())
                   ).grid(row=0, column=0, padx=3, sticky='ew')
        ttk.Button(acts, text='停住', width=8, style='Key.TButton',
                   command=self._hold_here).grid(row=0, column=1, padx=3, sticky='ew')
        ttk.Button(acts, text='停止曲线', width=8, style='Key.TButton',
                   command=self._stop_curve).grid(row=0, column=2, padx=3, sticky='ew')

        ttk.Label(card, text='点数字 → 点“执行”；“±”换方向；也可以直接用左侧输入框+回车',
                  style='Muted.TLabel', wraplength=330, justify='left').grid(
            row=9, column=0, columnspan=3, sticky='w', pady=(8, 0))
        win.protocol('WM_DELETE_WINDOW', lambda: (setattr(self, '_keypad', None),
                                                  win.destroy()))

    def _keypad_go(self):
        text = self.var_entry.get().strip()
        if not text or text in ('-', '.'):
            self._set_msg('先在键盘上输入目标角度')
            return
        self._goto_entry()

    def _keypad_nudge(self, delta):
        base = self._base_target()
        target = base + P.deg_to_counts(delta)
        self.var_entry.set('%.1f' % P.counts_to_deg(P.clamp_position(target)))
        self._send_position(target, '键盘 %+d°' % delta)

    def _preset_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'config', 'pid_presets.json')

    def _load_presets(self):
        try:
            with open(self._preset_path(), encoding='utf-8') as fh:
                data = json.load(fh)
            if isinstance(data, list):
                return data
        except (OSError, ValueError):
            pass
        src = getattr(self, 'src_pid', {}) or {}
        if not src:
            return []
        return [{'name': '源码默认', 'bank': 0,
                 'a': [src.get('KP_A_AUTO', 0), src.get('KI_A_AUTO', 0),
                       src.get('KD_A_AUTO', 0)],
                 'p': [src.get('KP_P_AUTO', 0), src.get('KI_P_AUTO', 0),
                       src.get('KD_P_AUTO', 0)],
                 'ks': 10, 'vp': 0, 'saved_at': '初始'}]

    def _save_presets(self):
        try:
            path = self._preset_path()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(self._presets, fh, ensure_ascii=False, indent=1)
        except OSError as exc:                             # noqa: BLE001
            self._log('预设保存失败：%s' % exc)

    def _refresh_preset_list(self):
        names = [p.get('name', '?') for p in self._presets]
        try:
            self.cmb_preset['values'] = names
        except Exception:                                  # noqa: BLE001
            pass
        self.var_preset_info.set('已存 %d 套：%s' % (len(names), '、'.join(names))
                                 if names else '还没有预设，填个名字点“保存为预设”')

    def _preset_fields(self):
        kp_a, ki_a, kd_a, kp_p, ki_p, kd_p = self._pid_inputs()
        try:
            ks = max(0, min(12, int(float(self.var_ks.get() or 0))))
            vp = max(0, min(400, int(float(self.var_vp.get() or 0))))
        except ValueError:
            ks = vp = 0
        return {'a': [kp_a, ki_a, kd_a], 'p': [kp_p, ki_p, kd_p],
                'ks': ks, 'vp': vp, 'bank': int(self.var_pid_bank.get())}

    def _preset_save(self):
        name = self.var_preset_name.get().strip() or '未命名'
        try:
            data = self._preset_fields()
        except ValueError:
            messagebox.showerror('输入错误', 'PID 参数必须是整数')
            return
        data['name'] = name
        data['saved_at'] = _dt.datetime.now().strftime('%m-%d %H:%M')
        self._presets = [p for p in self._presets if p.get('name') != name]
        self._presets.append(data)
        self._save_presets()
        self._refresh_preset_list()
        self.var_preset_name.set(name)
        self._set_msg('已保存预设「%s」' % name)
        self._log('保存参数预设「%s」：角度 %d/%d/%d 位置 %d/%d/%d KS %d 速度环 %d%%'
                  % (name, data['a'][0], data['a'][1], data['a'][2],
                     data['p'][0], data['p'][1], data['p'][2],
                     data['ks'], data['vp']))

    def _preset_find(self):
        name = self.var_preset_name.get().strip()
        for p in self._presets:
            if p.get('name') == name:
                return p
        return None

    def _preset_fill(self):
        p = self._preset_find()
        if p is None:
            self._set_msg('没有叫「%s」的预设' % self.var_preset_name.get().strip())
            return
        try:
            self.var_pid_a_kp.set(str(p['a'][0]))
            self.var_pid_a_ki.set(str(p['a'][1]))
            self.var_pid_a_kd.set(str(p['a'][2]))
            self.var_pid_p_kp.set(str(p['p'][0]))
            self.var_pid_p_ki.set(str(p['p'][1]))
            self.var_pid_p_kd.set(str(p['p'][2]))
            self.var_ks.set(str(p.get('ks', 0)))
            self.var_vp.set(str(p.get('vp', 0)))
            self.var_pid_bank.set(int(p.get('bank', 0)))
        except (KeyError, IndexError, TypeError):
            self._set_msg('预设「%s」格式不对' % p.get('name'))
            return
        self.var_pid_status.set('已载入预设「%s」（未写板）' % p['name'])
        self._update_pid_diff()
        self._set_msg('已把预设「%s」载入输入框，点“一键写入”下发' % p['name'])

    def _preset_apply(self):
        self._preset_fill()
        if self._preset_find() is None:
            return
        self._write_all()

    def _preset_delete(self):
        name = self.var_preset_name.get().strip()
        p = self._preset_find()
        if p is None:
            self._set_msg('没有叫「%s」的预设' % name)
            return
        if not messagebox.askyesno('删除预设', '删除预设「%s」？' % name):
            return
        self._presets = [x for x in self._presets if x.get('name') != name]
        self._save_presets()
        self._refresh_preset_list()
        self._set_msg('已删除预设「%s」' % name)

    def _write_all(self):
        """一键写入：PID（整套）→ 摩擦 KS → 速度环 P，逐条发、等回执。"""
        self._batch = []
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能写入')
            return
        try:
            kp_a, ki_a, kd_a, kp_p, ki_p, kd_p = self._pid_inputs()
            ks = max(0, min(12, int(float(self.var_ks.get() or 0))))
            vp = max(0, min(400, int(float(self.var_vp.get() or 0))))
        except ValueError:
            messagebox.showerror('输入错误', 'PID / KS / 速度环 P 必须是整数')
            return
        bank = int(self.var_pid_bank.get())
        self._batch = [
            ('pid', (kp_a, ki_a, kd_a, kp_p, ki_p, kd_p), bank),
            ('ks', ks, 0),
            ('vp', vp, 0),
        ]
        self.var_pid_status.set('一键写入中…')
        self._log('一键写入开始：PID %d/%d/%d·%d/%d/%d，KS %d，速度环 %d%%'
                  % (kp_a, ki_a, kd_a, kp_p, ki_p, kd_p, ks, vp))
        self._batch_step()

    def _batch_step(self):
        if not self._batch:
            self.var_pid_status.set('一键写入完成 ✓')
            self._set_msg('一键写入完成：PID / 摩擦 KS / 速度环 P')
            self._log('一键写入完成')
            self._update_pid_diff()
            return
        if self.link.pending is not None:
            waited = time.monotonic() - getattr(self, '_batch_t', time.monotonic())
            if waited > 1.2:
                self.link.pending = None            # 回执丢了，别卡住整批
                self._log('一键写入：上一条回执超时，跳过')
            else:
                self.root.after(60, self._batch_step)
                return
        kind, value, bank = self._batch.pop(0)
        try:
            if kind == 'pid':
                gains = value
                seq, packet = self.link.build(1, gains[0], bank, gains=gains)
            elif kind == 'ks':
                seq, packet = self.link.build(7, value)
            else:
                seq, packet = self.link.build(15, int(round(value * 2.56)))
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._batch = None
            self.var_pid_status.set('一键写入失败：%s' % exc)
            self._set_msg('一键写入失败：%s' % exc)
            return
        self._batch_t = time.monotonic()
        self.root.after(120, self._batch_step)

    def _write_pid(self, which='all'):
        """op1：写 PID。which='a' 只改角度环、'p' 只改位置环，另一环用板上当前值。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能写入 PID')
            return
        try:
            kp_a, ki_a, kd_a, kp_p, ki_p, kd_p = self._pid_inputs()
        except ValueError:
            messagebox.showerror('输入错误', '6 个 PID 参数都必须是整数')
            return
        d = self.last_frame or {}
        src = '界面'
        if which == 'p' and d:               # 只改位置环：角度用板上的
            kp_a = d.get('angle_kp', kp_a)
            ki_a = d.get('angle_ki', ki_a)
            kd_a = d.get('angle_kd', kd_a)
            src = '板上'
        elif which == 'a' and d:             # 只改角度环：位置用板上的
            kp_p = d.get('position_kp', kp_p)
            ki_p = d.get('position_ki', ki_p)
            kd_p = d.get('position_kd', kd_p)
            src = '板上'
        bank = int(self.var_pid_bank.get())
        # 固件按 {kd_p, ki_p, kp_p, kd_a, ki_a, kp_a} 从高到低拼接 pending_gains，
        # 所以帧里的 6 个字段必须是「角度 KP/KI/KD，位置 KP/KI/KD」这个顺序。
        gains = (kp_a, ki_a, kd_a, kp_p, ki_p, kd_p)
        try:
            seq, packet = self.link.build(1, gains[0], bank, gains=gains)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        label = {'a': '仅角度环', 'p': '仅位置环'}.get(which, '整套')
        # 写完后用遥测回读核对（0.6 s 后），防止“发了但没生效”
        self._pid_check = (time.monotonic() + 0.6, bank,
                           (kp_a, ki_a, kd_a, kp_p, ki_p, kd_p))
        self._log('→ 写入 PID（%s组，%s；未改的一环用%s值）'
                  ' 角度 %d/%d/%d 位置 %d/%d/%d 序号 %d'
                  % ('手动' if bank else '自动', label, src,
                     kp_a, ki_a, kd_a, kp_p, ki_p, kd_p, seq))
        self._rec_note('PID %d %s A=%d/%d/%d P=%d/%d/%d'
                       % (bank, label, kp_a, ki_a, kd_a, kp_p, ki_p, kd_p))
        self.var_pid_status.set('已发送（%s组，%s）' % ('手动' if bank else '自动', label))
        live = d.get('manual_mode')
        if live in (0, 1) and int(live) != bank:
            self.var_pid_status.set(
                '已发送（%s组，%s）· 注意：板子当前跑%s组，这组要切过去才生效'
                % ('手动' if bank else '自动', label,
                   '手动' if int(live) else '自动'))
        self._update_pid_diff()
        self._set_msg('PID 已发送，等确认')

    def _pid_write_check(self):
        """PID 写完后回读核对：板上生效值是否等于刚发的值。"""
        check = getattr(self, '_pid_check', None)
        if not check:
            return
        due, bank, want = check
        if time.monotonic() < due:
            return
        self._pid_check = None
        d = self.last_frame or {}
        live = d.get('manual_mode')
        if live in (0, 1) and int(live) != bank:
            return                        # 写的是另一组，回读对不上属正常
        got = (d.get('angle_kp'), d.get('angle_ki'), d.get('angle_kd'),
               d.get('position_kp'), d.get('position_ki'), d.get('position_kd'))
        if tuple(got) == tuple(want):
            self.var_pid_status.set('板上已生效 ✓  角度 %d/%d/%d  位置 %d/%d/%d'
                                    % want)
            self._log('PID 回读一致（角度 %d/%d/%d 位置 %d/%d/%d）✓' % want)
        else:
            self.var_pid_status.set('回读不一致！板上 %s，期望 %s' % (got, want))
            self._log('PID 回读不一致：板上 %s，期望 %s' % (got, want))

    def _update_pid_diff(self):
        """把“界面值”和“板子上当前值”的差异列出来，提醒还没写回。"""
        d = self.last_frame
        if not d:
            self.var_pid_diff.set('')
            return
        names = (('角度 KP', self.var_pid_a_kp, d.get('angle_kp')),
                 ('角度 KI', self.var_pid_a_ki, d.get('angle_ki')),
                 ('角度 KD', self.var_pid_a_kd, d.get('angle_kd')),
                 ('位置 KP', self.var_pid_p_kp, d.get('position_kp')),
                 ('位置 KI', self.var_pid_p_ki, d.get('position_ki')),
                 ('位置 KD', self.var_pid_p_kd, d.get('position_kd')))
        diff = []
        for label, var, board in names:
            try:
                if int(float(var.get())) != int(board or 0):
                    diff.append(label)
            except (TypeError, ValueError):
                diff.append(label)
        if diff:
            self.var_pid_diff.set('未写回：' + '、'.join(diff))
        else:
            self.var_pid_diff.set('与板上一致')

    def _write_center(self):
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能写中值')
            return
        try:
            value = int(float(self.var_center.get()))
        except ValueError:
            messagebox.showerror('输入错误', '中值必须是 0~4095 的整数')
            return
        value = max(0, min(4095, value))
        self.var_center.set(str(value))
        bank = int(self.var_pid_bank.get())
        try:
            seq, packet = self.link.build(13, value, bank)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self._log('→ 倒立摆中值 %d（%s组）序号 %d' % (value, '手动' if bank else '自动', seq))
        self._rec_note('CENTER %d' % value)
        self._set_msg('中值已发送')

    def _apply_ff(self):
        """op16：加速度前馈增益（Q8）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能写前馈')
            return
        try:
            q8 = int(float(self.var_ff_gain.get() or 0))
        except ValueError:
            messagebox.showerror('输入错误', '前馈增益必须是整数（Q8）')
            return
        q8 = max(0, min(4096, q8))
        self.var_ff_gain.set(str(q8))
        try:
            seq, packet = self.link.build(16, q8)
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_ff_gain_status.set('已下发 %d (%.2f)' % (q8, q8 / 256.0))
        self._log('→ 加速度前馈 %d（%.2f）序号 %d' % (q8, q8 / 256.0, seq))
        self._rec_note('FF %d' % q8)
        self._set_msg('前馈已发送')

    def _apply_vp(self):
        """op15：速度环 P（0 = 关闭，256 = 100%）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能下发速度环 P')
            return
        try:
            pct = int(float(self.var_vp.get()))
        except ValueError:
            messagebox.showerror('输入错误', '速度环 P 必须是 0~400 的百分比')
            return
        pct = max(0, min(400, pct))
        self.var_vp.set(str(pct))
        q8 = int(round(pct * 2.56))
        try:
            seq, packet = self.link.build(15, q8)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_vp_status.set('%d%%' % pct)
        self._log('→ 速度环 P %d%%（Q8=%d）序号 %d' % (pct, q8, seq))
        self._rec_note('VP %d' % pct)
        self._set_msg('速度环 P 已发送')

    def _write_ks(self):
        """op7：摩擦前馈 KS（0~12）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能写摩擦 KS')
            return
        try:
            ks = int(float(self.var_ks.get()))
        except ValueError:
            messagebox.showerror('输入错误', 'KS 必须是 0~12 的整数')
            return
        ks = max(0, min(12, ks))
        self.var_ks.set(str(ks))
        try:
            seq, packet = self.link.build(7, ks)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_ks_status.set('已下发 %d' % ks)
        self._log('→ 摩擦前馈 KS %d 序号 %d' % (ks, seq))
        self._rec_note('KS %d' % ks)
        self._set_msg('摩擦 KS 已发送')

    def _write_center(self):
        """op13：平衡点中值（12bit），写当前参数组。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能写中值')
            return
        try:
            value = int(float(self.var_center.get()))
        except ValueError:
            messagebox.showerror('输入错误', '中值必须是 0~4095 的整数')
            return
        value = max(0, min(4095, value))
        self.var_center.set(str(value))
        bank = int(self.var_pid_bank.get())
        try:
            seq, packet = self.link.build(13, value, bank)
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_center_status.set('已下发 %d' % value)
        self._log('→ 平衡点中值 %d（%s组）序号 %d'
                  % (value, '手动' if bank else '自动', seq))
        self._rec_note('CENTER %d' % value)
        self._set_msg('中值已发送')

    def _load_preset_file(self):
        """读 config/preset.json，把调好的参数填进界面（不自动下发）。"""
        import json
        import os
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'config', 'preset.json')
        if not os.path.exists(path):
            return
        try:
            with open(path, encoding='utf-8') as fh:
                data = json.load(fh)
        except Exception:                                  # noqa: BLE001
            return
        pairs = (('angle_kp', 'var_pid_a_kp'), ('angle_ki', 'var_pid_a_ki'),
                 ('angle_kd', 'var_pid_a_kd'), ('position_kp', 'var_pid_p_kp'),
                 ('position_ki', 'var_pid_p_ki'), ('position_kd', 'var_pid_p_kd'),
                 ('ks', 'var_ks'), ('vp_percent', 'var_vp'),
                 ('center', 'var_center'), ('step90', 'var_step90'),
                 ('step360', 'var_step360'))
        for key, var in pairs:
            if key in data and hasattr(self, var):
                getattr(self, var).set(str(data[key]))
        if 'auto_step' in data:
            self.var_auto_step.set(bool(data['auto_step']))

    def _save_preset_file(self):
        """把当前界面参数写回 config/preset.json。"""
        import json
        import os
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'config', 'preset.json')
        data = {
            'angle_kp': int(float(self.var_pid_a_kp.get() or 0)),
            'angle_ki': int(float(self.var_pid_a_ki.get() or 0)),
            'angle_kd': int(float(self.var_pid_a_kd.get() or 0)),
            'position_kp': int(float(self.var_pid_p_kp.get() or 0)),
            'position_ki': int(float(self.var_pid_p_ki.get() or 0)),
            'position_kd': int(float(self.var_pid_p_kd.get() or 0)),
            'ks': int(float(self.var_ks.get() or 0)),
            'vp_percent': int(float(self.var_vp.get() or 0)),
            'center': int(float(self.var_center.get() or 0)),
            'step90': int(float(self.var_step90.get() or 16)),
            'step360': int(float(self.var_step360.get() or 24)),
            'auto_step': bool(self.var_auto_step.get()),
            'curve_stream': bool(self.var_curve_stream.get()),
        }
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2)
            self._log('参数预设已保存到 config/preset.json')
            self._set_msg('预设已保存')
        except Exception as exc:                           # noqa: BLE001
            self._set_msg('保存预设失败：%s' % exc)

    def _read_ks(self):
        """op7 + -1：让板子用 bank=0x80 回一帧，把 KS 带回来。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能读 KS')
            return
        try:
            seq, packet = self.link.build_ks_read()
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._set_msg('发送失败：%s' % exc)
            return
        self._ks_read_pending = True
        self._log('→ 读取板上摩擦 KS 序号 %d' % seq)

    def _write_defaults(self):
        """op3：当前参数组恢复固件默认增益（含摩擦 KS）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能恢复默认')
            return
        if not messagebox.askyesno('恢复默认', '把当前参数组的 PID 增益恢复到源码默认值？'):
            return
        try:
            seq, packet = self.link.build(3, 0)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_ks.set('0')
        self.var_ks_status.set('已回默认 0')
        self._log('→ 恢复默认增益 序号 %d' % seq)
        self._rec_note('DEFAULTS')
        self._set_msg('恢复默认已发送')
        self.root.after(700, self._load_current_params)

    def _apply_deadzone(self):
        """op12：自动模式输出死区（占空比 0~255）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能下发死区')
            return
        try:
            value = int(float(self.var_deadzone.get()))
        except ValueError:
            messagebox.showerror('输入错误', '死区必须是 0~255 的整数')
            return
        value = max(0, min(255, value))
        self.var_deadzone.set(str(value))
        try:
            seq, packet = self.link.build(12, value)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self.var_deadzone_status.set('已下发 %d' % value)
        self._log('→ 角度死区 %d（占空比）序号 %d' % (value, seq))
        self._rec_note('DZ %d' % value)
        self._set_msg('角度死区已发送')

    def _apply_damp(self):
        """op10：位置环阻尼倍率（100% = 原参数，不动）。"""
        if not self.connected or not self.link.fresh:
            self._set_msg('未连接或没有遥测，不能下发阻尼')
            return
        try:
            pct = int(float(self.var_damp.get()))
        except ValueError:
            messagebox.showerror('输入错误', '阻尼必须是 50~400 的百分比')
            return
        pct = max(50, min(400, pct))
        self.var_damp.set(str(pct))
        q8 = int(round(pct * 2.56))
        try:
            seq, packet = self.link.build(10, q8)
            self.worker.send(packet)
        except Exception as exc:
            self._set_msg('发送失败：%s' % exc)
            return
        self._log('→ 位置环阻尼 %d%%（Q8=%d）序号 %d' % (pct, q8, seq))
        self._rec_note('DAMP %d' % pct)
        self._set_msg('阻尼命令已发送')

    def _send_scale(self):
        try:
            step = P.clamp_step(int(float(self.var_speed.get())))
        except ValueError:
            return
        scale = P.profile_scale(self.curve_editor.get_vals(), step)
        if not scale:
            return
        try:
            seq, packet = self.link.build(8, scale)
            self.worker.send(packet)
        except Exception as exc:
            self._log('时间轴下发失败：%s' % exc)
            return
        self._log('→ 曲线时间轴 %d（步长 %d）序号 %d' % (scale, step, seq))
        self._rec_note('CURVE_SCALE %d' % scale)

    def _update_speed_desc(self):
        try:
            step = P.clamp_step(int(float(self.var_speed.get())))
        except ValueError:
            self.var_speed_desc.set('步长需为 1~64 的整数')
            return
        deg_s = P.step_to_deg_s(step)
        text = ('≈ %.0f°/s；转 90° 约 %.1fs，转一圈约 %.1fs'
                % (deg_s, 90 / deg_s if deg_s else 0, 360 / deg_s if deg_s else 0))
        text += '　（随每次定位一起下发）'
        self.var_speed_desc.set(text)
        try:
            self._update_curve_info2()
        except Exception:
            pass

    # ------------------------------------------------------------ 摇杆
    def _jog_allowed(self):
        ok, _ = self._link_ready()
        return ok and self.last_state == 4

    def _on_joystick(self, dx, _dy):
        """摇杆 = 增量点动：按动摇杆每 150 ms 把目标往前推一点（op4）。"""
        try:
            vmax = max(1, P.clamp_jog(abs(int(float(self.var_jog_max.get())))))
        except ValueError:
            vmax = 16
        speed = 0 if abs(dx) < 0.08 else int(round(dx * vmax))
        if self.var_jog_invert.get():
            speed = -speed
        self._jog_pending = P.clamp_jog(speed)
        self._update_jog_desc()
        if self._jog_timer is None:
            self._jog_timer = self.root.after(30, self._jog_tick)

    def _jog_tick(self):
        """摇杆点动：目标永远放在转臂前方 0.8 秒的路程，永不被追上。

        固件只有"走到目标"，只要目标不被追上，它就一直跑、不会刹车；
        速度由步长决定（摇杆推多少 → 步长多少），所以手感是连续的。
        """
        self._jog_timer = None
        speed = self._jog_pending
        if speed == 0:
            if self._jog_moved:
                self._jog_moved = False
                self._hold_here()
                self._log('摇杆松开：冻结在当前角度')
            self._jog_target = None
            return
        if not self._jog_allowed() or self.last_location is None:
            self._jog_timer = self.root.after(80, self._jog_tick)
            return
        sign = 1 if speed > 0 else -1
        loc = self.last_location
        step = max(3, min(24, abs(speed)))
        v = step * 20.0                      # 计数/秒（固件每 50 ms 走 step 个计数）
        lead = v * 0.8 * sign                # 前导 0.8 秒（带方向！）
        if abs(lead) < 200.0:                # 太短会被追上而刹车，给个下限
            lead = 200.0 * sign
        want = P.clamp_position(loc + lead)
        self._jog_target = want
        try:
            _, packet = self.link.build(4, int(want), step)
            self.worker.send(packet)
        except Exception as exc:                           # noqa: BLE001
            self._log('摇杆发送失败：%s' % exc)
            return
        self._jog_moved = True
        self._jog_timer = self.root.after(80, self._jog_tick)

    def _update_jog_desc(self):
        if not self._jog_allowed():
            self.var_jog_desc.set('未平衡或未连接：摇杆暂时无效')
            return
        speed = self._jog_pending
        self.var_jog_desc.set(
            '指令 %+d 计/51ms ≈ %+.0f°/s；松手停在原地'
            % (speed, speed * P.DEG_PER_COUNT / P.POS_LOOP_S))

    def _flush_jog(self):
        self._jog_after = None
        speed = self._jog_pending
        if speed == self._jog_sent:
            return
        if not self._jog_allowed():
            self._jog_sent = speed
            self._update_jog_desc()
            return
        if self.link.busy:
            self._jog_after = self.root.after(30, self._flush_jog)
            return
        try:
            seq, packet = self.link.build(7, speed)
            self.worker.send(packet)
        except Exception as exc:
            self._log('摇杆发送失败：%s' % exc)
            return
        self._jog_sent = speed
        self._log('摇杆 %+d 计/51ms（≈%+.0f°/s）序号 %d'
                  % (speed, speed * P.DEG_PER_COUNT / P.POS_LOOP_S, seq))
        self._rec_note('JOG %+d' % speed)
        self._update_jog_desc()

    def _jog_stop(self):
        self._jog_pending = 0
        if self._jog_timer is not None:
            try:
                self.root.after_cancel(self._jog_timer)
            except Exception:                              # noqa: BLE001
                pass
            self._jog_timer = None
        self._jog_target = None
        self._jog_moved = False
        self.joystick.reset()
        self._hold_here()

    # ------------------------------------------------------------ 速度曲线
    def _curve_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'config', 'curve.json')

    # ------------------------------------------------------------ 会话参数
    def _session_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'config', 'session.json')

    def _save_session(self):
        """关闭时保存全部可手调的输入，下次打开原样恢复。"""
        def val(var, default=''):
            try:
                return var.get()
            except Exception:                              # noqa: BLE001
                return default

        data = {
            'speed': val(self.var_speed, '8'),
            'port': val(self.var_port, ''),
            'auto_rec': bool(getattr(self, 'var_auto_rec', None)
                             and self.var_auto_rec.get()),
            'span': getattr(self, 'cmb_span', None) and self.cmb_span.get() or '',
            'jog_max': val(getattr(self, 'var_jog_max', None) or tk.StringVar(), ''),
            'ks': val(self.var_ks, '0'),
            'vp': val(self.var_vp, '0'),
            'center': val(self.var_center, '2965'),
            'step90': val(self.var_step90, '8'),
            'step360': val(self.var_step360, '12'),
            'auto_step': bool(getattr(self, 'var_auto_step', None)
                              and self.var_auto_step.get()),
            'curve_exec': bool(getattr(self, 'var_curve_exec', None)
                               and self.var_curve_exec.get()),
            'pid_bank': int(self.var_pid_bank.get()) if hasattr(self, 'var_pid_bank') else 0,
            'pid_a': [val(self.var_pid_a_kp, ''), val(self.var_pid_a_ki, ''),
                      val(self.var_pid_a_kd, '')],
            'pid_p': [val(self.var_pid_p_kp, ''), val(self.var_pid_p_ki, ''),
                      val(self.var_pid_p_kd, '')],
            'curve_name': val(getattr(self, 'var_curve_name', None) or tk.StringVar(), ''),
            'curve_target': val(getattr(self, 'var_curve_target', None) or tk.StringVar(), ''),
            'auto_scale': bool(getattr(self, 'var_auto_scale', None)
                               and self.var_auto_scale.get()),
        }
        try:
            data['curve_vals'] = [int(v) for v in self.curve_editor.get_vals()]
        except Exception:                                  # noqa: BLE001
            pass
        try:
            path = self._session_path()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2)
        except OSError as exc:                             # noqa: BLE001
            self._log('保存界面状态失败：%s' % exc)

    def _restore_session(self, keep_port=False):
        """把上次关闭时的输入全部填回界面。"""
        if os.environ.get('POSITION_HOST_NO_SESSION') == '1':
            return
        try:
            with open(self._session_path(), encoding='utf-8') as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            return
        if not isinstance(data, dict):
            return

        def put(var, raw, cast=str, lo=None, hi=None):
            try:
                value = cast(raw)
                if lo is not None:
                    value = max(lo, value)
                if hi is not None:
                    value = min(hi, value)
                var.set(str(value) if cast is int else value)
            except (TypeError, ValueError, AttributeError):
                pass

        put(self.var_speed, data.get('speed', '8'), int, 1, 64)
        try:
            self._update_speed_desc()
        except Exception:                                  # noqa: BLE001
            pass
        port = data.get('port')
        if (not keep_port and isinstance(port, str)
                and port in list(self.cmb_port['values'])):
            self.var_port.set(port)
        if 'auto_rec' in data and hasattr(self, 'var_auto_rec'):
            self.var_auto_rec.set(bool(data['auto_rec']))
        span = str(data.get('span', ''))
        if hasattr(self, 'cmb_span') and span in tuple(self.cmb_span['values']):
            self.cmb_span.set(span)
            self._on_span()
        if hasattr(self, 'var_jog_max') and data.get('jog_max'):
            put(self.var_jog_max, data['jog_max'], str)

        put(self.var_step90, data.get('step90', '8'), int, 1, 64)
        put(self.var_step360, data.get('step360', '12'), int, 1, 64)
        put(self.var_center, data.get('center', '2965'), int, 0, 4095)
        put(self.var_ks, data.get('ks', '10'), int, 0, 12)
        put(self.var_vp, data.get('vp', '250'), int, 0, 400)
        if 'auto_step' in data:
            self.var_auto_step.set(bool(data['auto_step']))
        if 'curve_exec' in data and hasattr(self, 'var_curve_exec'):
            self.var_curve_exec.set(bool(data['curve_exec']))
        if 'pid_bank' in data and hasattr(self, 'var_pid_bank'):
            self.var_pid_bank.set(1 if data['pid_bank'] else 0)
        pa = data.get('pid_a') or []
        pp = data.get('pid_p') or []
        if len(pa) == 3:
            put(self.var_pid_a_kp, pa[0], str)
            put(self.var_pid_a_ki, pa[1], str)
            put(self.var_pid_a_kd, pa[2], str)
        if len(pp) == 3:
            put(self.var_pid_p_kp, pp[0], str)
            put(self.var_pid_p_ki, pp[1], str)
            put(self.var_pid_p_kd, pp[2], str)
        if hasattr(self, 'var_curve_name') and data.get('curve_name'):
            put(self.var_curve_name, data['curve_name'], str)
        if hasattr(self, 'var_curve_target') and data.get('curve_target'):
            put(self.var_curve_target, data['curve_target'], str)
        vals = data.get('curve_vals')
        if isinstance(vals, list) and len(vals) == 16:
            try:
                self.curve_editor.set_vals([int(v) for v in vals])
            except Exception:                              # noqa: BLE001
                pass

    def _on_close(self):
        """关窗口：先保存界面参数，再停记录、停串口线程，最后退出。"""
        try:
            self._save_session()
        except Exception:                                  # noqa: BLE001
            pass
        if self.recording:
            self._stop_record()
        for attr in ('_poll_id', '_draw_id'):
            after_id = getattr(self, attr, None)
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except Exception:                          # noqa: BLE001
                    pass
        if self.worker is not None:
            self.worker.stop()
        self.root.after(120, self.root.destroy)


def main(argv=None):
    parser = argparse.ArgumentParser(description='倒立摆位置控制上位机')
    parser.add_argument('--port', help='启动时自动选中的串口，例如 COM5')
    parser.add_argument('--demo', action='store_true',
                        help='演示模式：不接硬件，用模拟数据预览界面')
    args = parser.parse_args(argv)

    if sys.platform == 'win32':
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    root = tk.Tk()
    App(root, port=args.port, demo=args.demo)
    root.mainloop()


if __name__ == '__main__':
    main()
