"""Tk panel for arm position goto/hold (USB op4/op5, trapezoidal speed profile)."""
import math
import time
import tkinter as tk
from tkinter import ttk
from runtime_pid import (COUNTS_PER_REV, POSITION_LIMIT_COUNTS,
                         POSITION_STEP_MAX, position_result_text)

STEP_DEFAULT = 8
STEP_MIN = 1
BALANCE_STATE = 4
FRESH_SECONDS = 1
ARRIVE_TOL_COUNTS = 3
ARRIVE_SECONDS = 1
STATE_NAMES = {0:'停止', 1:'峰值判断', 4:'平衡', 21:'正向冲量', 22:'正向保持',
               23:'反向冲量', 24:'反向保持', 31:'反向冲量', 32:'反向保持',
               33:'正向冲量', 34:'正向保持'}


def degrees_to_counts(degrees):
    """Convert arm angle in degrees to encoder counts (1063 counts = 360°)."""
    return int(round(float(degrees)*COUNTS_PER_REV/360.0))


def counts_to_degrees(counts):
    return counts*360.0/COUNTS_PER_REV


def format_degrees(counts):
    return f'{counts_to_degrees(counts):+.1f}°'


def validate_target_degrees(text):
    """Parse the target entry (degrees) and return (signed counts, degrees)."""
    try:
        degrees = float(str(text).strip())
    except (TypeError, ValueError):
        raise ValueError('目标角度必须是数字（单位：度）')
    if not math.isfinite(degrees):
        raise ValueError('目标角度必须是有限数字')
    counts = degrees_to_counts(degrees)
    if not -POSITION_LIMIT_COUNTS <= counts <= POSITION_LIMIT_COUNTS:
        raise ValueError('目标超出 ±10 圈（±3600°，即 ±10630 计数）')
    return counts, degrees


def validate_step(text):
    """Parse the speed-curve peak entry (counts per 50 ms, 1…64)."""
    try:
        step = int(str(text).strip(), 10)
    except (TypeError, ValueError):
        raise ValueError('速度峰值必须是 1…64 的整数（计数/50ms）')
    if not STEP_MIN <= step <= POSITION_STEP_MAX:
        raise ValueError('速度峰值范围是 1…64（计数/50ms；8 ≈ 54°/s）')
    return step


def target_preview(text):
    """Count equivalent shown next to the entry; never raises for partial typing."""
    try:
        degrees = float(str(text).strip())
        if not math.isfinite(degrees):
            return '= — 计数'
        counts = degrees_to_counts(degrees)
    except (TypeError, ValueError):
        return '= — 计数'
    mark = '' if abs(counts) <= POSITION_LIMIT_COUNTS else '（超出 ±10 圈）'
    return f'= {counts:+d} 计数{mark}'


def estimate_move_seconds(distance_counts, max_speed):
    """Rough trapezoidal-profile travel time (seconds) for a goto command.

    Firmware 0x26101803+ starts at 1 count/tick, accelerates by +1 per 50 ms
    tick to the speed peak, cruises, then brakes by -1 per tick to 1 and stops
    exactly on target. Accel+decel ticks ≈ 2×Vmax; cruise ticks ≈
    max(0, (|Δ| − Vmax×(Vmax+1)) / Vmax). Vmax=0 means firmware default 8.
    """
    if type(max_speed) is not int:
        return None
    speed = STEP_DEFAULT if max_speed == 0 else max_speed
    if not STEP_MIN <= speed <= POSITION_STEP_MAX:
        return None
    distance = abs(int(distance_counts))
    if distance == 0:
        return 0.0
    cruise_ticks = max(0.0, (distance-speed*(speed+1))/speed)
    return (2*speed+cruise_ticks)*0.05


class PositionPanel:
    def __init__(self, parent, submit):
        self.submit = submit
        self.clock = time.monotonic
        self.target_var = tk.StringVar(value='0')
        self.step_var = tk.StringVar(value=str(STEP_DEFAULT))
        self.relative_var = tk.StringVar(value='90')
        self.precision_var = tk.BooleanVar(value=True)
        self.preview_var = tk.StringVar(value=target_preview(self.target_var.get()))
        self.estimate_var = tk.StringVar(value='预计 —（估算）')
        self.arrive_var = tk.StringVar(value='到位 —（等待遥测）')
        self.current_var = tk.StringVar(value='当前 —（等待遥测）')
        self.internal_var = tk.StringVar(value='内部目标 —（等待遥测）')
        self.arrived = False
        self.arrive_since = None
        self.delta_counts = None
        self.disconnected = False
        self.guide_var = tk.StringVar(
            value='速度曲线：加速→匀速→减速（梯形），无超调'
                  '（固件 0x26101803+；0x26101802 协议兼容但为匀速）')
        self.message = tk.StringVar(
            value='固件 0x26101803+ 支持梯形速度曲线定位，0x26101802 为匀速兼容；先连接并等待平衡遥测')
        self.last_seen = None
        self.snapshot = dict(fresh=False,pending=False,pending_operation=None)
        self.queued = False
        self.last_completion = None
        self.last_completion_record = None
        self.telemetry = {}
        self.buttons = {}
        self.frame = ttk.LabelFrame(parent, text='转臂位置控制（USB op4/op5；1063 计数 = 360°，相对平衡零点）', padding=8)
        self.frame.pack(fill='x', padx=12, pady=4)
        ttk.Label(self.frame, text='当前：').grid(row=0, column=0, sticky='w')
        ttk.Label(self.frame, textvariable=self.current_var, foreground='#0969da').grid(
            row=0, column=1, columnspan=2, sticky='w')
        ttk.Label(self.frame, text='内部目标：').grid(row=0, column=3, sticky='w')
        ttk.Label(self.frame, textvariable=self.internal_var, foreground='#8250df').grid(
            row=0, column=4, columnspan=3, sticky='w')
        self.arrive_label = tk.Label(self.frame, textvariable=self.arrive_var, foreground='#8b949e')
        self.arrive_label.grid(row=0, column=7, sticky='w', padx=(12, 0))
        ttk.Label(self.frame, text='目标角度(°)').grid(row=1, column=0, sticky='w', pady=(4, 0))
        target_entry = ttk.Entry(self.frame, textvariable=self.target_var, width=10)
        target_entry.grid(row=1, column=1, sticky='w', pady=(4, 0))
        target_entry.bind('<Return>', lambda event: self.send(4))
        ttk.Label(self.frame, textvariable=self.preview_var, foreground='#57606a').grid(
            row=1, column=2, sticky='w', pady=(4, 0))
        ttk.Label(self.frame, text='速度峰值').grid(row=1, column=3, sticky='e', pady=(4, 0))
        ttk.Entry(self.frame, textvariable=self.step_var, width=5).grid(
            row=1, column=4, sticky='w', pady=(4, 0))
        ttk.Label(self.frame, text='计数/50ms（1…64，曲线慢起-慢停；8≈54°/s）',
                  foreground='#57606a').grid(row=1, column=5, sticky='w', pady=(4, 0))
        ttk.Label(self.frame, textvariable=self.estimate_var, foreground='#57606a').grid(
            row=1, column=6, sticky='w', pady=(4, 0))
        command_row = ttk.Frame(self.frame)
        command_row.grid(row=2, column=0, columnspan=7, sticky='w', pady=(6, 0))
        specs = (('goto', '下发目标', lambda: self.send(4)),
                 ('zero', '回零(0°)', lambda: self.zero()),
                 ('hold', '保持当前位置', lambda: self.send(5)),
                 ('rel-90', '相对 -90°', lambda: self.relative(-90)),
                 ('rel+90', '相对 +90°', lambda: self.relative(90)),
                 ('rel-360', '相对 -360°', lambda: self.relative(-360)),
                 ('rel+360', '相对 +360°', lambda: self.relative(360)))
        for key, title, command in specs:
            button = ttk.Button(command_row, text=title, command=command, state='disabled')
            button.pack(side='left', padx=(0, 4))
            self.buttons[key] = button
        extra_row = ttk.Frame(self.frame)
        extra_row.grid(row=3, column=0, columnspan=7, sticky='w', pady=(4, 0))
        ttk.Label(extra_row, text='自定义相对角(°)').pack(side='left')
        ttk.Entry(extra_row, textvariable=self.relative_var, width=5).pack(side='left', padx=4)
        custom_button = ttk.Button(extra_row, text='相对移动', command=self.relative_custom, state='disabled')
        custom_button.pack(side='left', padx=(0, 4))
        self.buttons['rel-custom'] = custom_button
        ttk.Checkbutton(extra_row, text='精度模式：未到位禁止下发',
                        variable=self.precision_var, command=self.refresh_buttons).pack(side='left', padx=(12, 0))
        ttk.Label(self.frame, textvariable=self.message, foreground='#57606a').grid(
            row=4, column=0, columnspan=7, sticky='w', pady=(4, 0))
        ttk.Label(self.frame, textvariable=self.guide_var, foreground='#8b949e').grid(
            row=5, column=0, columnspan=7, sticky='w')
        ttk.Label(self.frame, foreground='#8b949e',
                  text='相对按钮锚定「当前目标」而非实际位置，避免连续点击累加误差；'
                       '「保持当前位置」冻结内部目标，不清除操作员输入').grid(
            row=6, column=0, columnspan=7, sticky='w')
        self.target_var.trace_add('write', lambda *args: self._update_preview())
        self.step_var.trace_add('write', lambda *args: self._update_estimate())

    def _update_preview(self):
        self.preview_var.set(target_preview(self.target_var.get()))
        self._update_estimate()

    def _update_estimate(self):
        try:
            target_counts, _ = validate_target_degrees(self.target_var.get())
            max_speed = validate_step(self.step_var.get())
        except (ValueError, TypeError):
            self.estimate_var.set('预计 —（估算）')
            return
        if 'arm' not in self.telemetry:
            self.estimate_var.set('预计 —（估算）')
            return
        seconds = estimate_move_seconds(target_counts-self.telemetry['arm'], max_speed)
        self.estimate_var.set('预计 —（估算）' if seconds is None else f'预计 ≈ {seconds:.1f} s（估算）')

    def _update_arrival(self):
        """Track |telemetry target - arm| <= 3 counts for >= 1 s (arrived state)."""
        if 'arm' not in self.telemetry or 'target' not in self.telemetry:
            self.delta_counts = None
            self.arrived = False
            self.arrive_since = None
            self.arrive_var.set('到位 —（未连接）' if self.disconnected else '到位 —（等待遥测）')
            self.arrive_label.configure(foreground='#8b949e')
            return
        delta = self.telemetry['target']-self.telemetry['arm']
        self.delta_counts = delta
        now = self.clock()
        if abs(delta) <= ARRIVE_TOL_COUNTS:
            if self.arrive_since is None:
                self.arrive_since = now
            self.arrived = (now-self.arrive_since) >= ARRIVE_SECONDS
            if self.arrived:
                text = f'到位（Δ={delta:+d} 计数，稳定 ≥{ARRIVE_SECONDS:g} s）'
                self.arrive_label.configure(foreground='#1a7f37')
            else:
                text = f'接近到位（Δ={delta:+d} 计数），稳定确认中…'
                self.arrive_label.configure(foreground='#9a6700')
        else:
            self.arrive_since = None
            self.arrived = False
            text = f'移动中（Δ={delta:+d} 计数）'
            self.arrive_label.configure(foreground='#cf222e')
        if not self.arrived and self.precision_var.get():
            text += ' · 未到位禁止下发'
        self.arrive_var.set(text)

    @property
    def fresh(self):
        return self.last_seen is not None and self.clock()-self.last_seen < FRESH_SECONDS

    @property
    def busy(self):
        return self.queued or bool(self.snapshot.get('pending'))

    def observe(self, frame):
        """Consume one 100 Hz telemetry frame; operator-edited target/step are untouched."""
        if not isinstance(frame, dict):
            return
        arm = frame.get('arm_relative_counts')
        target = frame.get('position_target_counts')
        if arm is None or target is None:
            return
        arm, target = int(arm), int(target)
        self.telemetry = dict(arm=arm, target=target, state=frame.get('state'),
                              firmware_id=frame.get('firmware_id'))
        self.disconnected = False
        self.last_seen = self.clock()
        state = frame.get('state')
        if state is None:
            state_text = ''
        else:
            state_text = f' · {STATE_NAMES.get(state, "状态"+str(state))}'
        self.current_var.set(f'{format_degrees(arm)}（{arm:+d} 计数）{state_text}')
        self.internal_var.set(f'{format_degrees(target)}（{target:+d} 计数）')
        self._update_estimate()
        self.refresh_buttons()

    def observe_link(self, snapshot):
        """Consume PIDLink.snapshot(): show 发送中 / 已生效 / 被拒 / 超时 messages."""
        previous = self.snapshot
        was_queued = self.queued
        self.snapshot = snapshot
        completed = snapshot.get('completed')
        new_completion = False
        if completed and completed.get('operation') in (4, 5):
            token = (completed.get('sequence'), completed.get('started'))
            if token != self.last_completion:
                self.last_completion = token
                self.last_completion_record = completed
                self.message.set(self.completion_text(completed))
                new_completion = True
        if snapshot.get('pending'):
            if snapshot.get('pending_operation') in (4, 5):
                self.message.set('发送中：位置命令等待板上确认…')
            elif was_queued:
                self.message.set('未发送：串口正有其它命令等待确认，请稍后再试')
        elif was_queued and not new_completion:
            # The reader rejected the queued command before writing it (link not
            # fresh, one-pending rule, serial write error…): show its reason.
            note = str(snapshot.get('message') or '')
            if note:
                self.message.set('未发送：'+note)
        elif previous.get('pending') and previous.get('pending_operation') in (4, 5) and not new_completion:
            note = str(snapshot.get('message', ''))
            if '超时' in note:
                self.message.set(note)
        self.queued = False
        self.refresh_buttons()

    def completion_text(self, completion):
        if completion.get('success'):
            if completion.get('operation') == 4:
                target = int(completion.get('target', 0))
                return f'已生效：转到 {format_degrees(target)}（{target:+d} 计数）'
            return '已生效：已冻结内部目标，转臂停止移动'
        return '被拒：'+position_result_text(completion.get('result'))

    def refresh_buttons(self):
        self._update_arrival()
        ready = self.fresh and not self.busy
        precision_block = self.precision_var.get() and not self.arrived
        for key, button in self.buttons.items():
            enabled = ready and not (precision_block and key != 'hold')
            button.configure(state='normal' if enabled else 'disabled')

    def disconnect(self):
        self.snapshot = dict(fresh=False,pending=False,pending_operation=None)
        self.last_seen = None
        self.telemetry = {}
        self.arrived = False
        self.arrive_since = None
        self.delta_counts = None
        self.disconnected = True
        self.queued = False
        self.current_var.set('当前 —（未连接）')
        self.internal_var.set('内部目标 —（未连接）')
        self.estimate_var.set('预计 —（估算）')
        self.arrive_var.set('到位 —（未连接）')
        self.arrive_label.configure(foreground='#8b949e')
        self.message.set('串口已断开；重新连接并等待平衡遥测')
        self.refresh_buttons()

    def send(self, operation, target_counts=None, step=None):
        """Queue op4 (word1 = speed-curve peak `step`) or op5 hold; returns queued."""
        if operation not in (4, 5):
            self.message.set('未发送：未知位置命令')
            return False
        try:
            if not self.fresh:
                raise ValueError('未收到有效实时遥测（或已超过1秒），不能下发位置命令')
            if self.busy:
                raise ValueError('上一条命令仍在等待确认')
            self._update_arrival()
            if operation == 4 and self.precision_var.get() and not self.arrived:
                delta = self.delta_counts
                detail = f'Δ={delta:+d} counts' if isinstance(delta, int) else 'Δ未知'
                raise ValueError(f'尚未到位（{detail}），请等待或取消精度模式')
            if operation == 4:
                if target_counts is None:
                    target_counts, degrees = validate_target_degrees(self.target_var.get())
                else:
                    if type(target_counts) is not int or not -POSITION_LIMIT_COUNTS <= target_counts <= POSITION_LIMIT_COUNTS:
                        raise ValueError('目标超出 ±10630 计数（±10 圈）')
                    degrees = counts_to_degrees(target_counts)
                    self.target_var.set(f'{degrees:.1f}')
                max_speed = validate_step(self.step_var.get() if step is None else step)
            else:
                target_counts, max_speed = 0, 0
            if not self.submit(operation, target_counts, max_speed):
                raise ValueError('请先开始记录并连接USB串口')
        except (ValueError, TypeError) as exc:
            self.message.set('未发送：'+str(exc))
            return False
        self.queued = True
        if operation == 4:
            self.message.set(f'命令已排队：转到 {format_degrees(target_counts)}，等待串口发送与板上确认')
        else:
            self.message.set('命令已排队：保持当前位置，等待串口发送与板上确认')
        self.refresh_buttons()
        return True

    def zero(self):
        return self.send(4, 0)

    def relative(self, degrees):
        """Move by degrees relative to the current TARGET (not the actual position).

        Anchoring on the commanded target avoids accumulating lag error when the
        operator clicks the relative button again before the arm has arrived.
        Falls back to the measured arm position if target telemetry is missing.
        """
        if not self.fresh or 'arm' not in self.telemetry:
            self.message.set('未发送：等收到有效遥测后再做相对移动')
            return False
        base = self.telemetry.get('target')
        if base is None:
            base = self.telemetry['arm']
        delta = degrees_to_counts(degrees)
        target = base+delta
        if not -POSITION_LIMIT_COUNTS <= target <= POSITION_LIMIT_COUNTS:
            self.message.set(f'未发送：相对 {degrees:+.0f}° 后目标 {format_degrees(target)} 超出 ±10 圈')
            return False
        if not self.send(4, target):
            return False
        self.message.set(f'相对 {degrees:+.0f}°：当前目标 {base:+d} {delta:+d} = {target:+d} 计数'
                         f'（{format_degrees(target)}），已排队')
        self.refresh_buttons()
        return True

    def relative_custom(self):
        """Custom relative move using the 自定义相对角 entry (degrees)."""
        try:
            degrees = float(self.relative_var.get().strip())
        except (TypeError, ValueError):
            self.message.set('未发送：自定义相对角度必须是数字（度）')
            return False
        if not math.isfinite(degrees):
            self.message.set('未发送：自定义相对角度必须是有限数字')
            return False
        return self.relative(degrees)
