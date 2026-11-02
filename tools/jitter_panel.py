"""Tk panel for runtime nonlinear tuning (USB op6): deadbands, deadzone, D shift.

Archived experimental firmware0x26101804 applies these overrides, which lets the
operator scan small-angle jitter online. Telemetry does not read these values
back, so the panel shows the last acknowledged values from this session.
"""
import time
import tkinter as tk
from tkinter import ttk
from runtime_pid import (TUNING_FIELDS, TUNING_LIMITS, TUNING_NAMES, tuning_result_text,
                         validate_tuning)

FRESH_SECONDS = 1
TUNING_FIRMWARE_ID = 0x26101804
DEFAULT_ENTRIES = ('-1', '-1', '-1', '-1')


def parse_tuning_entries(texts):
    """Parse the four entry strings; raises ValueError with a field-specific message."""
    values = []
    for text, name, (low, high) in zip(texts, TUNING_NAMES, TUNING_LIMITS):
        try:
            value = int(str(text).strip(), 10)
        except (TypeError, ValueError):
            raise ValueError(f'{name}必须是 {low}…{high} 的整数')
        if not low <= value <= high:
            raise ValueError(f'{name}范围是 {low}…{high}')
        values.append(value)
    return tuple(values)


def describe_tuning(values):
    """Human-readable summary of the last applied op6 values (None = firmware default)."""
    if not values:
        return '固件默认（本次连接未下发覆盖）'
    if isinstance(values, dict):
        sequence = [values.get(name) for name in TUNING_FIELDS]
    else:
        sequence = list(values)
    parts = []
    for name, value in zip(TUNING_NAMES, sequence):
        parts.append(f'{name} {value}' + ('（默认）' if isinstance(value, int) and value < 0 else ''))
    return ' · '.join(parts)


class JitterPanel:
    def __init__(self, parent, submit):
        self.submit = submit
        self.supported = False
        self.inputs = [tk.StringVar(value=text) for text in DEFAULT_ENTRIES]
        self.info_var = tk.StringVar(value='当前生效（本机记录）：'+describe_tuning(None))
        self.message = tk.StringVar(value='抖动调参需固件 0x26101804+；先连接并等待平衡遥测')
        self.compat_var = tk.StringVar(value='等待遥测确认固件版本（op6 需 0x26101804+）')
        self.tip_var = tk.StringVar(
            value='每次只改一个参数并记录 60 s 稳定日志；-1 = 固件默认；'
                  '范围：角度/位置死区 -1…20、电机死区 -1…10、D 移位 -1…4')
        self.last_seen = None
        self.snapshot = dict(fresh=False,pending=False,pending_operation=None,last_tuning=None)
        self.queued = False
        self.last_completion = None
        self.buttons = {}
        self.frame = ttk.LabelFrame(
            parent, text='抖动调参（USB op6；运行时生效、任意状态；固件 0x26101804+）', padding=8)
        self.frame.pack(fill='x', padx=12, pady=4)
        labels = ('角度死区 counts', '位置死区 counts', '电机死区 PWM', 'D 滤波 shift 0-4')
        for index, label in enumerate(labels):
            column = index*2
            ttk.Label(self.frame, text=label).grid(
                row=0, column=column, sticky='e', padx=(0 if index == 0 else 10, 2))
            ttk.Entry(self.frame, textvariable=self.inputs[index], width=6).grid(
                row=0, column=column+1, sticky='w')
        ttk.Label(self.frame, textvariable=self.info_var, foreground='#0969da').grid(
            row=1, column=0, columnspan=8, sticky='w', pady=(4, 0))
        bar = ttk.Frame(self.frame)
        bar.grid(row=2, column=0, columnspan=8, sticky='w', pady=(4, 0))
        for key, title, command in (('apply', '应用参数', self.apply),
                                    ('restore', '恢复默认（全部 -1）', self.restore)):
            button = ttk.Button(bar, text=title, command=command, state='disabled')
            button.pack(side='left', padx=(0, 6))
            self.buttons[key] = button
        ttk.Label(self.frame, textvariable=self.message, foreground='#57606a').grid(
            row=3, column=0, columnspan=8, sticky='w', pady=(4, 0))
        ttk.Label(self.frame, textvariable=self.tip_var, foreground='#8b949e').grid(
            row=4, column=0, columnspan=8, sticky='w')
        ttk.Label(self.frame, textvariable=self.compat_var, foreground='#8b949e').grid(
            row=5, column=0, columnspan=8, sticky='w')

    @property
    def fresh(self):
        return self.last_seen is not None and time.monotonic()-self.last_seen < FRESH_SECONDS

    @property
    def busy(self):
        return self.queued or bool(self.snapshot.get('pending'))

    def observe(self, frame):
        """Consume one telemetry frame: refresh freshness and firmware-compatibility hint."""
        if not isinstance(frame, dict):
            return
        self.last_seen = time.monotonic()
        firmware_id = frame.get('firmware_id')
        if firmware_id is None:
            self.compat_var.set('等待固件标识（op6 需 0x26101804+）')
        elif firmware_id == TUNING_FIRMWARE_ID:
            self.compat_var.set(f'固件 0x{firmware_id:08X} 支持 op6 在线调参')
        else:
            self.compat_var.set(f'固件 0x{firmware_id:08X} 不支持归档op6实验；新版本请使用摩擦前馈页')
        self.supported = firmware_id == TUNING_FIRMWARE_ID and frame.get('format') == 'trace_v3_100hz'
        self.refresh_buttons()

    def observe_link(self, snapshot):
        """Consume PIDLink.snapshot(): ACK/reject/timeout messages and last tuning values."""
        previous = self.snapshot
        was_queued = self.queued
        self.snapshot = snapshot
        self.info_var.set('当前生效（本机记录）：'+describe_tuning(snapshot.get('last_tuning')))
        completed = snapshot.get('completed')
        new_completion = False
        if completed:
            token = (completed.get('sequence'), completed.get('started'))
            if token != self.last_completion:
                self.last_completion = token
                new_completion = True
                if completed.get('operation') == 6:
                    self.message.set(self.completion_text(completed))
                elif completed.get('operation') == 3 and completed.get('success'):
                    self.message.set('板上已恢复默认：PID 默认值 + 在线抖动覆盖已同时清除')
        if snapshot.get('pending'):
            if snapshot.get('pending_operation') == 6:
                self.message.set('发送中：抖动调参命令等待板上确认…')
            elif was_queued:
                self.message.set('未发送：串口正有其它命令等待确认，请稍后再试')
        elif was_queued and not new_completion:
            note = str(snapshot.get('message') or '')
            if note:
                self.message.set('未发送：'+note)
        elif previous.get('pending') and previous.get('pending_operation') == 6 and not new_completion:
            note = str(snapshot.get('message', ''))
            if '超时' in note:
                self.message.set(note)
        self.queued = False
        self.refresh_buttons()

    def completion_text(self, completion):
        if completion.get('success'):
            values = {name: completion.get(name) for name in TUNING_FIELDS}
            return '已生效：'+describe_tuning(values)
        return '被拒：'+tuning_result_text(completion.get('result'))

    def refresh_buttons(self):
        ready = self.supported and self.fresh and not self.busy
        for button in self.buttons.values():
            button.configure(state='normal' if ready else 'disabled')

    def disconnect(self):
        self.snapshot = dict(fresh=False,pending=False,pending_operation=None,last_tuning=None)
        self.last_seen = None
        self.supported = False
        self.queued = False
        self.info_var.set('当前生效（本机记录）：'+describe_tuning(None))
        self.compat_var.set('等待遥测确认固件版本（op6 需 0x26101804+）')
        self.message.set('串口已断开；重新连接并等待遥测')
        self.refresh_buttons()

    def _send(self, values):
        try:
            if not self.fresh:
                raise ValueError('未收到有效实时遥测（或已超过1秒），不能下发抖动参数')
            if not self.supported:
                raise ValueError('当前固件未启用归档op6实验')
            if self.busy:
                raise ValueError('上一条命令仍在等待确认')
            validate_tuning(*values)
            if not self.submit(*values):
                raise ValueError('请先开始记录并连接USB串口')
        except (ValueError, TypeError) as exc:
            self.message.set('未发送：'+str(exc))
            return False
        self.queued = True
        self.message.set('命令已排队：'+describe_tuning(values)+'，等待串口发送与板上确认')
        self.refresh_buttons()
        return True

    def apply(self):
        try:
            values = parse_tuning_entries(var.get() for var in self.inputs)
        except ValueError as exc:
            self.message.set('未发送：'+str(exc))
            return False
        return self._send(values)

    def restore(self):
        for var in self.inputs:
            var.set('-1')
        return self._send((-1, -1, -1, -1))
