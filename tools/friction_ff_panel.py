"""Small on-board A/B control for bounded friction compensation (USB op7)."""
import time
import tkinter as tk
from tkinter import ttk
from runtime_pid import FRICTION_FF_FIRMWARE_ID, validate_friction_ff, friction_ff_limit, friction_ff_profile, FRICTION_FF_FLAG_MASK


class FrictionFFPanel:
    def __init__(self, parent, submit):
        self.submit = submit
        self.input = tk.StringVar(value='2')
        self.actual = tk.StringVar(value='板上强度未知；先读取')
        self.message = tk.StringVar(value='固定PID，先记录关闭状态，再从2 PWM开始对照。')
        self.snapshot = {}
        self.last_seen = None
        self.supported = False
        self.limit = 8
        self.queued = False
        self.frame = ttk.LabelFrame(parent, text='电机摩擦补偿', padding=8)
        self.frame.pack(fill='x', padx=12, pady=4)
        self.limit_label = tk.StringVar(value='最大补偿 PWM（0关闭，1–8）')
        ttk.Label(self.frame, textvariable=self.limit_label).grid(row=0,column=0,sticky='w')
        self.strength = ttk.Spinbox(self.frame, from_=0, to=8, textvariable=self.input, width=6)
        self.strength.grid(row=0,column=1,padx=8)
        self.buttons = {}
        for col, (key, title, value) in enumerate((('apply','应用',None),('off','关闭补偿',0),('read','读取板上值',-1)), 2):
            button = ttk.Button(self.frame, text=title, command=lambda v=value:self.send(v), state='disabled')
            button.grid(row=0,column=col,padx=4)
            self.buttons[key] = button
        ttk.Label(self.frame, textvariable=self.actual, foreground='#0969da').grid(row=1,column=0,columnspan=5,sticky='w',pady=4)
        ttk.Label(self.frame, textvariable=self.message, wraplength=900).grid(row=2,column=0,columnspan=5,sticky='w')
        ttk.Label(self.frame, text='默认关闭；不会改变PID增益。掉电或恢复PID默认值后关闭。',
                  foreground='#57606a').grid(row=3,column=0,columnspan=5,sticky='w',pady=4)
        self.profile = tk.StringVar(value='等待固件版本')
        ttk.Label(self.frame, textvariable=self.profile, foreground='#57606a').grid(row=4,column=0,columnspan=5,sticky='w')

    def observe(self, frame):
        self.last_seen = time.monotonic()
        self.supported = (frame.get('firmware_id',0) >= FRICTION_FF_FIRMWARE_ID and
                          frame.get('format') == 'trace_v3_100hz')
        self.limit = friction_ff_limit(frame.get('firmware_id'))
        self.profile.set(friction_ff_profile(frame.get('firmware_id')) if self.supported else '等待支持摩擦补偿的固件')
        self.limit_label.set(f'最大补偿 PWM（0关闭，1–{self.limit}）')
        self.strength.configure(to=self.limit)
        if not self.supported:
            self.actual.set('当前固件不支持摩擦补偿，请使用0x26102201版本')
        elif frame.get('pid_command_bank') == 0x80 and frame.get('pid_command_result') in (2,7):
            ks = frame.get('pid_angle_kp')
            if type(ks) is int and 0 <= ks <= self.limit:
                self.actual.set(f'板上回读：{ks} PWM'+('（关闭）' if ks == 0 else '（启用）'))
        elif not (frame.get('flags',0) & FRICTION_FF_FLAG_MASK):
            self.actual.set('板上补偿已关闭（强度0）')
        else:
            self.actual.set('板上补偿已启用；点击读取确认强度')
        self.refresh_buttons()

    def observe_link(self, snapshot):
        previous_pending = self.snapshot.get('pending_operation') == 7
        self.snapshot = snapshot
        if snapshot.get('pending_operation') == 7 or self.queued or previous_pending:
            self.message.set(snapshot.get('message','等待板上确认'))
        self.queued = False
        self.refresh_buttons()

    def refresh_buttons(self):
        fresh = self.last_seen is not None and time.monotonic()-self.last_seen < 1
        ready = self.supported and fresh and not self.queued and not self.snapshot.get('pending')
        for b in self.buttons.values():
            b.configure(state='normal' if ready else 'disabled')

    def send(self, value=None):
        try:
            if not self.supported or self.last_seen is None or time.monotonic()-self.last_seen >= 1:
                raise ValueError('请先连接支持摩擦补偿的新固件并接收实时数据')
            if self.queued or self.snapshot.get('pending'):
                raise ValueError('上一条命令仍在等待确认')
            value = int(self.input.get().strip(),10) if value is None else value
            validate_friction_ff(value)
            if value > self.limit:
                raise ValueError('当前固件上限8 PWM；10/12需烧录0x26102202')
            if not self.submit(value):
                raise ValueError('请先开始记录并连接USB串口')
            self.queued = True
            self.message.set('已排队，等待板上确认')
        except (ValueError, TypeError) as exc:
            self.message.set('未发送：'+str(exc))
        self.refresh_buttons()

    def disconnect(self):
        self.supported = False
        self.last_seen = None
        self.queued = False
        self.snapshot = {}
        self.actual.set('板上强度未知；重新连接后读取')
        self.profile.set('等待固件版本')
        self.message.set('串口已断开')
        self.refresh_buttons()
