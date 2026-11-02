"""Tk panel; ordinary telemetry never replaces an operator's edits."""
import time
import tkinter as tk
from tkinter import ttk
from runtime_pid import GAIN_FIELDS, validate_gains, BANK_NAMES


class PIDPanel:
    def __init__(self, parent, submit):
        self.submit = submit
        self.bank = tk.StringVar(value='自动')
        self.inputs = {bank:[tk.StringVar() for _ in GAIN_FIELDS] for bank in (0,1)}
        self.board = {}
        self.initialized = set()
        self.last_completion = None
        self.last_seen = None
        self.snapshot = dict(fresh=False,pending=False)
        self.queued = False
        self.message = tk.StringVar(value='新版串口固件支持实时调参；先连接并读取板上值')
        self.actual = [tk.StringVar(value='板上 —') for _ in GAIN_FIELDS]
        self.frame = ttk.LabelFrame(parent,text='实时 PID（Q12整数，实际系数 = 输入值 / 4096）',padding=8)
        self.frame.pack(fill='x',padx=12,pady=4)
        ttk.Label(self.frame,text='参数组').grid(row=0,column=0,padx=4)
        selector = ttk.Combobox(self.frame,textvariable=self.bank,values=BANK_NAMES,state='readonly',width=8)
        selector.grid(row=0,column=1,sticky='w')
        selector.bind('<<ComboboxSelected>>',lambda event:self.select_bank())
        self.buttons = {}
        for col,(op,title) in enumerate(((1,'整组下发'),(2,'读取板上值并填入'),(3,'恢复该组固件默认值')),2):
            b = ttk.Button(self.frame,text=title,command=lambda operation=op:self.send(operation),state='disabled')
            b.grid(row=0,column=col,padx=4,sticky='w')
            self.buttons[op] = b
        for col,label in enumerate(('P','I','D'),1):
            ttk.Label(self.frame,text=label).grid(row=1,column=col)
        self.entries = []
        for i in range(6):
            row,col=2+i//3,1+i%3
            cell=ttk.Frame(self.frame)
            cell.grid(row=row,column=col,padx=8,pady=3,sticky='w')
            entry=ttk.Entry(cell,textvariable=self.inputs[0][i],width=16)
            entry.pack(anchor='w')
            self.entries.append(entry)
            ttk.Label(cell,textvariable=self.actual[i],foreground='#0969da').pack(anchor='w')
        ttk.Label(self.frame,text='角度环').grid(row=2,column=0,padx=4)
        ttk.Label(self.frame,text='位置环').grid(row=3,column=0,padx=4)
        ttk.Label(self.frame,text='下发后在下个控制周期生效\n自动/手动分别保存，上电恢复默认值',
                  justify='left').grid(row=2,column=4,rowspan=2,padx=8,sticky='w')
        ttk.Label(self.frame,textvariable=self.message).grid(row=4,column=0,columnspan=5,sticky='w',pady=(4,0))

    @property
    def selected_bank(self):
        return BANK_NAMES.index(self.bank.get())

    def select_bank(self):
        bank=self.selected_bank
        for i,entry in enumerate(self.entries):
            entry.configure(textvariable=self.inputs[bank][i])
        self.update_actual()
        self.refresh_buttons()

    def update_actual(self):
        values=self.board.get(self.selected_bank)
        for i,var in enumerate(self.actual):
            var.set(f'板上 {values[i]} ({values[i]/4096:.5f})' if values else '板上 —（先读取）')

    def fill(self,bank,values):
        for var,value in zip(self.inputs[bank],values):
            var.set(str(value))
        self.initialized.add(bank)

    def observe(self,snapshot):
        self.snapshot=snapshot
        self.last_seen=time.monotonic() if snapshot['fresh'] else None
        self.board=snapshot.get('board',{})
        for bank,values in self.board.items():
            if bank not in self.initialized:
                self.fill(bank,values)
        completion=snapshot.get('completed')
        if completion:
            token=(completion['sequence'],completion['started'])
            if token!=self.last_completion:
                self.last_completion=token
                if completion['success'] and completion['operation'] in (2,3):
                    self.fill(completion['bank'],completion['readback'])
        self.queued=False
        self.message.set(snapshot['message'])
        self.update_actual()
        self.refresh_buttons()

    def refresh_buttons(self):
        fresh=self.snapshot['fresh'] and self.last_seen is not None and time.monotonic()-self.last_seen<1
        ready=fresh and not self.snapshot['pending'] and not self.queued
        for op,button in self.buttons.items():
            button.configure(state='normal' if ready and (op!=1 or self.selected_bank in self.initialized) else 'disabled')

    def disconnect(self):
        self.snapshot=dict(fresh=False,pending=False)
        self.last_seen=None
        self.queued=False
        self.message.set('串口已断开；重新连接并读取板上值')
        self.refresh_buttons()

    def send(self,operation):
        try:
            values=validate_gains(int(v.get().strip(),10) for v in self.inputs[self.selected_bank]) if operation==1 else (0,)*6
            if not self.submit(operation,self.selected_bank,values):
                raise ValueError('请先开始记录并连接USB串口')
        except (ValueError,TypeError) as exc:
            self.message.set('未发送：'+str(exc))
            return
        self.queued=True
        self.message.set('命令已排队，等待串口发送与板上确认')
        self.refresh_buttons()
