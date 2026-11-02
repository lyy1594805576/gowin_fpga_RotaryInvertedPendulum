"""100Hz recorder and USB runtime cascade PID tuning for jiangxiedaolibai.
Old v1/v2 and 9600 ASCII remain recording-only. Requires Tkinter and pyserial.
"""
import argparse
from collections import deque
import csv
import hashlib
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import queue
import re
import struct
import threading
import time
from runtime_pid import PIDLink, GAIN_FIELDS, READBACK_FIELDS, FRICTION_FF_FIRMWARE_ID, FRICTION_FF_FLAG_MASK

ROOT = Path(__file__).resolve().parents[1]
SHANGHAI = timezone(timedelta(hours=8))
HEADER = bytes((0xA6, 0x6A, 1, 65))
PAYLOAD = struct.Struct('<HIIBBHHhhHHHHiiiiihhii')
assert PAYLOAD.size == 60
FIELDS = ('sequence', 'firmware_id', 'fpga_ms', 'state', 'manual_mode', 'flags',
          'encoder_absolute', 'arm_relative_counts', 'position_target_counts',
          'theta_raw', 'theta_filtered', 'theta_min', 'theta_max',
          'angle_target_q8', 'position_output_q8', 'angle_error_q8',
          'd_angle_q8', 'd_position_q8', 'control_pwm', 'requested_pwm',
          'angle_integral_q8', 'position_integral_q8')
CONFIG_FIELDS = ('swing_pwm','swing_time_ticks','swing_return_ticks','angle_center','center_range',
                 'encoder_direction','angle_deadband','position_deadband',
                 'angle_kp','angle_ki','angle_kd','position_kp','position_ki','position_kd')
HEADER_V2 = bytes((0xA6,0x6A,2,93))
PAYLOAD_V2 = struct.Struct(PAYLOAD.format+'14h')
assert PAYLOAD_V2.size == 88
PID_FIELDS = ('pid_command_sequence','pid_active_sequence','pid_command_result','pid_command_bank',*READBACK_FIELDS)
HEADER_V3 = bytes((0xA6,0x6A,3,111))
PAYLOAD_V3 = struct.Struct(PAYLOAD_V2.format+'HHBB6h')
assert PAYLOAD_V3.size == 106
PROTOCOLS = {HEADER: (PAYLOAD,FIELDS,'trace_v1_100hz'),
             HEADER_V2: (PAYLOAD_V2,FIELDS+CONFIG_FIELDS,'trace_v2_100hz'),
             HEADER_V3: (PAYLOAD_V3,FIELDS+CONFIG_FIELDS+PID_FIELDS,'trace_v3_100hz')}
CSV_FIELDS = ('host_time', 'elapsed_s', 'format', 'epoch', *FIELDS, *CONFIG_FIELDS,
              *PID_FIELDS,
              'arm_session_counts', 'angle_target_counts', 'position_output_counts',
              'active_angle_ki', 'auto_integral_reduced',
              'friction_ff_enabled', 'friction_ff_ks', 'friction_ff_pwm')
STATES = {0: '停止', 1: '峰值判断', 21: '正向冲量', 22: '正向保持',
          23: '反向冲量', 24: '反向保持', 31: '反向冲量', 32: '反向保持',
          33: '正向冲量', 34: '正向保持', 4: '平衡'}
ASCII = re.compile(rb'P([+-]\d{5}) A(\d{5}) T(\d{4}) L(\d{4}) H(\d{4})')


def crc8(data):
    result = 0
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = ((result << 1) ^ (7 if result & 128 else 0)) & 255
    return result


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            header = next((h for h in PROTOCOLS if self.buffer.startswith(h)), None)
            if header is not None:
                length = header[3]
                if len(self.buffer) < length:
                    break
                packet = bytes(self.buffer[:length])
                if crc8(packet[:-1]) != packet[-1]:
                    self.bad_crc += 1
                    del self.buffer[0]
                    continue
                payload,fields,format_name = PROTOCOLS[header]
                frame = dict(zip(fields, payload.unpack(packet[4:-1])))
                # Reject impossible tags even when accidental data passes CRC.
                if frame['state'] not in STATES or frame['manual_mode'] not in (0, 1) or \
                        any(frame[k] > 4095 for k in ('theta_raw', 'theta_filtered', 'theta_min', 'theta_max')):
                    del self.buffer[0]
                    continue
                frame['format'] = format_name
                frames.append(frame)
                del self.buffer[:length]
            elif self.buffer[0] == ord('P'):
                newline = self.buffer.find(b'\n')
                candidates = [self.buffer.find(h,1) for h in PROTOCOLS]
                binary_start = min((p for p in candidates if p>=0),default=-1)
                if binary_start >= 0 and (newline < 0 or binary_start < newline):
                    self.discarded_bytes += binary_start
                    del self.buffer[:binary_start]
                    continue
                if newline < 0 and len(self.buffer) < 128:
                    break
                end = newline+1 if newline >= 0 else 1
                line = bytes(self.buffer[:end]).strip()
                match = ASCII.fullmatch(line)
                del self.buffer[:end]
                if match:
                    p, arm, theta, low, high = (int(v) for v in match.groups())
                    if 0 <= arm <= 65535 and all(0 <= v <= 4095 for v in (theta, low, high)):
                        frames.append({'format': 'legacy_ascii_2hz', 'encoder_absolute': arm,
                                       'theta_filtered': theta, 'theta_min': low, 'theta_max': high,
                                       'position_output_q8': p*256})
            elif any(h.startswith(bytes(self.buffer)) for h in PROTOCOLS):
                break
            else:
                del self.buffer[0]
                self.discarded_bytes += 1
        return frames


def local_manifest_status(manifest, root=ROOT):
    """Detect direct IDE rebuilds / edits that left build_verified.json stale."""
    result = {'local_manifest_mismatches': []}
    paths = [(root/name,wanted,name) for name,wanted in manifest['sha256'].items()]
    paths.append((Path(manifest['bitstream']),manifest['bitstream_sha256'],'bitstream'))
    for path,wanted,name in paths:
        if not path.resolve().is_relative_to(root.resolve()):
            result['local_manifest_mismatches'].append(name+' outside project')
            continue
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if name == 'bitstream': result['local_current_bitstream_sha256'] = actual
        if actual != wanted: result['local_manifest_mismatches'].append(name)
    result['local_build_manifest_matches_current_files'] = not result['local_manifest_mismatches']
    return result


class Session:
    def __init__(self, port, rate, directory=None):
        stamp = datetime.now(SHANGHAI).strftime('%Y%m%d_%H%M%S_%f')
        self.folder = Path(directory) if directory else ROOT/'logs'/stamp
        self.folder.mkdir(parents=True, exist_ok=False)
        self.raw = (self.folder/'serial_raw.bin').open('wb')
        self.samples = (self.folder/'samples.csv').open('w', newline='', encoding='utf-8-sig')
        self.csv = csv.DictWriter(self.samples, fieldnames=CSV_FIELDS)
        self.csv.writeheader()
        self.events = (self.folder/'events.csv').open('w', newline='', encoding='utf-8-sig')
        self.commands = (self.folder/'pid_commands.jsonl').open('w',encoding='utf-8')
        self.event_writer = csv.writer(self.events)
        self.event_writer.writerow(('host_time', 'elapsed_s', 'event', 'sequence', 'state'))
        self.start = time.monotonic()
        self.lock = threading.Lock()
        self.closed = False
        self.total = self.epoch = self.missing = self.resets = 0
        self.state_counts = {}
        self.last_seq = self.last_ms = self.last_arm = None
        self.arm_session = 0
        self.last_frame = {}
        self.meta = {'started_at': self.now(), 'port': port, 'baud_requested': rate,
                     'read_only': True, 'runtime_pid_supported': True,
                     'port_dtr': False, 'port_rts': False,
                     'local_settings_provenance': 'local_files_not_board_readback',
                     'raw_adc_meaning': '16-sample averaged ADC expanded to 12 bits',
                     'pwm_meaning': 'controller/requested duty; not measured motor voltage or speed'}
        manifest_path = ROOT/'build_verified.json'
        if manifest_path.exists():
            try:
                manifest = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
                self.meta['local_build_manifest'] = manifest
                self.meta.update(local_manifest_status(manifest))
            except (ValueError, OSError, KeyError, TypeError) as exc:
                self.meta['local_manifest_error'] = str(exc)
                self.meta['local_build_manifest_matches_current_files'] = False
        self.save_meta()

    @staticmethod
    def now():
        return datetime.now(SHANGHAI).isoformat(timespec='milliseconds')

    def save_meta(self):
        (self.folder/'session.json').write_text(json.dumps(self.meta, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

    def append_raw(self, data):
        self.raw.write(data)

    def append(self, frame):
        elapsed = time.monotonic()-self.start
        row = {'host_time': self.now(), 'elapsed_s': round(elapsed, 6), **frame}
        seq, ms = row.get('sequence'), row.get('fpga_ms')
        reset = self.last_ms is not None and ms is not None and \
            ((ms-self.last_ms) & 0xffffffff) > 0x80000000
        if reset:
            self.epoch += 1
            self.resets += 1
            self.last_seq = self.last_arm = None
            self.arm_session = 0
        if seq is not None and self.last_seq is not None:
            delta = (seq-self.last_seq) & 0xffff
            # A backward sequence can indicate reset without a sampled ms decrease.
            if delta > 0x8000:
                self.epoch += 1
                self.resets += 1
                self.last_arm = None
                self.arm_session = 0
            elif delta > 1:
                self.missing += delta-1
        arm = row['encoder_absolute']
        if self.last_arm is not None:
            self.arm_session += ((arm-self.last_arm+32768) & 65535)-32768
        self.last_arm, self.last_seq, self.last_ms = arm, seq, ms
        row['epoch'] = self.epoch
        row['arm_session_counts'] = self.arm_session
        # 0x26100504 preserves the 65-byte layout and adds active KI to flags.
        # Earlier firmware's high flag bits are unused, not a gain readback.
        if 'angle_ki' in row:
            row['active_angle_ki'] = row['angle_ki']
            row['auto_integral_reduced'] = int(bool(row['flags'] & 32))
        elif row.get('firmware_id') == 0x26100504:
            flags = row['flags']
            row['active_angle_ki'] = flags >> 8 if flags & 128 else ''
            row['auto_integral_reduced'] = int(bool(flags & 32))
        for source, target in (('angle_target_q8', 'angle_target_counts'),
                               ('position_output_q8', 'position_output_counts')):
            if source in row:
                row[target] = row[source]/256
        if row.get('firmware_id',0) >= FRICTION_FF_FIRMWARE_ID:
            row['friction_ff_enabled'] = int(bool(row['flags'] & FRICTION_FF_FLAG_MASK))
            if row.get('pid_command_bank') == 0x80 and row.get('pid_command_result') in (2,7):
                row['friction_ff_ks'] = row['pid_angle_kp']
            elif not row['friction_ff_enabled']:
                row['friction_ff_ks'] = 0
            # Suppressed PID commands inside the old deadzone are not FF.
            row['friction_ff_pwm'] = row['requested_pwm']-row['control_pwm'] if row['requested_pwm'] else 0
        with self.lock:
            state = row.get('state')
            if state is not None:
                self.state_counts[str(state)] = self.state_counts.get(str(state), 0)+1
                if state != self.last_frame.get('state') or row['epoch'] != self.last_frame.get('epoch'):
                    self.event_writer.writerow((row['host_time'], row['elapsed_s'],
                        f"状态→{STATES[state]}({state}) T={row['theta_filtered']} PWM={row.get('requested_pwm', '')}",
                        seq, state))
            self.csv.writerow({k: row.get(k, '') for k in CSV_FIELDS})
            self.total += 1
            self.last_frame = row
        if 'firmware_id' in row:
            self.meta['observed_firmware_id'] = f"0x{row['firmware_id']:08X}"
        self.meta['observed_format'] = row['format']
        return row

    def mark(self, text):
        with self.lock:
            if not self.closed:
                self.event_writer.writerow((self.now(), round(time.monotonic()-self.start, 6), text,
                                            self.last_frame.get('sequence', ''), self.last_frame.get('state', '')))
                self.events.flush()

    def command_event(self, event):
        with self.lock:
            if not self.closed:
                self.commands.write(json.dumps(dict(host_time=self.now(),elapsed_s=round(time.monotonic()-self.start,6),
                                                    **event),ensure_ascii=False)+'\n')
                self.commands.flush()
                if event['event']=='send':
                    self.meta['read_only'] = False

    def flush(self):
        self.raw.flush()
        self.samples.flush()

    def close(self, decoder, reason):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.flush()
            self.raw.close()
            self.samples.close()
            self.events.close()
            self.commands.close()
            self.meta.update(ended_at=self.now(), stop_reason=reason, frames=self.total,
                             missing_sequences=self.missing, resets=self.resets,
                             bad_crc=decoder.bad_crc, discarded_bytes=decoder.discarded_bytes,
                             state_frame_counts=self.state_counts)
            self.save_meta()


class Reader(threading.Thread):
    def __init__(self, port, baud, notifications):
        super().__init__(daemon=True)
        self.port_name, self.baud, self.notifications = port, baud, notifications
        self.stop_event = threading.Event()
        self.session = None
        self.command_queue = queue.Queue()

    def post(self, kind, value):
        try:
            self.notifications.put_nowait((kind, value))
        except queue.Full:
            pass  # only live display drops; worker has already saved CSV/raw

    def run(self):
        decoder = Decoder()
        connection = None
        link = None
        reason = '用户停止'
        try:
            import serial
            rates = (115200, 9600) if self.baud == '自动检测' else (int(self.baud),)
            self.session = Session(self.port_name, self.baud)
            link = PIDLink(self.session.command_event)
            self.post('folder', str(self.session.folder))
            detected = False
            for rate in rates:
                if self.stop_event.is_set():
                    break
                decoder = Decoder()
                connection = serial.Serial()
                connection.port = self.port_name
                connection.baudrate = rate
                connection.timeout = 0
                connection.write_timeout = .3
                connection.dtr = False
                connection.rts = False
                connection.open()
                self.post('status', f'已打开 {self.port_name} / {rate}，等候数据…')
                self.session.mark(f'串口打开 {rate} baud')
                deadline = time.monotonic()+2.5
                flush_time = time.monotonic()
                last_received = time.monotonic()
                while not self.stop_event.is_set():
                    data = connection.read(min(connection.in_waiting, 65536) or 1)
                    if data:
                        self.session.append_raw(data)
                        for frame in decoder.feed(data):
                            link.observe(frame)
                            detected = True
                            last_received = time.monotonic()
                            self.session.meta['baud_detected'] = rate
                            self.post('frame', self.session.append(frame))
                    while not self.command_queue.empty() and not self.stop_event.is_set():
                        operation, bank, gains = self.command_queue.get_nowait()
                        try:
                            if operation == 4:
                                link.send_position(connection, gains[0], gains[1])
                            elif operation == 5:
                                link.send_position_hold(connection)
                            elif operation == 6:
                                link.send_tuning(connection, gains[0], gains[1], gains[2], gains[3])
                            elif operation == 7:
                                link.send_friction_ff(connection, gains[0])
                            else:
                                link.send(connection,operation,bank,gains)
                        except (ValueError, OSError) as exc:
                            link.message = str(exc)
                    link.check_timeout()
                    self.post('pid', link.snapshot())
                    now = time.monotonic()
                    if now-flush_time > .5:
                        self.session.flush()
                        flush_time = now
                        self.post('stats', (self.session.total, self.session.missing, decoder.bad_crc))
                    if not detected and now > deadline and self.baud == '自动检测':
                        break
                    if now-last_received > 2:
                        self.post('status', '暂未收到有效帧：检查USB串口、位流及波特率；记录仍继续')
                        last_received = now
                    self.stop_event.wait(.01)
                connection.close()
                connection = None
                if detected or self.stop_event.is_set():
                    break
            if not detected and not self.stop_event.is_set():
                reason = '两个波特率均未检测到本工程数据'
                self.post('error', reason+'。可选择固定波特率持续等候。')
        except Exception as exc:
            reason = str(exc)
            self.post('error', reason)
        finally:
            if link is not None and link.pending:
                link.log(dict(event='unconfirmed',reason='串口关闭：'+reason,**link.pending))
                link.pending = None
            if connection is not None:
                try:
                    connection.close()
                except Exception as exc:
                    reason += '; close: '+str(exc)
            if self.session is not None:
                try:
                    self.session.close(decoder, reason)
                except Exception as exc:
                    reason += '; save: '+str(exc)
                    self.post('error', reason)
            self.post('stopped', reason)


def create_window(root):
    import tkinter as tk
    from tkinter import ttk, messagebox
    from serial.tools import list_ports
    from pid_panel import PIDPanel
    from position_panel import PositionPanel
    from jitter_panel import JitterPanel
    from friction_ff_panel import FrictionFFPanel
    root.title('江协倒立摆 · 实时PID调参与起摆记录')
    root.geometry('1080x940')
    root.minsize(920, 820)
    style = ttk.Style(root)
    if 'vista' in style.theme_names():
        style.theme_use('vista')
    controls = ttk.Frame(root, padding=12)
    controls.pack(fill='x')
    port = tk.StringVar()
    baud = tk.StringVar(value='自动检测')
    span = tk.StringVar(value='10')
    ttk.Label(controls, text='USB串口').grid(row=0, column=0, padx=4)
    ports_box = ttk.Combobox(controls, textvariable=port, width=16)
    ports_box.grid(row=0, column=1, padx=4)
    def refresh():
        ports = list(list_ports.comports())
        ports_box['values'] = [p.device for p in ports]
        if ports and not port.get():
            preferred = next((p for p in ports if any(s in p.description.lower() for s in ('usb', 'ch340', 'cp210'))), ports[0])
            port.set(preferred.device)
    ttk.Button(controls, text='刷新', command=refresh).grid(row=0, column=2, padx=4)
    ttk.Combobox(controls, textvariable=baud, values=('自动检测', '115200', '9600'),
                 state='readonly', width=11).grid(row=0, column=3, padx=4)
    button = ttk.Button(controls, text='开始记录')
    button.grid(row=0, column=4, padx=8)
    ttk.Label(controls, text='显示秒数').grid(row=0, column=5, padx=4)
    ttk.Combobox(controls, textvariable=span, values=('5', '10', '30', '60'),
                 state='readonly', width=5).grid(row=0, column=6)
    status = tk.StringVar(value='先开始记录，再在板上按 SW2、SW4 自动起摆；PID可在运行时调节。')
    stats = tk.StringVar(value='新记录固件：115200 / 100Hz；旧P/A/T固件：9600 / 2Hz')
    values = tk.StringVar(value='状态 —    角度 —    编码器 —    电机指令 —')
    folder_text = tk.StringVar(value=f'记录目录：{ROOT / "logs"}')
    for var in (status, stats, values):
        ttk.Label(root, textvariable=var, padding=(16, 4)).pack(anchor='w')
    def submit_pid(operation,bank,gains):
        current=context['reader']
        if not current or not current.is_alive() or current.stop_event.is_set():
            return False
        current.command_queue.put((operation,bank,gains))
        return True
    def submit_position(operation,target,step):
        current=context['reader']
        if not current or not current.is_alive() or current.stop_event.is_set():
            return False
        current.command_queue.put((operation,0,(target,step,0,0,0,0)))
        return True
    def submit_tuning(angle_deadband,position_deadband,motor_deadzone,d_filter_shift):
        current=context['reader']
        if not current or not current.is_alive() or current.stop_event.is_set():
            return False
        current.command_queue.put((6,0,(angle_deadband,position_deadband,
                                        motor_deadzone,d_filter_shift,0,0)))
        return True
    def submit_friction_ff(ks):
        return submit_pid(7,0,(ks,0,0,0,0,0))
    # Control areas share one tabbed row so the fixed 1080x940 window keeps
    # a large waveform canvas; each panel keeps its own submit callback.
    tabs = ttk.Notebook(root)
    tabs.pack(fill='x', padx=12, pady=(0, 2))
    pid_tab = ttk.Frame(tabs)
    position_tab = ttk.Frame(tabs)
    jitter_tab = ttk.Frame(tabs)
    friction_tab = ttk.Frame(tabs)
    tabs.add(pid_tab, text='PID 增益')
    tabs.add(position_tab, text='转臂位置')
    tabs.add(jitter_tab, text='抖动调参')
    tabs.add(friction_tab, text='摩擦前馈')
    pid_panel=PIDPanel(pid_tab,submit_pid)
    position_panel=PositionPanel(position_tab,submit_position)
    jitter_panel=JitterPanel(jitter_tab,submit_tuning)
    friction_panel=FrictionFFPanel(friction_tab,submit_friction_ff)
    # The canvas expands to fill the window; a modest requested height leaves
    # the tabs and canvas inside the fixed 1080x940 window.
    canvas = tk.Canvas(root, background='#f6f8fa', highlightthickness=0, height=160)
    canvas.pack(fill='both', expand=True, padx=12, pady=8)
    notes = ttk.Frame(root, padding=8)
    notes.pack(fill='x')
    note = tk.StringVar(value='起摆第1次')
    ttk.Entry(notes, textvariable=note, width=24).pack(side='left', padx=4)
    marker = ttk.Button(notes, text='标记现象')
    marker.pack(side='left', padx=4)
    open_button = ttk.Button(notes, text='打开日志目录')
    open_button.pack(side='left', padx=4)
    ttk.Label(root, textvariable=folder_text, padding=(16, 4)).pack(anchor='w')
    ttk.Label(root, text='PWM显示的是控制指令。旧文本帧不含状态/PWM，无法判断短暂切换；建议烧录本轮记录固件。',
              padding=(16, 4)).pack(anchor='w')
    notifications = queue.Queue(maxsize=10000)
    samples = deque(maxlen=6500)
    context = {'reader': None, 'folder': None, 'latest': None, 'closed': False}
    context['notifications'] = notifications
    context['canvas'] = canvas
    context['pid_panel'] = pid_panel
    context['position_panel'] = position_panel
    context['jitter_panel'] = jitter_panel
    context['friction_panel'] = friction_panel

    def toggle():
        current = context['reader']
        if current and current.is_alive():
            current.stop_event.set()
            button.configure(state='disabled')
            status.set('正在保存并关闭串口…')
        else:
            if not port.get().strip():
                messagebox.showinfo('串口', '请选择或填写USB串口，例如COM3。', parent=root)
                return
            samples.clear()
            context['latest'] = None
            pid_panel.disconnect()
            position_panel.disconnect()
            jitter_panel.disconnect()
            friction_panel.disconnect()
            current = Reader(port.get().strip(), baud.get(), notifications)
            context['reader'] = current
            current.start()
            button.configure(text='停止记录')
    button.configure(command=toggle)
    def mark():
        current = context['reader']
        if current and current.session and not current.session.closed:
            current.session.mark(note.get().strip() or '人工标记')
            status.set('已写入标记：'+(note.get().strip() or '人工标记'))
    marker.configure(command=mark)
    def open_folder():
        path = Path(context['folder']) if context['folder'] else ROOT/'logs'
        path.mkdir(parents=True, exist_ok=True)
        os.startfile(str(path))
    open_button.configure(command=open_folder)

    def draw():
        canvas.delete('all')
        width, height = max(canvas.winfo_width(), 600), max(canvas.winfo_height(), 300)
        duration = float(span.get())
        latest = samples[-1]['elapsed_s'] if samples else duration
        left_time, right_time = max(0, latest-duration), max(duration, latest)
        visible = [r for r in samples if r['elapsed_s'] >= left_time]
        panels = (
            ('摆杆角度（ADC扩展计数）', (('theta_raw', '#8b949e', 'ADC均值'),
              ('theta_filtered', '#0969da', '滤后'), ('angle_target_counts', '#cf222e', '目标'))),
            ('转臂位移（自记录起点，编码器计数）', (('arm_session_counts', '#8250df', '转臂'),)),
            ('电机PWM指令（−255…+255）', (('control_pwm', '#cf222e', '控制器'),
              ('requested_pwm', '#1a7f37', '死区后请求'))))
        panel_height = height/3
        for panel_index, (title, series) in enumerate(panels):
            y0 = panel_index*panel_height
            x1, x2, y1, y2 = 75, width-18, y0+30, y0+panel_height-26
            canvas.create_text(12, y0+12, text=title, anchor='w', fill='#24292f')
            legend = '  '.join(label for _, _, label in series)
            canvas.create_text(width-18, y0+12, text=legend, anchor='e', fill='#57606a')
            data_values = [r[k] for r in visible for k, _, _ in series if isinstance(r.get(k), (int, float))]
            if panel_index == 2:
                low, high = -255, 255
            elif data_values:
                low, high = min(data_values), max(data_values)
                padding = max(2, (high-low)*.1)
                low, high = low-padding, high+padding
            else:
                low, high = (3000, 3035) if panel_index == 0 else (-10, 10)
            for fraction in (0, .5, 1):
                y = y2-(y2-y1)*fraction
                canvas.create_line(x1, y, x2, y, fill='#d8dee4')
                canvas.create_text(x1-7, y, text=f'{low+(high-low)*fraction:.1f}', anchor='e', fill='#57606a')
            canvas.create_text(x1, y2+15, text=f'{left_time:.1f}s', anchor='w', fill='#57606a')
            canvas.create_text(x2, y2+15, text=f'{right_time:.1f}s', anchor='e', fill='#57606a')
            for key, color, _ in series:
                points = []
                for row in visible:
                    if isinstance(row.get(key), (int, float)):
                        points.extend((x1+(row['elapsed_s']-left_time)/(right_time-left_time)*(x2-x1),
                                       y2-(row[key]-low)/(high-low)*(y2-y1)))
                if len(points) >= 4:
                    canvas.create_line(*points, fill=color, width=1.5)
            # Shade sampled swing/balance transitions across all plots.
            previous_state = None
            for row in visible:
                current_state = row.get('state')
                if current_state is not None and previous_state is not None and current_state != previous_state:
                    x = x1+(row['elapsed_s']-left_time)/(right_time-left_time)*(x2-x1)
                    canvas.create_line(x, y1, x, y2, fill='#aeb8c2', dash=(2, 4))
                previous_state = current_state

    def poll():
        if context['closed']:
            return
        for _ in range(2000):
            try:
                kind, value = notifications.get_nowait()
            except queue.Empty:
                break
            if kind == 'frame':
                samples.append(value)
                context['latest'] = value
                position_panel.observe(value)
                jitter_panel.observe(value)
                friction_panel.observe(value)
                name = STATES.get(value.get('state'), '旧帧：状态未知')
                mode = ('手动' if value.get('manual_mode') else '自动') if 'manual_mode' in value else '未知'
                pwm = value.get('requested_pwm', '未知')
                values.set(f"{mode} / {name}    T={value['theta_filtered']}  L/H={value['theta_min']}/{value['theta_max']}"
                           f"    A={value['encoder_absolute']}    PWM={pwm}")
                identity = f"0x{value['firmware_id']:08X}" if 'firmware_id' in value else '旧文本帧'
                gain = value.get('active_angle_ki')
                gain_text = f' · 当前角度KI={gain}' if gain is not None and gain != '' else ''
                swing_text = f" · 起摆PWM={value['swing_pwm']}" if 'swing_pwm' in value else ''
                status.set('正在记录 · 板上标识 '+identity+gain_text+swing_text)
            elif kind == 'stats':
                stats.set(f'已保存 {value[0]} 帧    序号缺口 {value[1]}    CRC失败 {value[2]}')
            elif kind == 'pid':
                pid_panel.observe(value)
                position_panel.observe_link(value)
                jitter_panel.observe_link(value)
                friction_panel.observe_link(value)
            elif kind == 'folder':
                context['folder'] = value
                folder_text.set('记录目录：'+value)
            elif kind == 'status':
                status.set(value)
            elif kind == 'error':
                status.set('记录异常：'+value)
            elif kind == 'stopped':
                pid_panel.disconnect()
                position_panel.disconnect()
                jitter_panel.disconnect()
                friction_panel.disconnect()
                button.configure(text='开始记录', state='normal')
                status.set('记录已结束：'+value)
        pid_panel.refresh_buttons()
        position_panel.refresh_buttons()
        jitter_panel.refresh_buttons()
        friction_panel.refresh_buttons()
        draw()
        root.after(100, poll)

    def close():
        current = context['reader']
        if current and current.is_alive():
            current.stop_event.set()
            current.join(timeout=2)
            if current.is_alive():
                root.after(100, close)
                return
        context['closed'] = True
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', close)
    refresh()
    root.after(100, poll)
    return context


def self_test(fixture):
    import tempfile
    assert crc8(b'123456789') == 0xf4  # standard CRC8/ATM check value
    values = (65535, 0x26100503, 123456, 21, 0, 17, 65535, -3, 260,
              3018, 3017, 3000, 3030, 3017*256, -300*256, -256,
              -128, 512, -180, -180, -25600, 1280000)
    packet = HEADER+PAYLOAD.pack(*values)
    packet += bytes((crc8(packet),))
    decoder = Decoder()
    result = []
    for byte in b'junk'+packet:
        result.extend(decoder.feed(bytes((byte,))))
    assert len(result) == 1 and tuple(result[0][k] for k in FIELDS) == values
    corrupt = bytearray(packet)
    corrupt[30] ^= 1
    decoder = Decoder()
    assert len(decoder.feed(corrupt+packet)) == 1 and decoder.bad_crc == 1
    assert len(Decoder().feed(b'Pbroken'+packet)) == 1
    decoder = Decoder()
    text_frames = decoder.feed(b'junkP-00300 A65535 T3017 L3000 H3030\r\n')
    assert len(text_frames) == 1 and text_frames[0]['position_output_q8'] == -76800
    config_values = (180,65,95,3017,500,-1,0,2,1550,12,31600,940,16,9950)
    v2_values = list(values)
    v2_values[1],v2_values[5] = 0x26100505,3217
    packet_v2 = HEADER_V2+PAYLOAD_V2.pack(*v2_values,*config_values)
    packet_v2 += bytes((crc8(packet_v2),))
    decoder_v2 = Decoder()
    result_v2 = []
    for byte in b'Pbroken'+packet_v2:
        result_v2.extend(decoder_v2.feed(bytes((byte,))))
    assert len(result_v2)==1 and result_v2[0]['format']=='trace_v2_100hz'
    assert tuple(result_v2[0][k] for k in CONFIG_FIELDS)==config_values
    mixed = Decoder().feed(packet+packet_v2+packet)
    assert [f['format'] for f in mixed]==['trace_v1_100hz','trace_v2_100hz','trace_v1_100hz']
    broken_v2 = bytearray(packet_v2);broken_v2[80] ^= 1
    corrupt_decoder = Decoder()
    assert len(corrupt_decoder.feed(broken_v2+packet_v2))==1 and corrupt_decoder.bad_crc==1
    with tempfile.TemporaryDirectory(dir=ROOT/'sim') as temp:
        temp_root = Path(temp)
        (temp_root/'source.bin').write_bytes(b'original')
        (temp_root/'bitstream.bin').write_bytes(b'original')
        h = hashlib.sha256(b'original').hexdigest()
        fake_manifest = {'sha256':{'source.bin':h},'bitstream':str(temp_root/'bitstream.bin'),'bitstream_sha256':h}
        assert local_manifest_status(fake_manifest,temp_root)['local_build_manifest_matches_current_files']
        (temp_root/'bitstream.bin').write_bytes(b'IDE rebuilt')
        stale = local_manifest_status(fake_manifest,temp_root)
        assert not stale['local_build_manifest_matches_current_files'] and stale['local_manifest_mismatches']==['bitstream']
        session = Session('TEST', 115200, Path(temp)/'session')
        frame = dict(result[0])
        session.append_raw(packet)
        first = session.append(frame)
        frame.update(sequence=0, fpga_ms=123466, encoder_absolute=1)
        wrapped = session.append(frame)
        assert wrapped['arm_session_counts'] == 2 and session.missing == 0
        frame.update(sequence=3, fpga_ms=123496)
        session.append(frame)
        assert session.missing == 2
        frame.update(sequence=0, fpga_ms=0, encoder_absolute=10000)
        reset = session.append(frame)
        assert reset['epoch'] == 1 and reset['arm_session_counts'] == 0
        session.mark('起摆标记')
        session.close(decoder, 'self test')
        with (session.folder/'samples.csv').open(encoding='utf-8-sig', newline='') as handle:
            saved = list(csv.DictReader(handle))
        assert len(saved) == 4 and saved[0]['state'] == '21'
        assert saved[0]['active_angle_ki'] == '' and saved[0]['auto_integral_reduced'] == ''
        meta = json.loads((session.folder/'session.json').read_text(encoding='utf-8'))
        assert meta['read_only'] and meta['resets'] == 1 and meta['missing_sequences'] == 2
        trial = Session('TEST', 115200, Path(temp)/'trial')
        trial_values = list(values)
        trial_values[1] = 0x26100504
        for seq, ms, gain, reduced in ((65535,123456,12,False),(0,123466,6,True)):
            trial_values[0],trial_values[2] = seq,ms
            trial_values[5] = (gain<<8)|128|17|(96 if reduced else 0)
            current = HEADER+PAYLOAD.pack(*trial_values)
            current += bytes((crc8(current),))
            trial.append_raw(current)
            current_row = trial.append(Decoder().feed(current)[0])
            assert current_row['active_angle_ki'] == gain
            assert current_row['auto_integral_reduced'] == int(reduced)
        trial.close(Decoder(), 'self test')
        with (trial.folder/'samples.csv').open(encoding='utf-8-sig',newline='') as handle:
            trial_saved = list(csv.DictReader(handle))
        assert [r['active_angle_ki'] for r in trial_saved] == ['12','6']
        assert [r['auto_integral_reduced'] for r in trial_saved] == ['0','1']
        config_session = Session('TEST',115200,Path(temp)/'config')
        config_session.append_raw(packet_v2)
        config_row = config_session.append(result_v2[0])
        config_session.close(Decoder(),'self test')
        assert config_row['active_angle_ki']==12 and config_row['swing_pwm']==180
        with (config_session.folder/'samples.csv').open(encoding='utf-8-sig',newline='') as handle:
            config_saved = next(csv.DictReader(handle))
        assert tuple(int(config_saved[k]) for k in CONFIG_FIELDS)==config_values
        # Exercise the actual reader thread against a virtual serial endpoint.
        # Any attempted board write fails; fragments must still be saved intact.
        from unittest.mock import patch
        endpoint = {}
        notifications = queue.Queue()
        reader = Reader('REPLAY', '115200', notifications)
        class ReplaySerial:
            def __init__(self):
                self.chunks = deque((packet_v2[:7], packet_v2[7:35], packet_v2[35:]))
            @property
            def in_waiting(self):
                return len(self.chunks[0]) if self.chunks else 0
            def open(self):
                endpoint.update(baud=self.baudrate, dtr=self.dtr, rts=self.rts)
            def read(self, size):
                if self.chunks:
                    return self.chunks.popleft()
                reader.stop_event.set()
                return b''
            def write(self, data):
                raise AssertionError('read-only logger attempted a serial write')
            def close(self):
                endpoint['closed'] = True
        original_session = Session
        with patch('serial.Serial', ReplaySerial), patch(__name__+'.Session',
                side_effect=lambda p, b: original_session(p, b, Path(temp)/'reader')):
            reader.start()
            reader.join(timeout=3)
        assert not reader.is_alive() and reader.session.total == 1
        assert endpoint == {'baud': 115200, 'dtr': False, 'rts': False, 'closed': True}
        assert (reader.session.folder/'serial_raw.bin').read_bytes() == packet_v2
    if fixture:
        actual = bytes(int(token, 16) for token in Path(fixture).read_text().split())
        decoded = Decoder().feed(actual)
        assert len(decoded) >= 2
        current_expected = list(values)
        current_expected[1] = 0x26100801
        current_expected[5] = 3217
        assert tuple(decoded[0][k] for k in FIELDS) == tuple(current_expected)
        assert tuple(decoded[0][k] for k in CONFIG_FIELDS) == config_values
        assert tuple(decoded[0][k] for k in PID_FIELDS) == (17,16,7,1,1750,4,16500,870,4,8000)
        assert decoded[1]['sequence'] == 0 and decoded[1]['fpga_ms'] > decoded[0]['fpga_ms']
        print('PASS real RTL UART bytes, field order, signed values, CRC and next frame')
    print('PASS decoder fragmentation/resync, ASCII compatibility, rollover/reset, CSV/raw/events/metadata')
    print('PASS actual-KI readback 12/6, reduced flag and legacy unknown-gain compatibility')
    print('PASS v2 configuration signed fields, old/new protocol resync, CSV and stale build-manifest detection')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--fixture')
    parser.add_argument('--ui-smoke-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test(args.fixture)
    else:
        try:
            import tkinter as tk
            root = tk.Tk()
            if args.ui_smoke_test:
                root.withdraw()
            context = create_window(root)
            if args.ui_smoke_test:
                root.update()
                if args.fixture:
                    actual = bytes(int(token, 16) for token in Path(args.fixture).read_text().split())
                    frames = Decoder().feed(actual)
                    for i in range(50):
                        row = dict(frames[i % len(frames)], elapsed_s=i*.1, arm_session_counts=i-25)
                        row['angle_target_counts'] = row['angle_target_q8']/256
                        context['notifications'].put(('frame', row))
                def finish_smoke():
                    assert len(context['canvas'].find_all()) > 15
                    context['closed'] = True
                    root.destroy()
                root.after(350, finish_smoke)
            root.mainloop()
            if args.ui_smoke_test:
                print('PASS hidden Tk window creation and update')
        except Exception as exc:
            (ROOT/'logger_start_error.txt').write_text(str(exc), encoding='utf-8')
            try:
                from tkinter import messagebox
                messagebox.showerror('记录工具启动失败', str(exc))
            except Exception:
                pass
            raise
