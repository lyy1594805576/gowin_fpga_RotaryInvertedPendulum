"""Transactional USB runtime cascade PID tuning, arm position and nonlinear tuning.

op4 moves the arm to a target with a trapezoidal speed profile (firmware
0x26101803+); op4 payload word1 is that profile's peak speed in counts per
50 ms position tick. Firmware 0x26101802 accepts the same bytes but applies
word1 as a constant per-tick step instead of a curve peak.

op6 (archived experimental firmware0x26101804 only) sets four tuning overrides
used to scan small-angle jitter online without reflashing: angle deadband,
position deadband, motor deadzone and angle-D filter shift; -1 keeps the
firmware default. Production0x26101805..09 and0x26102201 reject op6.
op7 (0x26102201) reads/sets bounded actuator friction compensation independently
of PID gains. Its reply is tagged bank0x80, word0=actual KS PWM, other words0.
"""
import struct
import time

GAIN_FIELDS = ('angle_kp','angle_ki','angle_kd','position_kp','position_ki','position_kd')
READBACK_FIELDS = tuple('pid_'+name for name in GAIN_FIELDS)
RESULTS = {0:'上电默认值',1:'等待控制周期',2:'已生效',3:'CRC错误',4:'参数超出16位范围',
           5:'未知命令',6:'帧格式或超时错误',7:'读取成功',8:'未处于平衡状态',9:'模式编号错误'}
BANK_NAMES = ('自动','手动')
TUNING_FIELDS = ('angle_deadband','position_deadband','motor_deadzone','d_filter_shift')
TUNING_NAMES = ('角度死区', '位置死区', '电机死区', 'D 滤波移位')
# op6 payload word0..3: signed int32, -1 = keep firmware default.
TUNING_LIMITS = ((-1,20), (-1,20), (-1,10), (-1,4))
FRICTION_FF_FIRMWARE_ID = 0x26102201
FRICTION_FF_EXTENDED_FIRMWARE_ID = 0x26102202
FRICTION_FF_D_ONLY_FIRMWARE_ID = 0x26102401
FRICTION_FF_SMOOTH_FIRMWARE_ID = 0x26102402
FRICTION_FF_BANK = 0x80
# Actual RTL concatenation: bit3=FF configured, bit4=motor output active.
# Firmware01/02 already transmit this layout; no FPGA update is needed.
FRICTION_FF_FLAG_MASK = 1 << 3


def validate_friction_ff(ks):
    if type(ks) is not int or not -1 <= ks <= 12:
        raise ValueError('摩擦补偿强度须为0…12 PWM；0关闭，-1只读取')
    return ks


def friction_ff_limit(firmware_id):
    return 12 if firmware_id is not None and firmware_id >= FRICTION_FF_EXTENDED_FIRMWARE_ID else 8


def friction_ff_profile(firmware_id):
    if firmware_id == FRICTION_FF_SMOOTH_FIRMWARE_ID:
        return '两级D滤波 + 柔化前馈；同一KS的小指令补偿比旧版更缓'
    if firmware_id == FRICTION_FF_D_ONLY_FIRMWARE_ID:
        return '两级D滤波对照版；前馈曲线沿用0x26102202'
    return '原版前馈曲线；默认关闭'


def friction_ff_result_text(result):
    return {4:'补偿参数超出范围（新版本0…12，0x26102201上限8；-1读取，其余参数须为0）',
            5:'当前固件不支持摩擦补偿，请使用0x26102201版本',
            9:'摩擦补偿命令的bank须为0'}.get(result, RESULTS.get(result, '未知结果'))
TUNING_RESULT_TEXT = {
    4:'抖动参数超出范围（角度/位置死区 -1…20，电机死区 -1…10，D 滤波移位 -1…4，word4-5 必须为 0）',
    5:'固件不支持归档op6实验（仅0x26101804支持），新版本请使用摩擦前馈页',
}
# USB op4/op5 position commands. COUNTS_PER_REV is calibrated on the real arm:
# a physical full 360° turn read ~368° with the old 1040 constant, so
# 1040×368/360 ≈ 1063.1 → 1063 encoder counts per turn.
COUNTS_PER_REV = 1063
POSITION_LIMIT_COUNTS = 10*COUNTS_PER_REV   # ±10 turns = ±10630, matches RTL ARM_REV_MAX
POSITION_STEP_MAX = 64             # speed-curve peak in counts/50 ms tick; 0 = firmware default (8)
POSITION_RESULT_TEXT = {
    4:'位置参数超出范围（目标须在 ±10630 计数内，速度峰值 0…64，payload[2..5] 必须为 0）',
    5:'固件不支持位置控制（旧固件；需升级到 0x26101803，或至少 0x26101802）',
    8:'当前不在平衡态，位置命令被拒绝（内部目标保持不变）',
}


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (7 if value & 128 else 0)) & 255
    return value


def validate_gains(values):
    values = tuple(values)
    if len(values) != 6 or any(type(v) is not int or not -32768 <= v <= 32767 for v in values):
        raise ValueError('六项参数必须是 −32768…32767 的整数（实际系数 = 整数 / 4096）')
    return values


def validate_position(target, step=0):
    """Validate USB op4 payload: target counts and speed-curve peak (counts/50 ms).

    The parameter keeps the historical name ``step``; with firmware 0x26101803+
    it is the maximum (cruise) speed of the trapezoidal profile, not a fixed
    per-tick step. Validation is unchanged: 0…64, 0 = firmware default 8.
    """
    if type(target) is not int or not -POSITION_LIMIT_COUNTS <= target <= POSITION_LIMIT_COUNTS:
        raise ValueError('位置目标必须是 −10630…10630 的整数编码器计数（1063 计数 = 360°）')
    if type(step) is not int or not 0 <= step <= POSITION_STEP_MAX:
        raise ValueError('位置速度峰值必须是 0…64 的整数（计数/50ms；0 = 固件默认 8）')
    return target, step


def position_result_text(result):
    return POSITION_RESULT_TEXT.get(result, RESULTS.get(result, '未知结果'))


def validate_tuning(angle_deadband, position_deadband, motor_deadzone, d_filter_shift):
    """Validate the four USB op6 words; -1 keeps the firmware default."""
    values = (angle_deadband, position_deadband, motor_deadzone, d_filter_shift)
    for name,(low,high),value in zip(TUNING_NAMES,TUNING_LIMITS,values):
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f'{name}必须是 {low}…{high} 的整数')
    return values


def tuning_result_text(result):
    return TUNING_RESULT_TEXT.get(result, RESULTS.get(result, '未知结果'))


def command_packet(sequence, operation, bank=0, gains=(0,)*6):
    if operation not in (1,2,3,4,5,6,7) or bank not in (0,1) or (operation == 7 and bank != 0):
        raise ValueError('无效命令或模式')
    if operation in (1,2,3):
        payload = validate_gains(gains)
    elif operation == 4:
        payload = tuple(gains)
        if len(payload) != 6:
            raise ValueError('位置命令 payload 必须是六个 int32')
        target, step = validate_position(payload[0], payload[1])
        if any(type(value) is not int or value != 0 for value in payload[2:]):
            raise ValueError('位置命令 payload[2..5] 必须为 0')
        payload = (target, step, 0, 0, 0, 0)
    elif operation == 7:
        payload = tuple(gains)
        if len(payload) != 6:
            raise ValueError('摩擦补偿命令须包含六个int32参数')
        validate_friction_ff(payload[0])
        if any(type(value) is not int or value != 0 for value in payload[1:]):
            raise ValueError('摩擦补偿的其余五项参数须为0')
    elif operation == 6:
        payload = tuple(gains)
        if len(payload) != 6:
            raise ValueError('抖动调参 payload 必须是六个 int32')
        values = validate_tuning(*payload[:4])
        if any(type(value) is not int or value != 0 for value in payload[4:]):
            raise ValueError('抖动调参 payload[4..5] 必须为 0')
        payload = values + (0, 0)
    else:
        payload = tuple(gains)
        if len(payload) != 6 or any(type(value) is not int or value != 0 for value in payload):
            raise ValueError('保持命令的六项 payload 必须全部为 0')
        payload = (0,)*6
    body = struct.pack('<BBHBB6i',1,35,sequence & 65535,operation,bank,*payload)
    return b'\x5a\xa5'+body+bytes((crc8(body),))+b'\r\n'


class PIDLink:
    def __init__(self, log=lambda event: None, clock=time.monotonic):
        self.log, self.clock = log, clock
        self.pending = self.last_seen = self.last_ms = self.next_sequence = None
        self.available = False
        self.board = {}
        self.completed = None
        self.last_position = None
        self.last_tuning = None
        self.friction_ff_ks = None
        self.firmware_id = None
        self.message = '连接新版固件后可调参/定位；旧固件仍可记录波形'

    @property
    def fresh(self):
        return self.available and self.last_seen is not None and self.clock()-self.last_seen < 1

    def _check_ready(self):
        if not self.fresh:
            raise ValueError('未收到新版固件的有效实时数据，不能下发参数')
        if self.pending:
            raise ValueError('上一条命令仍在等待确认')

    def _transmit(self, port, operation, bank, payload, record, waiting):
        sequence = self.next_sequence
        packet = command_packet(sequence, operation, bank, payload)
        self.next_sequence = (sequence+1) & 65535
        self.completed = None
        record = dict(record, sequence=sequence, operation=operation, bank=bank)
        self.pending = dict(record, started=self.clock())
        self.log(dict(event='send',packet_hex=packet.hex(),**record))
        try:
            if port.write(packet) != len(packet):
                raise IOError('串口未完整写出命令')
        except Exception as exc:
            self.pending = None
            self.message = '发送结果未知，请读取板上值确认：'+str(exc)
            self.log(dict(event='unconfirmed',reason=str(exc),**record))
            raise ValueError(self.message) from exc
        self.message = waiting.format(sequence=sequence)

    def send(self, port, operation, bank, gains=(0,)*6):
        if operation not in (1,2,3):
            raise ValueError('无效命令：参数命令只接受 1/2/3，位置命令请用 send_position')
        self._check_ready()
        gains = validate_gains(gains)
        self._transmit(port, operation, bank, gains, dict(gains=gains),
                       f'命令 {{sequence}} 已发送，等待 {BANK_NAMES[bank]} 模式回读')

    def send_position(self, port, target_counts, step=0):
        """USB op4: move to target_counts with a trapezoidal speed profile.

        ``step`` is the profile's peak speed in counts/50 ms (1…64; 0 means
        firmware default 8). Firmware 0x26101803+ accelerates from 1, cruises
        at that peak and brakes to an exact stop without overshoot; older
        0x26101802 accepts the same payload as a constant per-tick step.
        """
        self._check_ready()
        target, step = validate_position(target_counts, step)
        self._transmit(port, 4, 0, (target, step, 0, 0, 0, 0),
                       dict(target=target, step=step),
                       f'位置命令 {{sequence}} 已发送（目标 {target} 计数，'
                       f'速度峰值 {step if step else "固件默认 8"} 计数/50ms），等待板上确认')

    def send_position_hold(self, port):
        """USB op5: freeze the internal position target at its current slewed value."""
        self._check_ready()
        self._transmit(port, 5, 0, (0,)*6, dict(),
                       '保持命令 {sequence} 已发送（冻结当前内部目标），等待板上确认')

    def send_tuning(self, port, angle_deadband, position_deadband, motor_deadzone, d_filter_shift):
        """USB op6: runtime nonlinear tuning for small-angle jitter scans.

        All four words are signed int32; -1 keeps the firmware default:
        angle deadband (-1..20 counts), position deadband (-1..20 counts),
        motor deadzone (-1..10 PWM), angle-D filter shift (-1..4). Any robot
        state accepts op6; op3 (restore defaults) clears these overrides.
        """
        self._check_ready()
        values = validate_tuning(angle_deadband, position_deadband, motor_deadzone, d_filter_shift)
        self._transmit(port, 6, 0, values+(0,0), dict(zip(TUNING_FIELDS,values)),
                       f'抖动调参命令 {{sequence}} 已发送（角度死区 {values[0]}，位置死区 {values[1]}，'
                       f'电机死区 {values[2]}，D移位 {values[3]}），等待板上确认')

    def send_friction_ff(self, port, ks=-1):
        """Op7: read(-1), disable(0), or set PWM1..12 (old0x26102201:1..8)."""
        self._check_ready()
        validate_friction_ff(ks)
        if self.firmware_id is not None and ks > friction_ff_limit(self.firmware_id):
            raise ValueError('当前板上版本上限为8 PWM；10/12需烧录0x26102202')
        self._transmit(port, 7, 0, (ks,0,0,0,0,0), dict(friction_ff_ks=ks),
                       '摩擦补偿命令 {sequence} 已发送，等待参数回读')

    def observe(self, frame):
        ms = frame.get('fpga_ms')
        reset = self.last_ms is not None and ms is not None and ((ms-self.last_ms)&0xffffffff)>0x80000000
        self.last_ms = ms
        if reset:
            if self.pending:
                self.log(dict(event='unconfirmed',reason='FPGA复位',**self.pending))
            self.pending = None
            self.board.clear()
            self.next_sequence = None
            self.completed = None
            self.last_position = None
            self.last_tuning = None
            self.friction_ff_ks = None
            self.message = 'FPGA已复位，参数回到固件默认值；可重新读取'
        if frame.get('format') != 'trace_v3_100hz':
            self.available = False
            return
        bank, result = frame['pid_command_bank'], frame['pid_command_result']
        self.available = True
        self.last_seen = self.clock()
        self.firmware_id = frame.get('firmware_id', self.firmware_id)
        if (frame.get('firmware_id',0) >= FRICTION_FF_FIRMWARE_ID and
                'flags' in frame and not (frame['flags'] & FRICTION_FF_FLAG_MASK)):
            self.friction_ff_ks = 0
        if self.next_sequence is None:
            self.next_sequence = (frame['pid_command_sequence']+1)&65535
        if bank in (0,1):
            self.board[bank] = validate_gains(frame[k] for k in READBACK_FIELDS)
        ff_values = tuple(frame[k] for k in READBACK_FIELDS)
        ff_readback_valid = (bank == FRICTION_FF_BANK and type(ff_values[0]) is int and
                             0 <= ff_values[0] <= friction_ff_limit(self.firmware_id) and ff_values[1:] == (0,)*5)
        if ff_readback_valid and result in (2,7):
            self.friction_ff_ks = ff_values[0]
        if not self.pending or frame['pid_command_sequence'] != self.pending['sequence']:
            return
        pending = self.pending
        # Position/tuning commands do not use the bank byte for matching (host always sends 0).
        if pending['operation'] not in (4,5,6,7) and bank != pending['bank']:
            return
        if result == 1:
            self.message = '板上已接收，等待控制周期生效'
            return
        if pending['operation'] == 7:
            reading = pending['friction_ff_ks'] == -1
            success = ff_readback_valid and ((reading and result == 7) or
                (not reading and result == 2 and frame['pid_active_sequence'] == pending['sequence']
                 and ff_values[0] == pending['friction_ff_ks']))
            self.pending = None
            self.completed = dict(pending, success=success, result=result,
                                  active_sequence=frame['pid_active_sequence'],
                                  readback=ff_values if ff_readback_valid else None)
            self.message = (f'板上摩擦补偿强度：{ff_values[0]} PWM（0为关闭）' if success else
                            '摩擦补偿未确认：'+friction_ff_result_text(result))
            self.log(dict(event='ack' if success else 'reject', **self.completed))
            return
        if pending['operation'] in (4,5):
            active = frame['pid_active_sequence']
            success = result == 2 and active == pending['sequence']
            self.pending = None
            self.completed = dict(pending,success=success,result=result,
                                  active_sequence=active,readback=self.board.get(bank))
            if success:
                if pending['operation'] == 4:
                    self.last_position = dict(self.completed)
                    self.message = '位置命令已生效（目标 %+d 计数）' % pending['target']
                else:
                    self.message = '已冻结内部位置目标（停止移动）'
            else:
                self.message = '位置命令被拒绝：'+position_result_text(result)
            self.log(dict(event='ack' if success else 'reject', **self.completed))
            return
        if pending['operation'] == 6:
            active = frame['pid_active_sequence']
            success = result == 2 and active == pending['sequence']
            self.pending = None
            self.completed = dict(pending,success=success,result=result,
                                  active_sequence=active,readback=self.board.get(bank))
            if success:
                self.last_tuning = {name: pending[name] for name in TUNING_FIELDS}
                self.message = ('抖动参数已生效：角度死区 %+d、位置死区 %+d、电机死区 %+d、D移位 %+d'
                                % tuple(pending[name] for name in TUNING_FIELDS))
            else:
                self.message = '抖动参数被拒绝：'+tuning_result_text(result)
            self.log(dict(event='ack' if success else 'reject', **self.completed))
            return
        success = ((pending['operation']==2 and result==7) or
                   (pending['operation'] in (1,3) and result==2 and
                    frame['pid_active_sequence']==pending['sequence']))
        if success and pending['operation']==1:
            success = self.board[bank] == pending['gains']
        if success and pending['operation']==3:
            self.last_tuning = None
        self.pending = None
        self.completed = dict(pending,success=success,readback=self.board.get(bank))
        self.log(dict(event='ack' if success else 'reject',result=result,
                      active_sequence=frame['pid_active_sequence'],**self.completed))
        if success:
            if pending['operation']==2:
                self.message = f'{BANK_NAMES[bank]} PID 读取成功'
            elif pending['operation']==3:
                self.message = f'{BANK_NAMES[bank]} PID 默认值已恢复（在线抖动覆盖已同时清除）'
            else:
                self.message = f'{BANK_NAMES[bank]} PID 已生效'
        else:
            self.message = '未确认生效：'+RESULTS.get(result,'未知结果')+'；请重新读取板上值'

    def check_timeout(self):
        if self.pending and self.clock()-self.pending['started'] >= 2:
            operation = self.pending.get('operation')
            self.log(dict(event='unconfirmed',reason='确认超时',**self.pending))
            self.pending = None
            if operation in (4,5):
                self.message = '位置命令确认超时，是否生效未知；请以遥测内部目标为准（不会自动重发）'
            elif operation == 6:
                self.message = '抖动调参确认超时，是否生效未知；请重新下发或读取板上状态（不会自动重发）'
            elif operation == 7:
                self.friction_ff_ks = None
                self.message = '摩擦补偿确认超时，生效情况未知；请读取板上值（不会自动重发）'
            else:
                self.message = '确认超时，生效情况未知；请读取板上值（不会自动重发）'

    def snapshot(self):
        pending = self.pending
        return dict(fresh=self.fresh,pending=bool(pending),board=dict(self.board),
                    completed=self.completed,message=self.message,
                    pending_operation=pending['operation'] if pending else None,
                    pending_sequence=pending['sequence'] if pending else None,
                    friction_ff_ks=self.friction_ff_ks,
                    last_position=dict(self.last_position) if self.last_position else None,
                    last_tuning=dict(self.last_tuning) if self.last_tuning else None)
