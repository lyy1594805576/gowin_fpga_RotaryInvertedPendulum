# -*- coding: utf-8 -*-
"""位置控制上位机协议层（无 UI 依赖，可单独测试）。

固件（jiangxiedaolibai_pure_pd，固件标识 0x26101801 起）：

USB 口 115200 8N1
  遥测 v3：111 字节/帧，约 100 Hz
      0..3    A6 6A 03 6F         帧头(版本3, 长度111)
      4..5    packet_seq          帧序号
      6..9    firmware_id         固件标识(小端)
      10..13  uptime_ms           板上毫秒
      14      state               0停止 1判断 21..34起摆 4平衡
      15      manual_mode         1手动 0自动
      16..17  flags               见 TRACE_FLAGS
      18..19  arm                 编码器绝对计数
      20..21  location            转臂相对位移(相对本次起摆零点, 有符号)
      22..23  pos_target          位置环斜坡目标(有符号, 同 location 坐标)
      24..25  theta_raw           原始角度(12bit)
      26..27  theta               滤波后角度(12bit)
      28..31  theta_min/max       窗口内最值
      32..35  angle_target_q8     内环目标角度 Q8
      36..39  pos_output_q8       位置环输出 Q8
      40..43  angle_error_q8      角度环误差 Q8
      44..47  d_angle_q8          角度微分 Q8
      48..51  d_pos_q8            位置微分 Q8
      52..53  u_duty              角度环输出(±255)
      54..55  requested_pwm       实际电机指令(±255)
      56..59  angle_integral_q8
      60..63  pos_integral_q8
      64..91  config_words        14 项只读配置(见 CONFIG_FIELDS)
      92..93  pid_command_sequence
      94..95  pid_active_sequence
      96      pid_command_result
      97      pid_command_bank
      98..109 pid_readback_gains   六项回读 Q12
      110     CRC8/ATM            覆盖字节 0..109，多项式0x07、初值0、不反射

  命令：35 字节固定帧
      0..1    5A A5
      2..3    01 23               命令版本1、长度35
      4..5    命令序号(小端)
      6       操作 1=整组设置 2=读取 3=恢复默认 4=位置目标 5=位置斜坡步长
      7       参数组 0=自动 1=手动（op4/op5 忽略）
      8..31   六个有符号32位；op4/op5 只用第一个字段
                  op4: 目标位置(编码器计数, 有符号, |x|<=10400)
                  op5: 每 50ms 逼近步长(1..64 计数)
      32      CRC8/ATM 覆盖 2..31
      33..34  0D 0A

结果码：
    0上电默认 1等待控制周期 2已生效 3CRC错误 4参数越界 5未知命令
    6帧格式/超时 7读取成功 9参数组错误 10未平衡(op4被忽略)

编码器实测 1064 计数/圈（转一圈界面角度变化 368.3° → 1040×368.3/360≈1064），
位置环 51ms 一拍（POS_DIV=17 × 3ms）。
"""

import struct
import time

# ---------------------------------------------------------------- 常量

COUNTS_PER_REV = 1064.0
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV       # 0.3383459... °/计数
POSITION_LIMIT = 10400                       # 与固件 ARM_REV_MAX 一致
STEP_MIN, STEP_MAX = 1, 64
POS_LOOP_S = 0.051                           # 位置环周期(17×3ms)
ARM_REV_MAX = 10630      # 固件 op4 目标限幅（±10 圈）
PROFILE_POINTS = 16                          # 速度曲线点数
CURVE_MIN, CURVE_MAX = -256, 256             # 曲线值 = 最大步长的 1/256（负数=反向制动）
FF_MIN, FF_MAX = -12800, 12800               # 参考速度前馈 Q8（100% = 6400）
CURVE_FLAT = [256] * PROFILE_POINTS
JOG_MIN, JOG_MAX = -64, 64                   # 摇杆速度（计数/51ms）

HEADER_V3 = bytes((0xA6, 0x6A, 3, 111))
HEADER_V2 = bytes((0xA6, 0x6A, 2, 93))
HEADER_V1 = bytes((0xA6, 0x6A, 1, 65))

FIELDS_V1 = (
    'sequence', 'firmware_id', 'fpga_ms', 'state', 'manual_mode', 'flags',
    'encoder_absolute', 'arm_relative_counts', 'position_target_counts',
    'theta_raw', 'theta_filtered', 'theta_min', 'theta_max',
    'angle_target_q8', 'position_output_q8', 'angle_error_q8',
    'd_angle_q8', 'd_position_q8', 'control_pwm', 'requested_pwm',
    'angle_integral_q8', 'position_integral_q8',
)
CONFIG_FIELDS = (
    'swing_pwm', 'swing_time_ticks', 'swing_return_ticks', 'angle_center',
    'center_range', 'encoder_direction', 'angle_deadband', 'position_deadband',
    'angle_kp', 'angle_ki', 'angle_kd', 'position_kp', 'position_ki', 'position_kd',
)
PID_FIELDS = (
    'pid_command_sequence', 'pid_active_sequence', 'pid_command_result',
    'pid_command_bank', 'pid_angle_kp', 'pid_angle_ki', 'pid_angle_kd',
    'pid_position_kp', 'pid_position_ki', 'pid_position_kd',
)

_STRUCT_V1 = struct.Struct('<HIIBBHHhhHHHHiiiiihhii')
_STRUCT_V2 = struct.Struct(_STRUCT_V1.format + '14h')
_STRUCT_V3 = struct.Struct(_STRUCT_V2.format + 'HHBB6h')
assert _STRUCT_V1.size == 60 and _STRUCT_V2.size == 88 and _STRUCT_V3.size == 106

PROTOCOLS = {
    HEADER_V1: (_STRUCT_V1, FIELDS_V1, 'trace_v1_100hz'),
    HEADER_V2: (_STRUCT_V2, FIELDS_V1 + CONFIG_FIELDS, 'trace_v2_100hz'),
    HEADER_V3: (_STRUCT_V3, FIELDS_V1 + CONFIG_FIELDS + PID_FIELDS, 'trace_v3_100hz'),
}

STATES = {
    0: '停止', 1: '判断', 4: '平衡',
    21: '正向冲量', 22: '正向保持', 23: '反向冲量', 24: '反向保持',
    31: '反向冲量', 32: '反向保持', 33: '正向冲量', 34: '正向保持',
}

RESULTS = {
    0: '上电默认值', 1: '等待控制周期', 2: '已生效', 3: 'CRC错误',
    4: '参数越界', 5: '未知命令', 6: '帧格式或超时错误', 7: '读取成功',
    9: '参数组错误', 10: '未平衡，位置命令被忽略',
}

FLAG_BITS = (
    (0, '平衡中角度在窗口内'), (2, '角度环饱和'), (3, '位置环饱和'),
    (5, '电机有输出'), (6, '自动KI已递减'), (7, '自动KI已降到稳态'),
    (8, '当前KI可用8位表示'),
)

ARM_DEG_PER_COUNT = DEG_PER_COUNT


def crc8(data):
    """CRC8/ATM：多项式 0x07、初值 0、不反射，与 RTL 一致。"""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (7 if value & 128 else 0)) & 255
    return value


# ---------------------------------------------------------------- 单位换算

def counts_to_deg(counts):
    return counts * DEG_PER_COUNT


def deg_to_counts(deg):
    return int(round(deg / DEG_PER_COUNT))


def step_to_deg_s(step):
    """op5 的步长(计数/50ms) -> 约等于 °/s。"""
    return step * (DEG_PER_COUNT / POS_LOOP_S)


def deg_s_to_step(deg_s):
    return int(round(deg_s * POS_LOOP_S / DEG_PER_COUNT))


def clamp_position(counts):
    return max(-POSITION_LIMIT, min(POSITION_LIMIT, int(counts)))


def clamp_step(step):
    return max(STEP_MIN, min(STEP_MAX, int(step)))


def clamp_curve(points):
    points = [int(max(CURVE_MIN, min(CURVE_MAX, p))) for p in points][:PROFILE_POINTS]
    if len(points) < PROFILE_POINTS:
        points += [points[-1] if points else CURVE_MAX] * (PROFILE_POINTS - len(points))
    return points


def curve_presets():
    """内置曲线：匀速 / 梯形 / S 形 / 慢起慢停。值域 -256~256。

    曲线的纵轴是“当前速度 / 最大速度”，梯形和 S 形都是 0 起步、中间匀速、
    末端回到 0：配合固件的时间轴（每点占相同时间）就是标准的加速—匀速—减速。
    """
    n = PROFILE_POINTS
    trapezoid = []
    for i in range(n):
        t = i / (n - 1.0)
        if t < 0.25:
            trapezoid.append(int(round(256 * t / 0.25)))
        elif t < 0.75:
            trapezoid.append(256)
        else:
            trapezoid.append(int(round(256 * (1.0 - (t - 0.75) / 0.25))))

    def smoothstep(x):
        x = max(0.0, min(1.0, x))
        return 3 * x * x - 2 * x * x * x

    smooth = []
    for i in range(n):
        t = i / (n - 1.0)
        if t < 0.25:
            shape = smoothstep(t / 0.25)
        elif t < 0.75:
            shape = 1.0
        else:
            shape = smoothstep((1.0 - t) / 0.25)
        smooth.append(int(round(256 * shape)))
    return {
        '匀速': list(CURVE_FLAT),
        '梯形': trapezoid,
        'S 形': smooth,
        '慢起慢停': [32, 96, 160, 224] + [256] * 8 + [224, 160, 96, 32],
        # 软停：减速段占一半以上（最后 9 个点都在降速），90° 这种短移动
        # 到点前已经降到很慢，转臂不会带着速度冲过去。
        '软停': [0, 50, 130, 200, 245, 256, 256, 220,
                170, 120, 75, 45, 25, 12, 5, 0],
    }


def profile_scale(points, step):
    """给固件的时间基准：每计数对应多少段长（Q16）。

    固件用 K_q8 = (行程 × scale) >> 16 得到“每个曲线点占多少拍”，于是每段
    走过的距离正比于点值，整段距离正好等于行程；曲线为 0 的点几乎不占距离，
    不会再把起步卡死。曲线净面积 <= 0 时返回 0（固件退回恒速）。
    """
    pts = clamp_curve(points)
    area = sum(max(0, p) for p in pts)
    if area <= 0:
        return 0
    step = clamp_step(step)
    scale = int(round(65536.0 * 65536.0 / (area * step)))
    return max(0, min(0x7FFFFFFF, scale))


def curve_time_estimate(points, step, counts):
    """按曲线估算走完 counts 计数需要多少秒（仅用于界面提示）。

    固件把 16 个点平均分配到时间轴上，每点的距离正比于点值，所以总拍数
    = 16 × K = 4096×行程/(面积×步长)。
    """
    points = clamp_curve(points)
    counts = abs(int(counts))
    if counts <= 0:
        return 0.0
    area = sum(max(0, p) for p in points)
    if area <= 0:
        return float('inf')
    step = clamp_step(step)
    return 4096.0 * counts / (area * step) * POS_LOOP_S


def clamp_jog(speed):
    return max(JOG_MIN, min(JOG_MAX, int(speed)))


# ---------------------------------------------------------------- 组帧

def command_packet(sequence, operation, bank=0, values=(0, 0, 0, 0, 0, 0)):
    """35 字节命令帧。values 为六个有符号32位，op4/op5/op6 只用前两个。"""
    if operation not in (1, 3, 4, 5, 7, 13, 15, 16):
        raise ValueError('未知操作码 %r' % (operation,))
    if bank not in (0, 1):
        raise ValueError('参数组必须是 0(自动) 或 1(手动)')
    values = tuple(int(v) for v in values)
    if len(values) != 6:
        raise ValueError('需要六个字段')
    for v in values:
        if not -2**31 <= v < 2**31:
            raise ValueError('字段超出 32 位范围')
    body = struct.pack('<BBHBB6i', 1, 35, sequence & 0xFFFF, operation, bank, *values)
    return b'\x5a\xa5' + body + bytes((crc8(body),)) + b'\r\n'


def position_packet(sequence, counts, step=8):
    """op4：位置目标 + 逼近步长（计/50ms，0 表示用固件的 ROT_STEP）。"""
    counts = max(-ARM_REV_MAX, min(ARM_REV_MAX, int(counts)))
    step = max(0, min(64, int(step)))
    return command_packet(sequence, 4, 0, (counts, step, 0, 0, 0, 0))


def hold_packet(sequence):
    """op5：冻结在当前位置（停止梯形运动）。"""
    return command_packet(sequence, 5, 0, (0, 0, 0, 0, 0, 0))


def friction_read_packet(sequence):
    """op7 + 字段0 = -1：读回当前摩擦 KS（板子用 bank=0x80 回帧）。"""
    return command_packet(sequence, 7, 0, (-1, 0, 0, 0, 0, 0))


def ff_packet(sequence, q8):
    """op16：加速度前馈增益（Q8，256=1.0，0=关闭）。"""
    q8 = max(0, min(4096, int(q8)))
    return command_packet(sequence, 16, 0, (q8, 0, 0, 0, 0, 0))


def defaults_packet(sequence):
    """op3：把当前参数组的增益恢复成固件默认值。"""
    return command_packet(sequence, 3, 0, (0, 0, 0, 0, 0, 0))


def friction_packet(sequence, ks):
    """op7：摩擦前馈 KS（0~12，0 = 关闭）。"""
    ks = max(0, min(12, int(ks)))
    return command_packet(sequence, 7, 0, (ks, 0, 0, 0, 0, 0))


def velocity_p_packet(sequence, q8):
    """op15：速度环 P 系数，Q8（0 = 关闭，256 = 100%）。"""
    q8 = max(0, min(1024, int(q8)))
    return command_packet(sequence, 15, 0, (q8, 0, 0, 0, 0, 0))


def center_packet(sequence, value, bank=0):
    """op13：倒立摆平衡点中值（12bit 0~4095），bank 0=自动 1=手动。"""
    value = max(0, min(4095, int(value)))
    return command_packet(sequence, 13, 1 if bank else 0, (value, 0, 0, 0, 0, 0))


def pid_packet(sequence, operation, bank, gains):
    if operation not in (1, 2, 3):
        raise ValueError('PID 操作码必须是 1/2/3')
    gains = tuple(int(g) for g in gains)
    if len(gains) != 6 or any(not -32768 <= g <= 32767 for g in gains):
        raise ValueError('六项参数必须是 −32768…32767 的整数')
    return command_packet(sequence, operation, bank, gains)


# ---------------------------------------------------------------- 解帧

class Decoder:
    """字节流 -> 帧字典。容忍旧固件(v1/v2)与 9600 ASCII 兼容流。"""

    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.dropped = 0

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
                payload, fields, name = PROTOCOLS[header]
                frame = dict(zip(fields, payload.unpack(packet[4:-1])))
                if (frame['state'] not in STATES or frame['manual_mode'] not in (0, 1)
                        or any(frame[k] > 4095 for k in
                               ('theta_raw', 'theta_filtered', 'theta_min', 'theta_max'))):
                    del self.buffer[0]
                    continue
                frame['format'] = name
                frame['host_time'] = time.monotonic()
                # 常用短名，UI 直接读；原始字段名保持不变以便和 CSV/文档对照。
                frame['theta'] = frame['theta_filtered']
                frame['location'] = frame['arm_relative_counts']
                frame['target'] = frame['position_target_counts']
                frame['pwm'] = frame['requested_pwm']
                frames.append(frame)
                del self.buffer[:length]
            elif self.buffer[0] == ord('P') and len(self.buffer) > 8:
                # 旧固件 9600 文本行 "P±xxxxx Ayyyyy Ttttt Lllll Hhhhh"
                newline = self.buffer.find(b'\n')
                if newline < 0:
                    if len(self.buffer) < 128:
                        break
                    del self.buffer[0]
                    self.dropped += 1
                    continue
                line = bytes(self.buffer[:newline + 1]).strip()
                del self.buffer[:newline + 1]
                parts = line.split()
                try:
                    values = {p[:1]: int(p[1:]) for p in parts}
                    if {'P', 'A', 'T'} <= set(values):
                        frames.append({
                            'format': 'legacy_ascii_2hz',
                            'position_output_q8': values['P'] * 256,
                            'encoder_absolute': values['A'] & 0xFFFF,
                            'theta_filtered': values['T'],
                            'theta_min': values.get('L', values['T']),
                            'theta_max': values.get('H', values['T']),
                            'state': -1, 'manual_mode': -1,
                            'host_time': time.monotonic(),
                        })
                except ValueError:
                    self.bad_crc += 1
            elif any(h.startswith(bytes(self.buffer)) for h in PROTOCOLS):
                break
            else:
                del self.buffer[0]
                self.dropped += 1
        return frames


# ---------------------------------------------------------------- 命令事务

class Pending:
    __slots__ = ('sequence', 'operation', 'value', 'started')

    def __init__(self, sequence, operation, value, started):
        self.sequence = sequence
        self.operation = operation
        self.value = value
        self.started = started


class Link:
    """单条在途命令 + 遥测确认。totally UI 无关，便于单测。"""

    TIMEOUT_S = 1.0

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.next_sequence = 0
        self.pending = None
        self.last_frame_at = None
        self.last_fpga_ms = None
        self.firmware_id = None
        self.format = None
        self.board_target = None
        self.board_step = None
        self.message = '未连接'

    # -- 状态 --
    @property
    def fresh(self):
        return (self.last_frame_at is not None
                and self.clock() - self.last_frame_at < 0.5)

    @property
    def busy(self):
        return self.pending is not None

    def reset(self):
        self.next_sequence = 0
        self.pending = None
        self.last_frame_at = None
        self.last_fpga_ms = None
        self.firmware_id = None
        self.format = None
        self.board_target = None
        self.board_step = None

    # -- 接收 --
    def observe(self, frame):
        """喂一帧遥测；返回 (event, detail)。

        event: 'frame' / 'ack' / 'reject' / 'reset' / 'stale_format'
        """
        now = self.clock()
        self.last_frame_at = now
        self.format = frame.get('format')
        ms = frame.get('fpga_ms')
        reset = (self.last_fpga_ms is not None and ms is not None
                 and ((ms - self.last_fpga_ms) & 0xFFFFFFFF) > 0x80000000)
        self.last_fpga_ms = ms
        if reset:
            self.pending = None
            self.next_sequence = 0
            self.board_target = None
            self.message = 'FPGA 已复位，运行参数回到固件默认值'
            return 'reset', None
        if frame.get('format') != 'trace_v3_100hz':
            return 'stale_format', None

        self.firmware_id = frame.get('firmware_id')
        if self.next_sequence == 0 and frame.get('pid_command_sequence') is not None:
            self.next_sequence = (frame['pid_command_sequence'] + 1) & 0xFFFF
        if frame.get('position_target_counts') is not None:
            self.board_target = frame['position_target_counts']

        pending = self.pending
        if pending is None:
            return 'frame', None
        if frame.get('pid_command_sequence') != pending.sequence:
            return 'frame', None
        result = frame.get('pid_command_result')
        if result == 1:
            self.message = '板上已接收，等待控制周期生效'
            return 'frame', None
        success = (result == 2
                   and frame.get('pid_active_sequence') == pending.sequence)
        self.pending = None
        if success:
            self.message = ('位置目标已生效' if pending.operation == 4 else
                            '已冻结在当前位置' if pending.operation == 5 else
                            '摩擦前馈 KS 已生效' if pending.operation == 7 else
                            '平衡点中值已生效' if pending.operation == 13 else
                            '速度环 P 已生效' if pending.operation == 15 else
                            '已恢复默认增益' if pending.operation == 3 else
                            'PID 参数已生效')
            return 'ack', pending
        self.message = '命令未确认：' + RESULTS.get(result, '未知结果(%r)' % (result,))
        return 'reject', (pending, result)

    # -- 发送 --
    def build(self, operation, value, value2=0, gains=None):
        """op1 写 PID / op3 恢复默认 / op4 目标+步长 / op5 冻结 / op7 摩擦 KS。"""
        sequence = self.next_sequence
        if gains is not None and operation in (1, 3):
            packet = pid_packet(sequence, operation, value2, gains)
        elif operation == 3:
            packet = defaults_packet(sequence)
        elif operation == 4:
            packet = position_packet(sequence, value, value2)
        elif operation == 5:
            packet = hold_packet(sequence)
        elif operation == 7:
            packet = friction_packet(sequence, value)
        elif operation == 13:
            packet = center_packet(sequence, value, value2)
        elif operation == 15:
            packet = velocity_p_packet(sequence, value)
        elif operation == 16:
            packet = ff_packet(sequence, value)
        else:
            packet = command_packet(sequence, operation, 0, (value, 0, 0, 0, 0, 0))
        self.next_sequence = (sequence + 1) & 0xFFFF
        self.pending = Pending(sequence, operation, value, self.clock())
        self.message = '命令 %d 已发送，等待回读确认' % sequence
        return sequence, packet

    def build_ks_read(self):
        sequence = self.next_sequence
        packet = friction_read_packet(sequence)
        self.next_sequence = (sequence + 1) & 0xFFFF
        self.pending = Pending(sequence, 7, -1, self.clock())
        self.message = '读取 KS 已发送'
        return sequence, packet

    def check_timeout(self):
        if self.pending and self.clock() - self.pending.started > self.TIMEOUT_S:
            lost = self.pending
            self.pending = None
            self.message = '命令 %d 确认超时（不会自动重发）' % lost.sequence
            return lost
        return None


def decode_flags(flags):
    if flags is None:
        return []
    return [text for bit, text in FLAG_BITS if flags & (1 << bit)]
