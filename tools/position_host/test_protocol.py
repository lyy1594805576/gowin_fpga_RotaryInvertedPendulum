# -*- coding: utf-8 -*-
"""协议层回归：命令帧顺序、op4/op5/op7、遥测解帧。

运行：python test_protocol.py
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P


def make_v3_frame(**over):
    """按 RTL 的拼接顺序造一帧 v3 遥测，用于解帧回归。"""
    values = dict(
        sequence=7, firmware_id=0x26101801, fpga_ms=123456, state=4,
        manual_mode=1, flags=0x0001, encoder_absolute=10000,
        arm_relative_counts=-260, position_target_counts=260,
        theta_raw=2971, theta_filtered=2972, theta_min=2960, theta_max=2980,
        angle_target_q8=2971 << 8, position_output_q8=-100 << 8,
        angle_error_q8=0, d_angle_q8=12, d_position_q8=-3,
        control_pwm=15, requested_pwm=-15, angle_integral_q8=100,
        position_integral_q8=-50,
        swing_pwm=200, swing_time_ticks=65, swing_return_ticks=95,
        angle_center=2971, center_range=400, encoder_direction=-1,
        angle_deadband=3, position_deadband=1,
        angle_kp=1750, angle_ki=47, angle_kd=27000,
        position_kp=1400, position_ki=10, position_kd=5509,
        pid_command_sequence=3, pid_active_sequence=3, pid_command_result=2,
        pid_command_bank=1, pid_angle_kp=1750, pid_angle_ki=47,
        pid_angle_kd=27000, pid_position_kp=1400, pid_position_ki=10,
        pid_position_kd=5509,
    )
    values.update(over)
    for short, full in (('theta', 'theta_filtered'), ('location', 'arm_relative_counts'),
                        ('target', 'position_target_counts'), ('pwm', 'requested_pwm')):
        if short in values:
            values[full] = values[short]
    payload = P._STRUCT_V3.pack(*[values[k] for k in
                                  P.FIELDS_V1 + P.CONFIG_FIELDS + P.PID_FIELDS])
    packet = P.HEADER_V3 + payload
    return packet + bytes((P.crc8(packet),))


class TestCommands(unittest.TestCase):
    def test_position_packet(self):
        packet = P.position_packet(5, 1234, 12)
        self.assertEqual(packet[:2], b'\x5a\xa5')
        self.assertEqual(packet[6], 4)
        self.assertEqual(struct.unpack('<2i', packet[8:16]), (1234, 12))

    def test_position_clamped(self):
        packet = P.position_packet(0, 99999, 99)
        counts, step = struct.unpack('<2i', packet[8:16])
        self.assertEqual(counts, P.ARM_REV_MAX)
        self.assertEqual(step, 64)

    def test_hold_packet(self):
        packet = P.hold_packet(3)
        self.assertEqual(packet[6], 5)
        self.assertEqual(struct.unpack('<6i', packet[8:32]), (0,) * 6)

    def test_friction_ks(self):
        packet = P.friction_packet(4, 7)
        self.assertEqual(packet[6], 7)
        self.assertEqual(struct.unpack('<i', packet[8:12])[0], 7)
        packet = P.friction_packet(4, 99)
        self.assertEqual(struct.unpack('<i', packet[8:12])[0], 12)

    def test_pid_order(self):
        """固件按 {kd_p,ki_p,kp_p,kd_a,ki_a,kp_a} 拼接，字段 0 必须是角度 KP。"""
        packet = P.pid_packet(1, 1, 0, (1750, 47, 27000, 1400, 10, 5509))
        self.assertEqual(struct.unpack('<6i', packet[8:32]),
                         (1750, 47, 27000, 1400, 10, 5509))

    def test_center_packet(self):
        packet = P.center_packet(3, 2971, 1)
        self.assertEqual(packet[6], 13)
        self.assertEqual(packet[7], 1)
        self.assertEqual(struct.unpack('<i', packet[8:12])[0], 2971)
        self.assertEqual(struct.unpack('<i', P.center_packet(3, 9999)[8:12])[0], 4095)

    def test_velocity_p(self):
        packet = P.velocity_p_packet(9, 512)
        self.assertEqual(packet[6], 15)
        self.assertEqual(struct.unpack('<i', packet[8:12])[0], 512)
        packet = P.velocity_p_packet(9, 9999)
        self.assertEqual(struct.unpack('<i', packet[8:12])[0], 1024)

    def test_bad_operation(self):
        with self.assertRaises(ValueError):
            P.command_packet(0, 6)

    def test_build_routes(self):
        link = P.Link()
        for op, val, val2, opcode in ((4, 100, 8, 4), (5, 0, 0, 5),
                                      (7, 3, 0, 7), (3, 0, 0, 3)):
            _, packet = link.build(op, val, val2)
            self.assertEqual(packet[6], opcode)


class TestDecode(unittest.TestCase):
    def test_v3_roundtrip(self):
        packet = make_v3_frame()
        frames = P.Decoder().feed(packet)
        self.assertEqual(len(frames), 1)
        f = frames[0]
        self.assertEqual(f['firmware_id'], 0x26101801)
        self.assertEqual(f['angle_center'], 2971)
        self.assertEqual(f['angle_kp'], 1750)
        self.assertEqual(f['position_kd'], 5509)
        self.assertEqual(f['location'], -260)
        self.assertEqual(f['target'], 260)

    def test_bad_crc(self):
        packet = bytearray(make_v3_frame())
        packet[20] ^= 0xFF
        self.assertEqual(P.Decoder().feed(bytes(packet)), [])


if __name__ == '__main__':
    unittest.main()
