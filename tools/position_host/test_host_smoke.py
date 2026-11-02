# -*- coding: utf-8 -*-
"""无硬件冒烟测试：建窗口、灌遥测、点按钮、检查发出去的字节。

运行：python test_host_smoke.py
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

os.environ['POSITION_HOST_NO_SESSION'] = '1'

import protocol as P
import host_app
from test_protocol import make_v3_frame


class DummyWorker:
    def __init__(self):
        self.sent = []

    def send(self, data):
        self.sent.append(bytes(data))

    def stop(self):
        pass


def frame(**over):
    data = make_v3_frame(**over)
    return P.Decoder().feed(data)[0]


class HostSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tkinter as tk
        cls.root = tk.Tk()
        try:
            cls.root.withdraw()
        except tk.TclError as exc:
            raise unittest.SkipTest('no display: %s' % exc)
        cls.app = host_app.App(cls.root)
        cls.app.worker = DummyWorker()
        cls.app.connected = True

    @classmethod
    def tearDownClass(cls):
        try:
            cls.root.destroy()
        except Exception:
            pass

    def setUp(self):
        self.app.worker.sent.clear()
        self.app.link.reset()
        self.app.link.next_sequence = 0
        self.app.last_state = None
        self.app.wave.clear()
        self.app._move_queue = []
        self.app._curve_plan = None
        self.app._curve_job = None

    def feed(self, **over):
        self.app._on_frame(frame(**over))

    def last_packet(self):
        self.assertTrue(self.app.worker.sent, 'nothing was sent')
        packet = self.app.worker.sent[-1]
        self.assertEqual(len(packet), 35)
        self.assertEqual(packet[:2], b'\x5a\xa5')
        self.assertEqual(packet[32], P.crc8(packet[2:32]))
        self.assertEqual(packet[33:], b'\r\n')
        return packet

    def settle(self, location=0, target=0, manual_mode=1):
        """灌两帧让 link 变新鲜，并且状态为平衡。"""
        self.feed(state=4, manual_mode=manual_mode, location=location, target=target)
        self.feed(state=4, manual_mode=manual_mode, location=location, target=target)

    # ------------------------------------------------------------------
    def test_tabs(self):
        texts = [self.app.notebook.tab(i, 'text')
                 for i in range(self.app.notebook.index('end'))]
        self.assertEqual(texts, ['实时波形', '轨迹曲线', '定位跟踪', '参数', '摇杆'])

    def test_curve_plan_streams_op4(self):
        self.app.var_curve_stream.set(False)
        self.settle(location=0, target=0)
        self.app.last_location = 0
        self.app.var_speed.set('8')
        self.app._goto_deg(30)
        plan = self.app._curve_plan
        self.assertIsNotNone(plan)
        self.assertEqual(plan['target'], P.deg_to_counts(30))
        self.app._stop_curve()
        self.assertIsNone(self.app._curve_plan)

    def test_goto_curve_staged(self):
        self.app.var_curve_stream.set(False)
        self.app.var_auto_step.set(True)
        self.app.var_step90.set('8')
        self.app.var_curve_exec.set(True)
        self.settle(location=0, target=0)
        self.app.last_location = 0
        self.app._apply_curve_preset('软停')
        self.app._goto_deg(90)
        plan = self.app._curve_plan
        self.assertIsNotNone(plan)
        self.assertEqual(plan['target'], P.deg_to_counts(90))
        self.assertEqual(len(plan['stages']), 4)          # 曲线分 4 段
        self.assertEqual(plan['stages'][-1][0], P.deg_to_counts(90))
        counts, step = struct.unpack('<2i', self.last_packet()[8:16])
        self.assertEqual(counts, plan['stages'][0][0])
        self.assertEqual(step, plan['stages'][0][1])

    def test_goto_single_when_curve_off(self):
        self.app.var_curve_stream.set(False)
        self.app.var_auto_step.set(False)
        self.app.var_speed.set('8')
        self.app.var_curve_exec.set(False)
        self.settle(location=0, target=0)
        self.app.last_location = 0
        self.app._goto_deg(90)
        plan = self.app._curve_plan
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan['stages']), 1)          # 关闭曲线 → 一条直达
        counts, _ = struct.unpack('<2i', self.last_packet()[8:16])
        self.assertEqual(counts, P.deg_to_counts(90))

    def test_hold_sends_op5(self):
        self.settle(location=100, target=100)
        self.app._hold_here()
        packet = self.last_packet()
        self.assertEqual(packet[6], 5)
        self.assertEqual(struct.unpack('<6i', packet[8:32]), (0,) * 6)

    def test_ks_sends_op7(self):
        self.settle()
        self.app.var_ks.set('5')
        self.app._write_ks()
        packet = self.last_packet()
        self.assertEqual(packet[6], 7)
        self.assertEqual(struct.unpack('<i', packet[8:12])[0], 5)

    def test_pid_order_and_readback(self):
        self.app.var_pid_a_kp.set('1750')
        self.app.var_pid_a_ki.set('47')
        self.app.var_pid_a_kd.set('27000')
        self.app.var_pid_p_kp.set('1400')
        self.app.var_pid_p_ki.set('10')
        self.app.var_pid_p_kd.set('5509')
        self.app.var_pid_bank.set(0)
        self.settle()
        self.app._write_pid()
        packet = self.last_packet()
        self.assertEqual(packet[6], 1)
        self.assertEqual(struct.unpack('<6i', packet[8:32]),
                         (1750, 47, 27000, 1400, 10, 5509))
        # 回读核对：让遥测返回同样的增益，并把时间推到 0.6 s 之后
        self.app._pid_check = (0, 0, (1750, 47, 27000, 1400, 10, 5509))
        self.feed(state=4, manual_mode=0, angle_kp=1750, angle_ki=47,
                  angle_kd=27000, position_kp=1400, position_ki=10,
                  position_kd=5509)
        self.assertIn('已生效', self.app.var_pid_status.get())

    def test_pid_readback_mismatch(self):
        self.app._pid_check = (0, 0, (1750, 47, 27000, 1400, 10, 5509))
        self.feed(state=4, manual_mode=0, angle_kp=5509, angle_ki=10,
                  angle_kd=1400, position_kp=27000, position_ki=47,
                  position_kd=1750)
        self.assertIn('不一致', self.app.var_pid_status.get())

    def test_analyze_move(self):
        rows = []
        for i in range(400):
            t = i * 0.01
            tgt = 0.0 if t < 1.0 else 90.0
            loc = 0.0 if t < 1.0 else min(90.0, (t - 1.0) * 60.0)
            rows.append((t, 0.5, loc, tgt, 10, 4))
        m = host_app.analyze_move(rows)
        self.assertIsNotNone(m)
        self.assertGreater(m['peak_v'], 50.0)
        self.assertIn('rest_pkpk', m)


if __name__ == '__main__':
    unittest.main()
