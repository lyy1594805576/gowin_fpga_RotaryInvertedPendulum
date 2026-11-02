"""Friction FF packet, tagged ACK, recorder and hidden Tk integration checks."""
from pathlib import Path
import argparse
import csv
import json
import struct
import tempfile
import time
import tkinter as tk
from unittest.mock import patch
import runtime_pid as pid
import auto_swing_logger as logger
from test_runtime_pid import frame, packet, AUTO

ROOT = Path(__file__).resolve().parents[1]


def fixtures():
    cases = [(7,0,(2,0,0,0,0,0)), (7,0,(-1,0,0,0,0,0)),
             (7,0,(0,0,0,0,0,0)), (7,0,(12,0,0,0,0,0)),
             (7,0,(13,0,0,0,0,0)), (7,0,(-2,0,0,0,0,0)),
             (7,0,(2,1,0,0,0,0)), (7,1,(2,0,0,0,0,0)),
             (7,0,(2,0,0,0,0,0)), (6,0,(0,0,0,0,0,0)),
             (4,0,(100,8,0,0,0,0)), (5,0,(0,)*6), (3,0,(0,)*6)]
    result = []
    for i,(op,bank,values) in enumerate(cases):
        body = struct.pack('<BBHBB6i',1,35,301+i,op,bank,*values)
        data = b'\x5a\xa5'+body+bytes((pid.crc8(body),))+b'\r\n'
        if i == 8:
            data = data[:32]+bytes((data[32]^1,))+data[33:]
        result.extend(data)
    (ROOT/'sim/friction_ff_commands.hex').write_text(''.join(f'{v:02x}\n' for v in result),encoding='ascii')


def test_protocol():
    for ks in (-1,0,2,8,10,12):
        p = pid.command_packet(65535,7,0,(ks,0,0,0,0,0))
        assert len(p) == 35 and struct.unpack_from('<6i',p,8) == (ks,0,0,0,0,0)
        assert p[32] == pid.crc8(p[2:32])
    for ks in (-2,13,True,2.0):
        try: pid.command_packet(0,7,0,(ks,0,0,0,0,0))
        except ValueError: pass
        else: raise AssertionError('invalid FF coefficient accepted')
    for bank,payload in [(1,(2,0,0,0,0,0)),(0,(2,1,0,0,0,0))]:
        try: pid.command_packet(0,7,bank,payload)
        except ValueError: pass
        else: raise AssertionError('invalid FF bank/tail accepted')
    events=[];now=[0.0]
    link=pid.PIDLink(events.append,lambda:now[0])
    link.observe(frame())
    class Port:
        def write(self,data): return len(data)
    link.send_friction_ff(Port(),2);s=link.pending['sequence']
    link.observe(frame(s,1,0x80,(0,)*6,ms=101))
    assert link.pending
    link.observe(frame(s,2,0x80,(2,0,0,0,0,0),active=s,ms=102))
    assert link.completed['success'] and link.friction_ff_ks == 2 and link.board[0] == AUTO
    link.send_friction_ff(Port(),-1);s=link.pending['sequence']
    link.observe(frame(s,7,0x80,(2,0,0,0,0,0),active=s-1,ms=103))
    assert link.completed['success'] and link.board[0] == AUTO
    link.send_friction_ff(Port(),0);s=link.pending['sequence']
    link.observe(frame(s,2,0x80,(2,0,0,0,0,0),active=s,ms=104))
    assert not link.completed['success']  # an ACK with a wrong actual value is insufficient
    for result in (4,5,9):
        link.send_friction_ff(Port(),2);s=link.pending['sequence']
        link.observe(frame(s,result,0,active=s-1,ms=105+result))
        assert not link.completed['success']
    link.send_friction_ff(Port(),2);now[0]=3;link.check_timeout()
    assert link.friction_ff_ks is None and '未知' in link.message
    link.observe(frame(ms=1))
    assert link.friction_ff_ks is None
    link.observe(dict(frame(ms=2),firmware_id=pid.FRICTION_FF_FIRMWARE_ID))
    try:link.send_friction_ff(Port(),10)
    except ValueError as exc:assert '上限' in str(exc)
    else:raise AssertionError('old firmware accepted an extended strength')
    link.observe(dict(frame(ms=3),firmware_id=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID))
    for n,ks in enumerate((10,12,0)):
        link.send_friction_ff(Port(),ks);s=link.pending['sequence']
        link.observe(dict(frame(s,2,0x80,(ks,0,0,0,0,0),active=s,ms=100+n),
                          firmware_id=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID))
        assert link.completed['success'] and link.friction_ff_ks==ks and link.board[0]==AUTO
    print('PASS friction FF packets, signed query, range/bank/tail checks, tagged readback, PID bank isolation, ACK mismatch and timeout')


def test_log_and_worker():
    # Feed real encoded111-byte frames through the actual Reader command queue.
    from collections import deque
    import queue
    with tempfile.TemporaryDirectory() as temp:
        reader=logger.Reader('VIRTUAL','115200',queue.Queue());writes=[]
        initial=bytearray(packet());struct.pack_into('<I',initial,6,pid.FRICTION_FF_EXTENDED_FIRMWARE_ID)
        initial[-1]=pid.crc8(initial[:-1])
        class Serial:
            def __init__(self): self.chunks=deque([bytes(initial)]);self.empty=0
            @property
            def in_waiting(self): return len(self.chunks[0]) if self.chunks else 0
            def open(self): pass
            def close(self): pass
            def read(self,size):
                if self.chunks:return self.chunks.popleft()
                self.empty+=1
                if self.empty>4:reader.stop_event.set()
                return b''
            def write(self,data):
                writes.append(data);s=struct.unpack_from('<H',data,4)[0]
                assert data == pid.command_packet(s,7,0,(12,0,0,0,0,0))
                ack=bytearray(packet(s,2,0x80,(12,0,0,0,0,0),s,110))
                struct.pack_into('<I',ack,6,pid.FRICTION_FF_EXTENDED_FIRMWARE_ID)
                struct.pack_into('<H',ack,16,24)  # bit3=FF on, bit4=motor on
                struct.pack_into('<h',ack,54,-192)  # requested PWM; base is -180
                ack[-1]=pid.crc8(ack[:-1]);self.chunks.append(bytes(ack));return len(data)
        original=reader.post;seen=[0]
        def post(kind,value):
            original(kind,value)
            if kind=='frame':
                seen[0]+=1
                if seen[0]==1:reader.command_queue.put((7,0,(12,0,0,0,0,0)))
        reader.post=post
        factory=logger.Session
        with patch('serial.Serial',Serial),patch('auto_swing_logger.Session',side_effect=lambda p,b:factory(p,b,Path(temp)/'session')):
            reader.start();reader.join(15)
            if reader.is_alive():reader.stop_event.set();reader.join(5)
        assert not reader.is_alive() and len(writes)==1 and reader.session.total==2
        rows=list(csv.DictReader((reader.session.folder/'samples.csv').open(encoding='utf-8-sig')))
        assert rows[-1]['friction_ff_ks']=='12' and rows[-1]['friction_ff_enabled']=='1'
        assert rows[-1]['friction_ff_pwm']=='-12'
        events=[json.loads(x) for x in (reader.session.folder/'pid_commands.jsonl').read_text(encoding='utf-8').splitlines()]
        assert [e['event'] for e in events]==['send','ack'] and events[-1]['success']
    print('PASS friction FF actual serial worker queue, binary telemetry, CSV strength/effort and command ACK log')


def test_panel():
    from friction_ff_panel import FrictionFFPanel
    root=tk.Tk();root.withdraw();requests=[]
    panel=FrictionFFPanel(root,lambda x:requests.append(x) or True)
    try:
        assert str(panel.buttons['apply']['state'])=='disabled'
        panel.observe(dict(format='trace_v3_100hz',firmware_id=0x26101809))
        assert str(panel.buttons['apply']['state'])=='disabled'
        live=dict(format='trace_v3_100hz',firmware_id=pid.FRICTION_FF_FIRMWARE_ID,flags=0)
        panel.observe(live);panel.buttons['apply'].invoke();assert requests==[2]
        assert str(panel.buttons['off']['state'])=='disabled'
        panel.observe_link(dict(pending=False,pending_operation=None,message='已确认',friction_ff_ks=2))
        panel.buttons['read'].invoke();assert requests[-1]==-1
        panel.observe_link(dict(pending=False,pending_operation=None,message='已读取'))
        panel.buttons['off'].invoke();assert requests[-1]==0
        panel.observe_link(dict(pending=False,pending_operation=None,message='已关闭'))
        panel.input.set('99');panel.buttons['apply'].invoke();assert requests==[2,-1,0]
        panel.observe(dict(live,pid_command_bank=0x80,pid_command_result=7,pid_angle_kp=2,flags=24))
        assert '2 PWM' in panel.actual.get()
        panel.input.set('10');panel.buttons['apply'].invoke()
        assert requests==[2,-1,0] and '上限' in panel.message.get()
        panel.observe(dict(live,firmware_id=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID))
        assert panel.limit==12 and int(panel.strength.cget('to'))==12
        panel.buttons['apply'].invoke();assert requests[-1]==10
        panel.observe_link(dict(pending=False,pending_operation=None,message='已确认'))
        panel.observe(dict(live,firmware_id=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID,
                           pid_command_bank=0x80,pid_command_result=7,pid_angle_kp=12,flags=24))
        assert '12 PWM' in panel.actual.get()
        panel.observe(dict(live,firmware_id=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID,flags=8))
        assert '已启用' in panel.actual.get()  # motor off, FF still enabled
        panel.observe(dict(live,firmware_id=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID,flags=16))
        assert '已关闭' in panel.actual.get()  # motor on, FF disabled
        panel.last_seen=time.monotonic()-2;panel.refresh_buttons()
        assert str(panel.buttons['apply']['state'])=='disabled'
        panel.disconnect();assert all(str(x['state'])=='disabled' for x in panel.buttons.values())
    finally:root.destroy()
    print('PASS friction FF hidden Tk controls, old firmware/freshness/pending gating, apply/read/off, validation and disconnect')
    import queue, threading
    root=tk.Tk();root.withdraw();context=logger.create_window(root)
    class Reader:
        command_queue=queue.Queue()
        stop_event=threading.Event()
        def is_alive(self):return True
    try:
        context['reader']=Reader()
        panel=context['friction_panel']
        panel.observe(dict(format='trace_v3_100hz',firmware_id=pid.FRICTION_FF_FIRMWARE_ID,flags=0))
        panel.buttons['apply'].invoke()
        assert context['reader'].command_queue.get_nowait()==(7,0,(2,0,0,0,0,0))
        context['closed']=True
    finally:root.destroy()
    print('PASS friction FF main-window tab routes apply to the real shared serial command queue')


def test_independent_flags():
    """Real RTL uses bit3 for configuration, bit4 for instantaneous drive."""
    link=pid.PIDLink()
    assert pid.friction_ff_limit(pid.FRICTION_FF_SMOOTH_FIRMWARE_ID)==12
    assert '柔化前馈' in pid.friction_ff_profile(pid.FRICTION_FF_SMOOTH_FIRMWARE_ID)
    assert '对照版' in pid.friction_ff_profile(pid.FRICTION_FF_D_ONLY_FIRMWARE_ID)
    fw=pid.FRICTION_FF_EXTENDED_FIRMWARE_ID
    link.observe(dict(frame(1,7,0x80,(12,0,0,0,0,0),ms=100),firmware_id=fw,flags=8))
    assert link.friction_ff_ks==12
    link.observe(dict(frame(ms=110),firmware_id=fw,flags=8))
    assert link.friction_ff_ks==12  # no motor drive does not turn FF off
    link.observe(dict(frame(ms=120),firmware_id=fw,flags=16))
    assert link.friction_ff_ks==0   # active motor does not imply FF on
    base=logger.Decoder().feed(packet())[0]
    with tempfile.TemporaryDirectory() as temp:
        session=logger.Session('VIRTUAL',115200,Path(temp)/'flags')
        expected=[]
        try:
            for i,(ff_on,motor_on) in enumerate([(False,False),(False,True),(True,False),(True,True)]):
                flags=(8 if ff_on else 0)|(16 if motor_on else 0)
                data=dict(base,firmware_id=fw,sequence=i,fpga_ms=100+i*10,flags=flags,
                          control_pwm=20 if motor_on else 0,requested_pwm=20 if motor_on else 0)
                row=session.append(data);expected.append(int(ff_on))
                assert row['friction_ff_enabled']==int(ff_on)
                assert row.get('friction_ff_ks','')==('' if ff_on else 0)
        finally:session.close(logger.Decoder(),'test')
        rows=list(csv.DictReader((session.folder/'samples.csv').open(encoding='utf-8-sig')))
        assert [int(r['friction_ff_enabled']) for r in rows]==expected
    print('PASS actual FF bit3 independent of motor bit4 in link state and all four CSV combinations')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--fixtures-only',action='store_true');args=p.parse_args()
    fixtures()
    if not args.fixtures_only:test_protocol();test_log_and_worker();test_panel();test_independent_flags()
