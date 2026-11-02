"""Host protocol, acknowledgement, logging, serial worker and Tk integration."""
from collections import deque
import json
from pathlib import Path
import queue
import struct
import tempfile
import time
from unittest.mock import patch
import runtime_pid as pid
import auto_swing_logger as logger

ROOT=Path(__file__).resolve().parents[1]
AUTO=(1800,0,30000,1540,0,13050)
MAN=(1750,0,31500,1570,0,9879)
TUNED=(1800,9,16000,1500,17,12000)


def frame(sequence=0,result=0,bank=0,gains=AUTO,active=0,ms=100):
    return dict(format='trace_v3_100hz',fpga_ms=ms,pid_command_sequence=sequence,
                pid_command_result=result,pid_command_bank=bank,pid_active_sequence=active,
                **dict(zip(pid.READBACK_FIELDS,gains)))


def packet(sequence=0,result=0,bank=0,gains=AUTO,active=0,ms=100):
    base=(sequence,0x26100801,ms,0,0,3217,65535,-3,260,2961,2960,2958,2962,
          2960*256,-300*256,-256,-128,512,-180,-180,-25600,1280000)
    config=(220,65,95,2960,500,-1,0,2,*AUTO)
    data=logger.HEADER_V3+logger.PAYLOAD_V3.pack(*base,*config,sequence,active,result,bank,*gains)
    return data+bytes((pid.crc8(data),))


def fixtures():
    packets=[pid.command_packet(101,1,0,TUNED),pid.command_packet(102,2,0),
             pid.command_packet(103,1,1,(-32768,0,32767,-200,4,8000)),pid.command_packet(104,3,0),
             pid.command_packet(105,1,0,TUNED)]
    packets[-1]=packets[-1][:32]+bytes((packets[-1][32]^1,))+packets[-1][33:]
    # 0x26101802 adds real ops 4/5 and 0x26101804 adds op6, so the "unknown
    # operation" fixture uses8: op7 is bounded friction FF in0x26102201.
    for sequence,op,bank,values in ((106,1,0,(32768,0,0,0,0,0)),(107,8,0,TUNED),(108,1,2,TUNED)):
        body=struct.pack('<BBHBB6i',1,35,sequence,op,bank,*values)
        packets.append(b'\x5a\xa5'+body+bytes((pid.crc8(body),))+b'\r\n')
    bad_tail=pid.command_packet(109,1,0,TUNED)
    packets.extend((bad_tail[:-2]+b'\x00\n',pid.command_packet(110,2,1),pid.command_packet(111,3,1),
                    pid.command_packet(112,1,0,(1700,11,15000,1400,15,12500)),
                    pid.command_packet(113,1,0,(1900,8,14000,1300,12,11000))))
    (ROOT/'sim/pid_command_fixture.hex').write_text(''.join(f'{v:02x}\n' for p in packets for v in p),encoding='ascii')


def test_link():
    now=[0.0];events=[];writes=[]
    class Port:
        def write(self,data):writes.append(data);return len(data)
    link=pid.PIDLink(events.append,lambda:now[0])
    try:link.send(Port(),1,0,TUNED)
    except ValueError:pass
    else:raise AssertionError('disconnected send accepted')
    link.observe(frame(sequence=65535))
    link.send(Port(),1,0,TUNED)
    assert link.pending['sequence']==0 and len(writes[-1])==35
    assert writes[-1][32]==pid.crc8(writes[-1][2:32])
    try:link.send(Port(),1,0,TUNED)
    except ValueError:pass
    else:raise AssertionError('second in-flight command sent')
    link.observe(frame(sequence=999,result=2,active=999,gains=TUNED,ms=110))
    assert link.pending
    link.observe(frame(sequence=0,result=1,gains=AUTO,ms=120))
    assert link.pending
    link.observe(frame(sequence=0,result=2,active=0,gains=TUNED,ms=130))
    assert link.completed['success'] and not link.pending
    link.send(Port(),2,1)
    link.observe(frame(sequence=1,result=7,bank=1,gains=MAN,ms=140))
    assert link.completed['success'] and link.board[1]==MAN
    link.send(Port(),3,0)
    link.observe(frame(sequence=2,result=2,active=2,gains=AUTO,ms=150))
    assert link.completed['success']
    link.send(Port(),1,0,TUNED)
    link.observe(frame(sequence=3,result=2,active=3,gains=AUTO,ms=160))
    assert not link.completed['success']
    link.send(Port(),1,0,TUNED)
    link.observe(frame(sequence=4,result=3,active=3,gains=AUTO,ms=170))
    assert not link.completed['success']
    link.send(Port(),1,0,TUNED)
    now[0]=2.01;link.check_timeout()
    assert not link.pending and events[-1]['event']=='unconfirmed' and len(writes)==6
    link.observe(frame(ms=180))
    link.send(Port(),1,0,TUNED)
    link.observe(frame(ms=0))
    assert not link.pending and link.board=={0:AUTO} and '复位' in link.message
    link.observe(dict(format='trace_v2_100hz',fpga_ms=10))
    assert not link.fresh
    link.observe(frame(ms=20))
    class Partial:
        def write(self,data):return 5
    try:link.send(Partial(),1,0,TUNED)
    except ValueError:pass
    else:raise AssertionError('partial serial write accepted')
    assert events[-1]['event']=='unconfirmed' and '未知' in link.message
    for bad in ((32768,0,0,0,0,0),(-32769,0,0,0,0,0),(True,0,0,0,0,0),(1,2,3)):
        try:pid.validate_gains(bad)
        except ValueError:pass
        else:raise AssertionError('invalid gain accepted')
    print('PASS PID transactions, exact ACK, wrong sequence/values, CRC rejection, rollover, reset, timeout, partial write and legacy gating')


def test_decoder_worker():
    raw=packet()
    assert len(raw)==111
    decoder=logger.Decoder();rows=[]
    for b in raw:rows.extend(decoder.feed(bytes((b,))))
    assert len(rows)==1 and rows[0]['pid_angle_kp']==AUTO[0]
    damaged=bytearray(raw);damaged[85]^=2
    rows=decoder.feed(bytes(damaged)+b'noise'+raw)
    assert len(rows)==1 and decoder.bad_crc==1
    with tempfile.TemporaryDirectory() as temp:
        q=queue.Queue();reader=logger.Reader('VIRTUAL','115200',q)
        writes=[]
        class VirtualSerial:
            def __init__(self):self.chunks=deque((raw[:7],raw[7:]));self.empty=0
            @property
            def in_waiting(self):return len(self.chunks[0]) if self.chunks else 0
            def open(self):assert not self.dtr and not self.rts and self.baudrate==115200
            def read(self,size):
                if self.chunks:return self.chunks.popleft()
                self.empty+=1
                if self.empty>4:reader.stop_event.set()
                return b''
            def write(self,data):
                writes.append(data)
                seq=struct.unpack_from('<H',data,4)[0]
                if data[6]==4:
                    target,step=struct.unpack_from('<ii',data,8)
                    assert data==pid.command_packet(seq,4,0,(target,step,0,0,0,0))
                elif data[6]==6:
                    values=struct.unpack_from('<6i',data,8)
                    assert data==pid.command_packet(seq,6,0,values)
                else:
                    assert data==pid.command_packet(seq,1,0,TUNED)
                self.chunks.append(packet(seq,2,0,TUNED,seq,110))
                return len(data)
            def close(self):pass
        # Commands are queued only after complete current v3 frames arrive: op1
        # on the first frame, op4/op6 on the following ACK frames.
        reader.command_queue=queue.Queue()
        original_post=reader.post
        frames_seen=[0]
        def post(kind,value):
            original_post(kind,value)
            if kind=='frame':
                frames_seen[0]+=1
                if frames_seen[0]==1:reader.command_queue.put((1,0,TUNED))
                elif frames_seen[0]==2:reader.command_queue.put((4,0,(266,8,0,0,0,0)))
                elif frames_seen[0]==3:reader.command_queue.put((6,0,(5,1,1,1,0,0)))
        reader.post=post
        original_session=logger.Session
        with patch('serial.Serial',VirtualSerial),patch('auto_swing_logger.Session',
                side_effect=lambda p,b:original_session(p,b,Path(temp)/'session')):
            reader.start();reader.join(3)
        assert not reader.is_alive() and len(writes)==3 and reader.session.total==4
        folder=reader.session.folder
        events=[json.loads(line) for line in (folder/'pid_commands.jsonl').read_text(encoding='utf-8').splitlines()]
        assert [e['event'] for e in events]==['send','ack','send','ack','send','ack']
        assert events[1]['success'] and events[3]['success'] and events[5]['success']
        assert (events[2]['operation'],events[2]['target'],events[2]['step'])==(4,266,8)
        assert events[3]['active_sequence']==events[2]['sequence']
        assert (events[4]['operation'],events[4]['angle_deadband'],events[4]['position_deadband'],
                events[4]['motor_deadzone'],events[4]['d_filter_shift'])==(6,5,1,1,1)
        assert events[5]['active_sequence']==events[4]['sequence']
        assert not json.loads((folder/'session.json').read_text(encoding='utf-8'))['read_only']
        assert len(logger.Decoder().feed((folder/'serial_raw.bin').read_bytes()))==4
        assert 'pid_angle_kp' in (folder/'samples.csv').read_text(encoding='utf-8-sig').splitlines()[0]
    print('PASS v3 byte fragmentation, CRC resync, actual serial worker roundtrip (op1+op4+op6), '
          'raw/CSV/command/session logs')


def test_position_protocol():
    class Port:
        def __init__(self):self.writes=[]
        def write(self,data):self.writes.append(data);return len(data)
    packet4=pid.command_packet(7,4,0,(266,8,0,0,0,0))
    assert len(packet4)==35 and packet4[:4]==b'\x5a\xa5\x01\x23'
    assert packet4[4:6]==struct.pack('<H',7) and packet4[6]==4 and packet4[7]==0
    assert struct.unpack('<6i',packet4[8:32])==(266,8,0,0,0,0)
    assert packet4[32]==pid.crc8(packet4[2:32]) and packet4[33:]==b'\r\n'
    packet5=pid.command_packet(8,5,0)
    assert len(packet5)==35 and packet5[6]==5 and packet5[8:32]==bytes(24)
    assert struct.unpack('<6i',pid.command_packet(9,4,0,(-10630,64,0,0,0,0))[8:32])==(-10630,64,0,0,0,0)
    for operation,gains in ((4,(10631,8,0,0,0,0)),(4,(-10631,8,0,0,0,0)),(4,(0,65,0,0,0,0)),
                            (4,(0,-1,0,0,0,0)),(4,(0,8,1,0,0,0)),(4,(0,8,0,0,0,1)),
                            (4,(True,8,0,0,0,0)),(4,(0,True,0,0,0,0)),
                            (5,(0,0,0,0,0,1)),(5,(1,0,0,0,0,0)),(8,(0,)*6),(0,(0,)*6)):
        try:pid.command_packet(1,operation,0,gains)
        except ValueError:pass
        else:raise AssertionError(f'invalid op{operation} payload accepted')
    for operation in (4,5,6):
        try:pid.command_packet(1,operation,2,(0,)*6)
        except ValueError:pass
        else:raise AssertionError('position command with invalid bank accepted')
    for target,step in ((10631,0),(-10631,0),(0,65),(0,-1),(True,0),(0,True),(0.0,0)):
        try:pid.validate_position(target,step)
        except ValueError:pass
        else:raise AssertionError('invalid position payload accepted by validate_position')
    assert pid.validate_position(-10630,0)==(-10630,0) and pid.validate_position(10630,64)==(10630,64)
    assert pid.position_result_text(8)!=pid.position_result_text(4)!=pid.position_result_text(5)
    assert '平衡' in pid.position_result_text(8) and '固件' in pid.position_result_text(5)

    now=[0.0];events=[];port=Port()
    link=pid.PIDLink(events.append,lambda:now[0])
    try:link.send_position(port,266,8)
    except ValueError:pass
    else:raise AssertionError('disconnected position send accepted')
    link.observe(frame(sequence=65535))
    link.send_position(port,266,8)
    assert (link.pending['operation'],link.pending['target'],link.pending['step'])==(4,266,8)
    assert port.writes[-1]==pid.command_packet(0,4,0,(266,8,0,0,0,0))
    try:link.send_position_hold(port)
    except ValueError:pass
    else:raise AssertionError('second position command sent while one pending')
    # A frame for another sequence is ignored; a matching sequence whose
    # active_sequence is stale must be treated as a rejection, not success.
    link.observe(frame(sequence=999,result=2,active=999,gains=TUNED,ms=105))
    assert link.pending
    link.observe(frame(sequence=0,result=2,active=999,bank=1,gains=MAN,ms=106))
    assert not link.pending and not link.completed['success'] and link.completed['result']==2
    link.send_position(port,266,8)
    link.observe(frame(sequence=1,result=1,active=0,ms=110))
    assert link.pending and '等待控制周期' in link.message
    link.observe(frame(sequence=1,result=2,active=1,ms=120))
    assert link.completed['success'] and link.completed['operation']==4 and not link.pending
    assert link.last_position['target']==266 and '已生效' in link.message
    assert link.completed['active_sequence']==1
    # An error ACK repeats the sequence but never advances active_sequence.
    next_sequence=link.next_sequence
    for index,(result,needle) in enumerate(((8,'平衡'),(5,'固件'),(4,'范围'))):
        link.send_position(port,90,8)
        assert link.pending['sequence']==next_sequence+index
        link.observe(frame(sequence=link.pending['sequence'],result=result,active=0,ms=130+index))
        assert not link.completed['success'] and link.completed['result']==result
        assert needle in link.message,(result,link.message)
    link.send_position_hold(port)
    assert port.writes[-1]==pid.command_packet(link.pending['sequence'],5,0,(0,)*6)
    sequence=link.pending['sequence']
    link.observe(frame(sequence=sequence,result=2,active=sequence,ms=150))
    assert link.completed['success'] and link.completed['operation']==5
    snapshot=link.snapshot()
    assert snapshot['pending'] is False and snapshot['pending_operation'] is None
    assert snapshot['last_position']['operation']==4 and snapshot['fresh']
    link.send_position(port,0,8)
    now[0]=2.01;link.check_timeout()
    assert not link.pending and '超时' in link.message and events[-1]['event']=='unconfirmed'
    # Old firmware rejects op4 with result 5; a v3 link still records it.
    link.observe(frame(sequence=link.next_sequence,ms=200))
    link.send_position(port,266,0)
    sequence=link.pending['sequence']
    link.observe(frame(sequence=sequence,result=5,active=sequence-1,ms=210))
    assert not link.completed['success'] and '0x26101802' in link.message
    # PID commands still work after position traffic (backward compatibility).
    link.send(port,1,0,TUNED)
    sequence=link.pending['sequence']
    link.observe(frame(sequence=sequence,result=2,active=sequence,gains=TUNED,ms=220))
    assert link.completed['success'] and link.completed['operation']==1
    print('PASS op4/op5 packet bytes+CRC, payload/bank validation, ACK (result 2 + active sequence), '
          'result 4/5/8 messages, wrong sequence/bank, result 1 hold, one-pending rule, timeout, '
          'old-firmware rejection and op1-3 backward compatibility')


def test_position_helpers():
    from position_panel import (degrees_to_counts,counts_to_degrees,format_degrees,
                                validate_target_degrees,validate_step,target_preview,
                                estimate_move_seconds)
    assert degrees_to_counts(90)==266 and degrees_to_counts(-90)==-266
    assert degrees_to_counts(360)==1063 and degrees_to_counts(0)==0
    assert counts_to_degrees(1063)==360 and counts_to_degrees(-1063)==-360
    assert format_degrees(1063)=='+360.0°' and format_degrees(-1063)=='-360.0°'
    assert validate_target_degrees('90')==(266,90.0)
    assert validate_target_degrees('-3600')==(-10630,-3600.0)
    assert validate_target_degrees('3600')==(10630,3600.0)
    for text in ('abc','',None,'nan','inf','3610',3601.0):
        try:validate_target_degrees(text)
        except ValueError:pass
        else:raise AssertionError(f'invalid target {text!r} accepted')
    assert validate_step('8')==8 and validate_step(1)==1 and validate_step('64')==64
    for text in ('0','65','-1','abc','8.5','',None):
        try:validate_step(text)
        except ValueError:pass
        else:raise AssertionError(f'invalid speed peak {text!r} accepted')
    assert target_preview('90')=='= +266 计数' and target_preview('abc')=='= — 计数'
    assert '超出' in target_preview('3610') and target_preview('-360')=='= -1063 计数'
    # Trapezoid time estimate: ramp ticks 2*Vmax, cruise = max(0,(|d|-Vmax*(Vmax+1))/Vmax).
    assert estimate_move_seconds(0,8)==0.0
    assert abs(estimate_move_seconds(266,8)-2.0125)<1e-12
    assert estimate_move_seconds(-266,8)==estimate_move_seconds(266,8)
    assert estimate_move_seconds(72,8)==0.8
    assert abs(estimate_move_seconds(266,0)-estimate_move_seconds(266,8))<1e-12
    assert estimate_move_seconds(266,65) is None and estimate_move_seconds(266,0.0) is None
    print('PASS position panel degrees<->counts conversion (1063 counts/rev), ±10630 limits, '
          'speed-peak 1..64, live preview and trapezoid time estimate helpers')


def test_position_panel_smoke():
    try:
        import tkinter as tk
        from position_panel import PositionPanel
        root=tk.Tk();root.withdraw()
    except Exception as exc:
        print('SKIP position panel GUI smoke test (Tk unavailable):',type(exc).__name__,exc)
        return
    try:
        class Port:
            def write(self,data):return len(data)
        now=[1000.0]
        requests=[]
        panel=PositionPanel(root,lambda *args: requests.append(args) or True)
        panel.clock=lambda: now[0]
        root.update()
        assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
        assert panel.precision_var.get() is True
        assert '等待遥测' in panel.arrive_var.get()
        telemetry=dict(format='trace_v3_100hz',arm_relative_counts=0,position_target_counts=-5,state=4)
        panel.observe(telemetry);root.update()
        # Precision mode blocks op4 while moving, but "保持当前位置" (op5) stays
        # available as an emergency stop.
        assert str(panel.buttons['hold']['state'])=='normal'
        assert all(str(panel.buttons[key]['state'])=='disabled'
                   for key in panel.buttons if key!='hold')
        assert '移动中' in panel.arrive_var.get() and '未到位禁止下发' in panel.arrive_var.get()
        # |target-arm| <= 3 counts held for >= 1 s counts as arrived.
        aligned=dict(telemetry,position_target_counts=0)
        panel.observe(aligned)
        now[0]+=1.01
        panel.observe(aligned)
        assert panel.arrived and '到位' in panel.arrive_var.get()
        assert all(str(b['state'])=='normal' for b in panel.buttons.values())
        assert '平衡' in panel.current_var.get()
        panel.target_var.set('90')
        assert panel.preview_var.get()=='= +266 计数'
        assert '估算' in panel.estimate_var.get() and '2.0' in panel.estimate_var.get()
        assert panel.guide_var.get().startswith('速度曲线：加速→匀速→减速（梯形），无超调')
        # Relative buttons anchor the commanded TARGET, not the measured arm.
        panel.precision_var.set(False);panel.refresh_buttons()
        panel.observe(dict(telemetry,arm_relative_counts=100,position_target_counts=13,state=4))
        panel.buttons['rel+90'].invoke()
        assert requests[-1]==(4,279,8) and panel.target_var.get()=='94.5'
        assert '当前目标 +13' in panel.message.get()
        panel.queued=False;panel.refresh_buttons()
        panel.observe(dict(telemetry,arm_relative_counts=700,position_target_counts=500,state=4))
        panel.buttons['rel-90'].invoke()
        assert requests[-1]==(4,234,8),(requests[-1],panel.message.get())   # target-based, not 700-266
        panel.queued=False;panel.refresh_buttons()
        panel.relative_var.set('45')
        panel.buttons['rel-custom'].invoke()
        assert requests[-1]==(4,633,8)                                      # 500 + round(45*1063/360)
        panel.queued=False;panel.refresh_buttons()
        # Precision mode (default on): op4 blocked while moving, op5 hold still allowed.
        panel.precision_var.set(True);panel.refresh_buttons()
        assert str(panel.buttons['goto']['state'])=='disabled'
        assert str(panel.buttons['rel-custom']['state'])=='disabled'
        assert str(panel.buttons['hold']['state'])=='normal'
        assert '未到位禁止下发' in panel.arrive_var.get()
        before=len(requests)
        assert panel.send(4,500) is False and '尚未到位' in panel.message.get()
        assert len(requests)==before
        panel.buttons['hold'].invoke()
        assert requests[-1]==(5,0,0) and len(requests)==before+1
        panel.queued=False
        arrived=dict(telemetry,arm_relative_counts=500,position_target_counts=500,state=4)
        panel.observe(arrived);now[0]+=1.01;panel.observe(arrived)
        assert panel.arrived and '到位' in panel.arrive_var.get()
        assert all(str(b['state'])=='normal' for b in panel.buttons.values())
        # Input validation still rejects out-of-range target/step.
        panel.target_var.set('99999');panel.buttons['goto'].invoke()
        assert '未发送' in panel.message.get()
        panel.target_var.set('0');panel.step_var.set('0');panel.buttons['goto'].invoke()
        assert '未发送' in panel.message.get()
        panel.step_var.set('8')
        panel.buttons['zero'].invoke()
        assert requests[-1]==(4,0,8)
        panel.queued=False;panel.refresh_buttons()
        zeros=dict(telemetry,arm_relative_counts=0,position_target_counts=0,state=4)
        panel.observe(zeros);now[0]+=1.01;panel.observe(zeros)
        panel.buttons['rel+90'].invoke()
        assert requests[-1]==(4,266,8) and panel.target_var.get()=='90.1'
        panel.queued=False
        # Link ACK/reject messages and state plumbing.
        link=pid.PIDLink();link.observe(frame(sequence=65535))
        panel.queued=False;panel.observe_link(link.snapshot())
        link.send_position(Port(),266,8);sequence=link.pending['sequence']
        panel.observe_link(link.snapshot())
        assert '发送中' in panel.message.get()
        link.observe(frame(sequence=sequence,result=2,active=sequence,ms=110))
        panel.observe_link(link.snapshot())
        assert '已生效' in panel.message.get() and '90.1' in panel.message.get()
        link.send_position_hold(Port());sequence=link.pending['sequence']
        link.observe(frame(sequence=sequence,result=8,active=sequence-1,ms=120))
        panel.observe_link(link.snapshot())
        assert '平衡' in panel.message.get() and '拒绝' in panel.message.get()
        # Typed values survive telemetry refresh and disconnect.
        panel.target_var.set('123');panel.step_var.set('16')
        panel.observe(dict(telemetry,arm_relative_counts=1,position_target_counts=2,state=4))
        assert panel.target_var.get()=='123' and panel.step_var.get()=='16'
        panel.disconnect()
        assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
        assert panel.target_var.get()=='123' and panel.arrived is False
        assert '未连接' in panel.arrive_var.get()
        root.update()
    finally:
        root.destroy()
    print('PASS hidden Tk position panel: fresh-telemetry gating, target-based relative buttons '
          '(13+266=279, mismatch case), custom relative, arrival detection (|Δ|<=3 for 1 s), '
          'precision-mode gating (op4 blocked, op5 allowed), ACK/reject messages, edit preservation, '
          'disconnect')


def test_tuning_protocol():
    class Port:
        def __init__(self):self.writes=[]
        def write(self,data):self.writes.append(data);return len(data)
    packet6=pid.command_packet(3,6,0,(5,1,1,1,0,0))
    assert len(packet6)==35 and packet6[:4]==b'\x5a\xa5\x01\x23'
    assert packet6[4:6]==struct.pack('<H',3) and packet6[6]==6
    assert struct.unpack('<6i',packet6[8:32])==(5,1,1,1,0,0)
    assert packet6[32]==pid.crc8(packet6[2:32]) and packet6[33:]==b'\r\n'
    assert struct.unpack('<6i',pid.command_packet(4,6,0,(-1,-1,-1,-1,0,0))[8:32])==(-1,-1,-1,-1,0,0)
    for values in ((21,0,0,0,0,0),(0,21,0,0,0,0),(0,0,11,0,0,0),(0,0,0,5,0,0),
                   (-2,0,0,0,0,0),(0,-2,0,0,0,0),(0,0,-2,0,0,0),(0,0,0,-2,0,0),
                   (5,1,1,1,1,0),(5,1,1,1,0,1),(True,0,0,0,0,0),(0,0,0,1.0,0,0)):
        try:pid.command_packet(1,6,0,values)
        except ValueError:pass
        else:raise AssertionError(f'invalid op6 payload {values!r} accepted')
    try:pid.command_packet(1,6,2,(-1,-1,-1,-1,0,0))
    except ValueError:pass
    else:raise AssertionError('op6 with invalid bank accepted')
    for bad in ((21,0,0,0),(0,21,0,0),(0,0,11,0),(0,0,0,5),(-2,0,0,0),(True,0,0,0),(0,1.0,0,0)):
        try:pid.validate_tuning(*bad)
        except ValueError:pass
        else:raise AssertionError(f'invalid tuning {bad!r} accepted')
    for good in ((-1,-1,-1,-1),(20,20,10,4),(0,0,0,0),(5,1,1,1)):
        assert pid.validate_tuning(*good)==good
    assert '范围' in pid.tuning_result_text(4) and '固件' in pid.tuning_result_text(5)
    assert '0x26101804' in pid.tuning_result_text(5)

    now=[0.0];events=[];port=Port()
    link=pid.PIDLink(events.append,lambda:now[0])
    try:link.send_tuning(port,5,1,1,1)
    except ValueError:pass
    else:raise AssertionError('disconnected tuning send accepted')
    link.observe(frame(sequence=65535))
    link.send_tuning(port,5,1,1,1)
    assert (link.pending['operation'],link.pending['angle_deadband'],link.pending['position_deadband'],
            link.pending['motor_deadzone'],link.pending['d_filter_shift'])==(6,5,1,1,1)
    assert port.writes[-1]==pid.command_packet(0,6,0,(5,1,1,1,0,0))
    try:link.send_position(port,0)
    except ValueError:pass
    else:raise AssertionError('second command accepted while tuning pending')
    link.observe(frame(sequence=0,result=1,active=0,ms=105))
    assert link.pending and '等待控制周期' in link.message
    link.observe(frame(sequence=0,result=2,active=0,ms=110))
    assert link.completed['success'] and link.completed['operation']==6 and not link.pending
    assert link.last_tuning==dict(angle_deadband=5,position_deadband=1,motor_deadzone=1,d_filter_shift=1)
    assert '已生效' in link.message and '角度死区' in link.message
    snapshot=link.snapshot()
    assert snapshot['last_tuning']['angle_deadband']==5 and snapshot['pending_operation'] is None
    # result 4 = parameter rejected, result 5 = old firmware; errors echo seq only.
    next_sequence=link.next_sequence
    for index,(result,needle) in enumerate(((4,'范围'),(5,'固件'))):
        link.send_tuning(port,-1,-1,-1,-1)
        assert link.pending['sequence']==next_sequence+index
        link.observe(frame(sequence=link.pending['sequence'],result=result,active=0,ms=120+index))
        assert not link.completed['success'] and needle in link.message,(result,link.message)
    assert link.last_tuning['angle_deadband']==5  # failed commands do not alter overrides
    link.send_tuning(port,0,0,0,0)
    now[0]=2.01;link.check_timeout()
    assert not link.pending and '超时' in link.message and events[-1]['event']=='unconfirmed'
    # op3 (restore defaults) clears the runtime overrides on the board.
    link.observe(frame(sequence=link.next_sequence,ms=200))
    link.send(port,3,0)
    sequence=link.pending['sequence']
    link.observe(frame(sequence=sequence,result=2,active=sequence,gains=AUTO,ms=210))
    assert link.completed['success'] and '已同时清除' in link.message
    assert link.snapshot()['last_tuning'] is None
    # A successful tuning set is cleared by an FPGA reset as well.
    link.send_tuning(port,5,1,1,1)
    sequence=link.pending['sequence']
    link.observe(frame(sequence=sequence,result=2,active=sequence,ms=220))
    assert link.last_tuning
    link.observe(frame(sequence=sequence,ms=0))
    assert link.last_tuning is None and '复位' in link.message
    print('PASS op6 packet bytes+CRC, per-field range/zero-tail validation, ACK (result 2 + active '
          'sequence), result 4/5 messages, one-pending rule, timeout, op3/reset clearing last_tuning '
          'and snapshot compatibility')


def test_jitter_helpers():
    from jitter_panel import parse_tuning_entries, describe_tuning
    assert parse_tuning_entries(('-1','0','3','1'))==(-1,0,3,1)
    assert parse_tuning_entries((20,20,10,4))==(20,20,10,4)
    for texts in (('21','0','0','0'),('0','21','0','0'),('0','0','11','0'),('0','0','0','5'),
                  ('-2','0','0','0'),('abc','0','0','0'),('0','','0','0')):
        try:parse_tuning_entries(texts)
        except ValueError as exc:
            assert '范围' in str(exc) or '整数' in str(exc),str(exc)
        else:raise AssertionError(f'invalid tuning entries {texts!r} accepted')
    assert '固件默认' in describe_tuning(None)
    summary=describe_tuning(dict(angle_deadband=-1,position_deadband=1,
                                 motor_deadzone=2,d_filter_shift=3))
    assert '角度死区 -1（默认）' in summary and 'D 滤波移位 3' in summary
    print('PASS jitter panel entry parsing, -1 default semantics and tuning summary helpers')


def test_jitter_panel_smoke():
    try:
        import tkinter as tk
        from jitter_panel import JitterPanel
        root=tk.Tk();root.withdraw()
    except Exception as exc:
        print('SKIP jitter panel GUI smoke test (Tk unavailable):',type(exc).__name__,exc)
        return
    try:
        requests=[]
        panel=JitterPanel(root,lambda *args: requests.append(args) or True)
        root.update()
        assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
        telemetry=dict(format='trace_v3_100hz',firmware_id=0x26101804,state=4)
        panel.observe(telemetry);root.update()
        assert all(str(b['state'])=='normal' for b in panel.buttons.values())
        assert '0x26101804' in panel.compat_var.get() and '支持' in panel.compat_var.get()
        panel.observe(dict(telemetry,firmware_id=0x26101803))
        assert '不支持' in panel.compat_var.get()
        assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
        panel.observe(dict(telemetry,firmware_id=0x26102201))
        assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
        panel.observe(telemetry)
        panel.buttons['apply'].invoke()
        assert requests[-1]==(-1,-1,-1,-1)
        panel.queued=False;panel.refresh_buttons()
        for var,value in zip(panel.inputs,('5','1','1','1')):var.set(value)
        panel.buttons['apply'].invoke()
        assert requests[-1]==(5,1,1,1)
        panel.queued=False;panel.refresh_buttons()
        panel.buttons['restore'].invoke()
        assert requests[-1]==(-1,-1,-1,-1)
        assert [v.get() for v in panel.inputs]==['-1','-1','-1','-1']
        panel.queued=False;panel.refresh_buttons()
        panel.inputs[0].set('21');panel.buttons['apply'].invoke()
        assert '未发送' in panel.message.get() and len(requests)==3
        panel.inputs[0].set('5')
        class Port:
            def write(self,data):return len(data)
        link=pid.PIDLink();link.observe(frame(sequence=65535))
        panel.queued=False;panel.observe_link(link.snapshot())
        link.send_tuning(Port(),5,1,1,1);sequence=link.pending['sequence']
        panel.observe_link(link.snapshot())
        assert '发送中' in panel.message.get()
        link.observe(frame(sequence=sequence,result=2,active=sequence,ms=110))
        panel.observe_link(link.snapshot())
        assert '已生效' in panel.message.get() and '角度死区 5' in panel.message.get()
        assert '角度死区 5' in panel.info_var.get()
        link.send_tuning(Port(),-1,-1,-1,-1);sequence=link.pending['sequence']
        link.observe(frame(sequence=sequence,result=5,active=sequence-1,ms=120))
        panel.observe_link(link.snapshot())
        assert '固件不支持' in panel.message.get()
        link.send(Port(),3,0);sequence=link.pending['sequence']
        link.observe(frame(sequence=sequence,result=2,active=sequence,gains=AUTO,ms=130))
        panel.observe_link(link.snapshot())
        assert '已恢复默认' in panel.message.get() and '固件默认' in panel.info_var.get()
        panel.disconnect()
        assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
        root.update()
    finally:
        root.destroy()
    print('PASS hidden Tk jitter panel: firmware-compat hint, fresh-telemetry gating, apply/restore '
          'payloads, range validation, ACK/reject messages, op3 clears display, disconnect')


def test_ui():
    import tkinter as tk
    from pid_panel import PIDPanel
    root=tk.Tk();root.withdraw()
    context=logger.create_window(root)
    panel=context['pid_panel'];root.update()
    position_panel=context['position_panel'];root.update()
    jitter_panel=context['jitter_panel'];root.update()
    assert str(position_panel.buttons['goto']['state'])=='disabled'
    assert str(jitter_panel.buttons['apply']['state'])=='disabled'
    # Precision-mode gating itself is covered by test_position_panel_smoke; here
    # only the main-window wiring is checked, so disable it for the enable check.
    position_panel.precision_var.set(False)
    position_panel.observe(dict(format='trace_v3_100hz',arm_relative_counts=0,position_target_counts=0,state=4))
    jitter_panel.observe(dict(format='trace_v3_100hz',firmware_id=0x26101804,state=4))
    root.update()
    assert str(position_panel.buttons['goto']['state'])=='normal'
    assert str(jitter_panel.buttons['apply']['state'])=='normal'
    snapshots=[]
    link=pid.PIDLink()
    link.observe(frame());panel.observe(link.snapshot());root.update()
    assert [v.get() for v in panel.inputs[0]]==list(map(str,AUTO))
    assert str(panel.buttons[1]['state'])=='normal'
    panel.inputs[0][0].set('1999');link.observe(frame(ms=110));panel.observe(link.snapshot())
    assert panel.inputs[0][0].get()=='1999'
    panel.bank.set('手动');panel.select_bank()
    assert str(panel.buttons[1]['state'])=='disabled'
    requests=[];panel.submit=lambda *args:requests.append(args) or True
    panel.buttons[2].invoke();assert requests==[(2,1,(0,)*6)]
    class Port:
        def write(self,data):return len(data)
    link.send(Port(),2,1);seq=link.pending['sequence']
    link.observe(frame(seq,7,1,MAN,ms=120));panel.observe(link.snapshot())
    assert [v.get() for v in panel.inputs[1]]==list(map(str,MAN))
    panel.inputs[1][0].set('40000');panel.buttons[1].invoke()
    assert len(requests)==1 and '未发送' in panel.message.get()
    panel.inputs[1][0].set('2000');panel.buttons[1].invoke()
    assert requests[-1]==(1,1,(2000,*MAN[1:]))
    panel.disconnect();assert all(str(b['state'])=='disabled' for b in panel.buttons.values())
    assert panel.inputs[0][0].get()=='1999'
    position_panel.disconnect()
    jitter_panel.disconnect()
    root.update()
    context['closed']=True;root.destroy()
    print('PASS hidden Tk main window, six-entry PID panel, position panel and jitter-tuning panel '
          'wiring, bank selection, read/set buttons, input validation, edit preservation and '
          'disconnect gating')


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--fixtures-only',action='store_true');parser.add_argument('--ui',action='store_true')
    args=parser.parse_args();fixtures()
    if not args.fixtures_only:
        test_link();test_decoder_worker()
        test_position_protocol();test_position_helpers();test_position_panel_smoke()
        test_tuning_protocol();test_jitter_helpers();test_jitter_panel_smoke()
        if args.ui:test_ui()
