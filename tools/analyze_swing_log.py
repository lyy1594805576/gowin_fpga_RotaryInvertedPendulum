"""Analyze saved trace_v1 records without opening a serial port.

Uses FPGA timestamps, checks every CSV payload against CRC-checked raw data,
and saves measured statistics and plots. PID term reconstruction uses explicitly
supplied local gains; it is not parameter readback or a plant simulation.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from auto_swing_logger import Decoder, CONFIG_FIELDS


def stats(x):
    return {k: float(v) for k, v in zip(
        ('mean', 'std', 'min', 'max', 'p05', 'p95'),
        (np.mean(x), np.std(x), np.min(x), np.max(x),
         *np.percentile(x, (5, 95))))}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory', type=Path)
    ap.add_argument('--angle-gains', type=int, nargs=3, default=(1550, 12, 31600))
    ap.add_argument('--position-gains', type=int, nargs=3, default=(940, 16, 9950))
    args = ap.parse_args()
    folder = args.directory.resolve()
    with (folder/'samples.csv').open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows or any(r['format'] not in ('trace_v1_100hz','trace_v2_100hz') for r in rows):
        raise ValueError('Expected nonempty 100 Hz trace CSV')
    raw = (folder/'serial_raw.bin').read_bytes()
    decoder = Decoder()
    decoded = decoder.feed(raw)
    assert len(decoded) == len(rows), 'Raw / CSV frame count mismatch'
    for index, (frame, row) in enumerate(zip(decoded, rows)):
        assert all(value == int(row[k]) for k,value in frame.items() if k != 'format'), f'Payload mismatch at row {index}'
    a = {k: np.array([float(r[k]) for r in rows])
         for k in rows[0] if k not in ('host_time', 'format') and all(r[k] != '' for r in rows)}
    assert len(np.unique(a['epoch'])) == 1, 'Analyze epochs separately after a reset'
    assert np.all(np.diff(a['fpga_ms']) == 10), 'Expected continuous 10 ms FPGA time'
    assert np.all((np.diff(a['sequence']).astype(int) & 65535) == 1)
    t = (a['fpga_ms']-a['fpga_ms'][0])/1000
    bal = a['state'] == 4
    if not np.any(bal):
        raise ValueError('No balance interval in this recording')
    first = int(np.flatnonzero(bal)[0])
    # Scope this analysis to the first continuous capture; do not splice retries.
    after = np.flatnonzero(~bal[first:])
    end = first+int(after[0]) if len(after) else len(rows)
    bal[end:] = False
    start = t[first]
    stable = bal & (t >= start+5)
    flags = a['flags'].astype(int)
    active_ki = np.full(len(rows), args.angle_gains[1], dtype=float)
    known_ki = np.isin(a['firmware_id'], (0x26100504,0x26100505)) & ((flags & 128) != 0)
    active_ki[known_ki] = flags[known_ki] >> 8
    if np.any(np.isin(a['firmware_id'], (0x26100504,0x26100505))):
        # Compare after the recorded ramp completes, rather than mixing gains.
        at_steady = bal & ((flags & 64) != 0)
        stable = bal & (t >= t[np.flatnonzero(at_steady)[0]]+3) if np.any(at_steady) else np.zeros(len(rows),dtype=bool)
    settled_available = np.count_nonzero(stable) >= 200
    # Failed capture is an analyzable outcome. Never estimate a steady hunting
    # frequency from its sub-second transient or silently join later retries.
    selected = stable if settled_available else bal
    def parameter(name,default):
        return a.get(name,np.full(len(rows),default,dtype=float))
    active_ki = parameter('angle_ki',active_ki)
    if 'angle_ki' in a: known_ki = np.ones(len(rows),dtype=bool)
    center_counts = parameter('angle_center',3017)
    angle_error = a['angle_error_q8']/256
    angle_db = parameter('angle_deadband',0)
    angle_error = np.sign(angle_error)*np.maximum(np.abs(angle_error)-angle_db,0)
    reconstructed_angle = np.array([
        angle_error*parameter('angle_kp',args.angle_gains[0])/4096,
        a['angle_integral_q8']/256*active_ki/4096,
        a['d_angle_q8']/256*parameter('angle_kd',args.angle_gains[2])/4096])*2611/1024
    pos_error = parameter('encoder_direction',-1)*(a['position_target_counts']-a['arm_relative_counts'])
    pos_error = np.sign(pos_error)*np.maximum(np.abs(pos_error)-parameter('position_deadband',2), 0)
    reconstructed_pos = np.array([
        pos_error*parameter('position_kp',args.position_gains[0])/4096,
        a['position_integral_q8']/256*parameter('position_ki',args.position_gains[1])/4096,
        a['d_position_q8']/256*parameter('position_kd',args.position_gains[2])/4096])
    n = int(np.count_nonzero(selected))
    window = np.hanning(n)
    freq = np.fft.rfftfreq(n, .01)
    spectra = {}
    for key in ('arm_relative_counts', 'theta_filtered', 'angle_target_counts', 'requested_pwm'):
        x = a[key][selected]
        spectra[key] = np.fft.rfft((x-np.mean(x))*window)
    candidates = np.flatnonzero((freq >= .1) & (freq <= 5))
    peak = int(candidates[np.argmax(np.abs(spectra['arm_relative_counts'][candidates]))]) if settled_available else None
    rmse_samples = selected & ((flags & 1) != 0) & (t > start)
    summary = {
        'input_sha256': {name: hashlib.sha256((folder/name).read_bytes()).hexdigest()
                         for name in ('samples.csv', 'events.csv', 'serial_raw.bin', 'session.json')},
        'integrity': {'frames': len(rows), 'raw_csv_all_payloads_match': True,
                      'bad_crc': decoder.bad_crc, 'missing_sequences': 0,
                      'discarded_prefix_or_resync_bytes': decoder.discarded_bytes,
                      'uncompleted_last_packet_bytes': len(decoder.buffer), 'fpga_period_ms': 10},
        'firmware_ids': sorted({f'0x{int(x):08X}' for x in a['firmware_id']}),
        'board_configuration_readback': {k:sorted({int(x) for x in a[k]}) for k in CONFIG_FIELDS if k in a},
        'modes': sorted({int(x) for x in a['manual_mode']}),
        'first_balance_start_s': float(start),
        'first_balance_observed_span_s': float(t[end-1]-start),
        'first_balance_frames': end-first,
        'first_balance_exit_s': float(t[end]) if end<len(rows) else None,
        'record_ends_in_first_balance': end == len(rows),
        'settled_comparison_available': bool(settled_available),
        'ki_reduced_frames': int(np.count_nonzero(flags[bal]&32)),
        'first_balance_stats': {k: stats(a[k][bal]) for k in
                          ('theta_filtered','arm_relative_counts','angle_target_counts','requested_pwm')},
        'settled_window': {'start_s': float(t[selected][0]), 'end_s': float(t[selected][-1]), 'frames': n} if settled_available else None,
        'settled_stats': {k: stats(a[k][selected]) for k in
                          ('theta_raw', 'theta_filtered', 'arm_relative_counts', 'position_target_counts',
                           'angle_target_counts', 'position_output_counts', 'requested_pwm')} if settled_available else None,
        'dominant_arm_frequency_hz': float(freq[peak]) if settled_available else None,
        'dominant_period_s': float(1/freq[peak]) if settled_available else None,
        'dominant_amplitude_counts': {k: float(2*abs(v[peak])/window.sum()) for k,v in spectra.items()} if settled_available else None,
        'settled_flag_counts': {str(bit): int(np.count_nonzero(flags[selected] & (1<<bit))) for bit in range(5)} if settled_available else None,
        'first_balance_flag_counts': {str(bit): int(np.count_nonzero(flags[bal] & (1<<bit))) for bit in range(8)},
        'observed_swing_pwm_magnitudes': sorted({int(abs(x)) for x in a['requested_pwm'][np.isin(a['state'],(22,24,32,34))] if x != 0}),
        'angle_integral_at_limit_first_2s_frames': int(np.count_nonzero(
            bal & (t<start+2) & (np.abs(a['angle_integral_q8']) == 2560000))),
        'local_assumed_gains_not_board_readback': {'angle': args.angle_gains, 'position': args.position_gains},
        'term_statistics_scope': 'settled' if settled_available else 'first_capture_transient',
        'angle_terms_pwm_stats': {name: stats(x[selected]) for name,x in zip(('P','I','D'),reconstructed_angle)},
        'active_ki_readback_frames': int(np.count_nonzero(known_ki)),
        'observed_active_ki': sorted({int(x) for x in active_ki[bal]}),
        'settled_active_ki': sorted({int(x) for x in active_ki[selected]}) if settled_available else None,
        'angle_integral_dominant_pwm_amplitude': float(2*abs(np.fft.rfft(
            (reconstructed_angle[1][selected]-np.mean(reconstructed_angle[1][selected]))*window)[peak])/window.sum()) if settled_available else None,
        'position_terms_counts_stats': {name: stats(x[selected]) for name,x in zip(('P','I','D'),reconstructed_pos)},
        'angle_reconstruction_pwm_rmse': float(np.sqrt(np.mean(
            (np.clip(reconstructed_angle.sum(axis=0)[rmse_samples],-255,255)-a['control_pwm'][rmse_samples])**2))) if np.any(rmse_samples) else None,
        'reconstruction_scope': 'valid in-range samples excluding first capture snapshot; no PID is run while out of range',
        'raw_minus_filtered_counts_stats': stats((a['theta_raw']-a['theta_filtered'])[selected]),
        'limitations': ['100 Hz snapshots cannot reconstruct every 3 ms control sample.',
                       'Raw ADC is already a 16-sample mean, not electrical sample noise.',
                       'BAL state is not proof of hands-free operation.',
                       'PID term reconstruction is approximate and based on assumed local gains.',
                       'A closed-loop trace cannot uniquely identify plant dynamics or prove a gain change fixes the hardware.']}
    out = folder/'analysis'
    out.mkdir(exist_ok=True)
    (out/'statistics.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False,
                         'font.size': 10, 'figure.dpi': 140})
    fig, axes = plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
    axes[0].plot(t,a['theta_filtered'],label='摆杆角度')
    axes[0].plot(t,a['angle_target_counts'],label='位置环给出的角度目标',lw=.9)
    axes[0].plot(t,center_counts,color='gray',ls='--',lw=.8,label='标定平衡点')
    axes[0].set_ylabel('角度 / ADC计数'); axes[0].legend(ncol=3,loc='lower right')
    axes[1].plot(t,a['arm_session_counts'],label='相对记录起点的转臂位移')
    axes[1].set_ylabel('编码器计数'); axes[1].legend(loc='upper right')
    axes[2].plot(t,a['requested_pwm'],lw=.8,label='死区后的 PWM 请求')
    axes[2].set_ylabel('PWM / ±255'); axes[2].legend(loc='upper right')
    axes[3].plot(t[bal],a['angle_integral_q8'][bal]/256,label='角度积分')
    axes[3].plot(t[bal],a['position_integral_q8'][bal]/256,label='位置积分')
    axes[3].set_ylabel('积分累加 / 计数'); axes[3].set_xlabel('FPGA时间 / 秒（记录首帧为0）')
    axes[3].legend(ncol=2,loc='upper right')
    for ax in axes:
        ax.axvline(start,color='tab:red',ls='--',lw=1)
        ax.grid(alpha=.2); ax.set_xlim(t[0],t[-1])
    result_text = f'持续往返（主频{freq[peak]:.3f}Hz）' if settled_available else f'平衡仅{(end-first)*.01:.2f}s，未获得稳态比较'
    fig.suptitle(f'自动起摆记录：约{start:.2f}s进入平衡，{result_text}')
    fig.savefig(out/'overview.png'); plt.close(fig)
    zoom = stable & (t>=t[end-1]-8) if settled_available else (t>=start) & (t<=t[min(end,len(t)-1)])
    term_zoom = zoom & bal
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
    axes[0].plot(t[zoom],a['theta_filtered'][zoom]-center_counts[zoom],label='摆杆相对标定点')
    axes[0].plot(t[zoom],a['angle_target_counts'][zoom]-center_counts[zoom],label='目标相对标定点')
    axes[0].set_ylabel('ADC计数'); axes[0].legend(ncol=2,loc='upper right')
    axes[1].plot(t[zoom],a['arm_relative_counts'][zoom],label='转臂相对捕获点')
    axes[1].axhline(0,color='gray',lw=.8); axes[1].set_ylabel('编码器计数')
    axes[1].legend(loc='upper right')
    for name,values in zip(('P','I','D'),reconstructed_angle):
        axes[2].plot(t[term_zoom],values[term_zoom],label=f'角度{name}项估算',lw=1)
    axes[2].plot(t[zoom],a['control_pwm'][zoom],color='black',alpha=.65,lw=.8,label='记录PWM')
    axes[2].set_ylabel('PWM分量'); axes[2].legend(ncol=4,loc='upper right')
    for name,values in zip(('P','I','D'),reconstructed_pos):
        axes[3].plot(t[term_zoom],values[term_zoom],label=f'位置{name}项估算',lw=1)
    axes[3].plot(t[zoom],a['position_output_counts'][zoom],color='black',alpha=.6,lw=.8,label='记录位置环输出')
    axes[3].set_ylabel('角度目标偏置 / ADC计数'); axes[3].legend(ncol=4,loc='upper right')
    axes[3].set_xlabel('FPGA时间 / 秒（记录首帧为0）')
    for ax in axes: ax.grid(alpha=.2); ax.set_xlim(t[zoom][0],t[zoom][-1])
    if not settled_available:
        for ax in axes:
            ax.axvline(start,color='tab:red',ls='--',lw=.8)
            if end<len(rows): ax.axvline(t[end],color='tab:red',ls=':',lw=.8)
    fig.suptitle('稳态段两环分量' if settled_available else '捕获后失稳暂态：该区间不能用于KI减小效果比较')
    fig.savefig(out/'balance_detail.png'); plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('integrity','first_balance_start_s',
                      'first_balance_observed_span_s','dominant_arm_frequency_hz',
                      'angle_integral_dominant_pwm_amplitude','angle_reconstruction_pwm_rmse')},ensure_ascii=False))
    print(out)


if __name__ == '__main__':
    main()
