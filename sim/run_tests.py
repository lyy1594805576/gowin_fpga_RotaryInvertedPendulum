"""Portable, self-contained RTL and optional virtual host/UI checks."""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'sim'

def run(args):
    p = subprocess.run([str(x) for x in args], cwd=ROOT, input=b'', capture_output=True,
                       env=dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUTF8='1'))
    output = (p.stdout+p.stderr).decode('utf-8', errors='replace')
    if p.returncode:
        raise RuntimeError(f'Command failed ({p.returncode}): {args}\n{output}')
    print(output, end='', flush=True)
    return output

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iverilog-bin', type=Path, help='Directory containing iverilog and vvp')
    parser.add_argument('--host-tests', action='store_true', help='Also run virtual serial/Tk checks; needs a display or Xvfb')
    args = parser.parse_args()
    directory = args.iverilog_bin or (Path(os.environ['IVERILOG_BIN']) if os.environ.get('IVERILOG_BIN') else None)
    def tool(name):
        found = shutil.which(name, path=str(directory) if directory else None)
        if not found:
            raise SystemExit(f'{name} not found: add Icarus to PATH or use --iverilog-bin')
        return found
    compiler, runtime = tool('iverilog'), tool('vvp')
    project = ET.parse(ROOT/'jiangxiedaolibai.gprj').getroot()
    rtl = [ROOT/f.attrib['path'] for f in project.find('FileList')
           if f.attrib.get('enable')=='1' and f.attrib.get('type')=='file.verilog']
    run([compiler, '-g2001', '-Wall', '-s', 'top', '-o', SIM/'top.vvp', *rtl])
    print('PASS Verilog-2001 project compile', flush=True)
    cases = json.loads((SIM/'cases.json').read_text(encoding='utf-8'))
    for name, files in cases.items():
        run([compiler, '-g2012', '-s', name, '-o', SIM/(name+'.vvp'), SIM/(name+'.v'),
             *[ROOT/'src'/f for f in files]])
        output = run([runtime, SIM/(name+'.vvp')])
        if 'PASS' not in output or 'FATAL:' in output:
            raise RuntimeError(f'{name} did not report PASS')
    print(f'PASS all {len(cases)} standalone RTL simulations', flush=True)
    if args.host_tests:
        for file, options in (
            ('tools/auto_swing_logger.py', ['--self-test','--fixture','sim/trace_uart_fixture.hex']),
            ('tools/test_runtime_pid.py', ['--ui']), ('tools/test_friction_ff.py', []),
            ('tools/position_host/test_protocol.py', []), ('tools/position_host/test_host_smoke.py', [])):
            run([sys.executable, ROOT/file, *options])
        print('PASS virtual serial, protocol and host/UI checks', flush=True)

if __name__ == '__main__':
    main()
