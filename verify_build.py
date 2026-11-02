"""Check Gowin logs and the actual clock timing, then record source hashes."""
from pathlib import Path
import hashlib
import html
import json
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent
report = root / 'impl/pnr/jiangxiedaolibai_tr_content.html'
text = report.read_text(encoding='utf-8', errors='replace')

def table_rows(heading):
    start = text.index(heading)
    part = text[start:text.index('</table>', start)]
    rows = []
    for row in re.findall(r'<tr[^>]*>(.*?)</tr>', part, re.S):
        cells = [html.unescape(re.sub(r'<[^>]*>', '', c)).strip()
                 for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>', row, re.S)]
        if cells:
            rows.append(cells)
    return rows

clock = next(row for row in table_rows('Max Frequency Summary:') if len(row)>3 and row[1]=='clk_50m')
constraint = float(clock[2].replace('(MHz)', ''))
fmax = float(clock[3].replace('(MHz)', ''))
slack_rows = table_rows('Total Negative Slack Summary:')
setup = next(row for row in slack_rows if 'Setup' in row)
hold = next(row for row in slack_rows if 'Hold' in row)
assert constraint == 50.0, f'Unexpected clock constraint: {constraint}'
assert fmax >= constraint, f'Setup failed: actual Fmax {fmax} MHz < {constraint} MHz'
assert float(setup[-2]) == 0 and int(setup[-1]) == 0, f'Setup violations: {setup}'
assert float(hold[-2]) == 0 and int(hold[-1]) == 0, f'Hold violations: {hold}'
for path in [root/'impl/gwsynthesis/jiangxiedaolibai.log', root/'impl/pnr/jiangxiedaolibai.log']:
    log = path.read_text(encoding='utf-8', errors='replace')
    assert not re.search(r'\b(?:ERROR|FATAL)\s*\(', log), f'Error in {path}'
assert 'GowinSynthesis finish' in (root/'impl/gwsynthesis/jiangxiedaolibai.log').read_text(encoding='utf-8',errors='replace')
assert 'Bitstream generation completed' in (root/'impl/pnr/jiangxiedaolibai.log').read_text(encoding='utf-8',errors='replace')
project = ET.fromstring((root/'jiangxiedaolibai.gprj').read_text(encoding='utf-8').replace('version="1"','version="1.0"'))
sources = [root/f.attrib['path'] for f in project.find('FileList') if f.attrib.get('enable')=='1' and f.attrib.get('type') in ('file.verilog','file.cst','file.sdc')]
bitstream = root/'impl/pnr/jiangxiedaolibai.fs'
assert bitstream.stat().st_size > 0
assert min(report.stat().st_mtime, bitstream.stat().st_mtime) >= max(p.stat().st_mtime for p in sources), 'Sources changed since this build'
summary = {
    'clock_mhz': constraint, 'actual_fmax_mhz': fmax,
    'setup_tns_ns': float(setup[-2]), 'setup_violating_endpoints': int(setup[-1]),
    'hold_tns_ns': float(hold[-2]), 'hold_violating_endpoints': int(hold[-1]),
    'bitstream': str(bitstream),
    'sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    'bitstream_sha256': hashlib.sha256(bitstream.read_bytes()).hexdigest(),
}
(root/'build_verified.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(f'PASS Gowin synthesis, place/route and bitstream; Fmax={fmax:.3f} MHz; setup/hold TNS=0')
