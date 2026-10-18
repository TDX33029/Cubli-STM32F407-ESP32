"""Summarize raw bench evidence and export measured high-speed traces."""
import json
import os
import statistics
from pathlib import Path

os.environ['QT_QPA_PLATFORM'] = 'windows'
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
import pyqtgraph.exporters

folder = Path(__file__).resolve().parent / '2026-10-14'
names = ['highspeed_20_60', 'highspeed_90', 'highspeed_120', 'highspeed_150',
         'highspeed_180', 'highspeed_dual', 'highspeed_dual_fixed',
         'highspeed_180_fixed', 'highspeed_dual_150_final', 'highspeed_dual_120_final']
evidence = {}


def parse(line):
    return dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))


def read(name):
    return [json.loads(l) for l in (folder / (name + '.jsonl')).read_text(encoding='utf-8').splitlines()]


for name in names:
    rows = read(name)
    frames = [(r['t'], r['phase'], parse(r['data'])) for r in rows
              if r['kind'] == 'rx' and r['data'].startswith('$FAST,') and r['data'].endswith('#')]
    states = [parse(r['data']) for r in rows
              if r['kind'] == 'rx' and r['data'].startswith('$STATE,') and r['data'].endswith('#')]
    freq = [parse(r['data']) for r in rows
            if r['kind'] == 'rx' and r['data'].startswith('$FREQ,') and r['data'].endswith('#')]
    errors = [{'phase': r['phase'], 'message': r['data']} for r in rows
              if r['kind'] == 'rx' and r['data'].startswith('ERR')]
    e = {'frames': len(frames), 'frequency': freq, 'errors': errors,
         'last_state': states[-1] if states else None, 'steady': []}
    for ph in dict.fromkeys(ph for t, ph, d in frames):
        if '_modulation' in ph or ph in ('connect', 'finish', 'cooldown'):
            continue
        group = [(t, d) for t, p, d in frames if p == ph]
        if len(group) < 50:
            continue
        tail = [d for t, d in group if t >= group[-1][0] - .7]
        for axis in (1, 3):
            if 'V' + str(axis) not in tail[0]:
                continue
            v = [float(d[f'V{axis}']) for d in tail]
            u = [float(d[f'U{axis}']) for d in tail]
            e['steady'].append({'phase': ph, 'axis': axis, 'reference': float(tail[-1][f'R{axis}']),
                                'n': len(v), 'mean': round(statistics.mean(v), 3),
                                'std': round(statistics.pstdev(v), 3), 'min': min(v), 'max': max(v),
                                'uq_mean': round(statistics.mean(u), 3)})
    if frames:
        periods = [((int(b[2]['T']) - int(a[2]['T'])) & 0xffffffff) / 1000
                   for a, b in zip(frames, frames[1:]) if a[1] == b[1]]
        e['fast_period_ms'] = {'median': statistics.median(periods), 'max': max(periods)}
    evidence[name] = e

final_rows = read('highspeed_dual_120_final')
mod = [parse(r['data']) for r in final_rows if '_modulation' in r['phase']
       and r['kind'] == 'rx' and r['data'].startswith('$FAST,') and r['data'].endswith('#')]
evidence['modulation'] = {}
for axis in (1, 3):
    errs = [float(d[f'V{axis}']) - float(d[f'R{axis}']) for d in mod]
    evidence['modulation'][str(axis)] = {
        'frames': len(errs), 'rms_error_to_ramp': round((statistics.mean(v*v for v in errs)) ** .5, 3),
        'max_abs_error_to_ramp': round(max(abs(v) for v in errs), 3),
        'speed_min': min(float(d[f'V{axis}']) for d in mod),
        'speed_max': max(float(d[f'V{axis}']) for d in mod)}
(folder / 'highspeed_metrics.json').write_text(json.dumps(evidence, indent=2) + '\n', encoding='utf-8')

app = QApplication([])
app.setFont(QFont('Microsoft YaHei', 9))
pg.setConfigOptions(background='w', foreground='#334155', antialias=True)
w = pg.GraphicsLayoutWidget()
w.setAttribute(Qt.WA_DontShowOnScreen, True)
w.resize(1450, 950)
w.addLabel('12V供电 / Uq限幅5V / ACC=400：双轴高转速实测', colspan=2, size='16pt', bold=True)
for col, axis in enumerate((1, 3)):
    title = '侧面M4（程序M1）' if axis == 1 else '顶层M3'
    for row, name, selected, subtitle in (
        (1, 'highspeed_dual_120_final', lambda ph: ph.startswith('axes'),
         '±120 rad/s稳速、换向与停车'),
        (2, 'highspeed_dual_120_final', lambda ph: '_modulation' in ph,
         '90↔120 rad/s，每150ms切换'),
        (3, 'highspeed_dual_fixed', lambda ph: ph.endswith('_v180'),
         '双轴180 rad/s：M3负向触及限压')):
        rows = read(name)
        points = [(r['t'], parse(r['data'])) for r in rows if selected(r['phase']) and
                  r['kind'] == 'rx' and r['data'].startswith('$FAST,') and r['data'].endswith('#')]
        t0 = points[0][0]
        plot = w.addPlot(row=row, col=col, title=title + '：' + subtitle)
        plot.addLegend(offset=(10, 10))
        plot.showGrid(x=True, y=True, alpha=.2)
        plot.setLabel('left', '速度', units='rad/s')
        plot.setLabel('bottom', '时间', units='s')
        plot.plot([t-t0 for t, d in points], [float(d[f'R{axis}']) for t, d in points],
                  pen=pg.mkPen('#ef4444', width=1.4, style=Qt.DashLine), name='斜坡后参考')
        plot.plot([t-t0 for t, d in points], [float(d[f'V{axis}']) for t, d in points],
                  pen=pg.mkPen('#2563eb' if axis == 1 else '#059669', width=1.8), name='实测速度')
w.addLabel('短时速度测试，未测相电流/母线电压/温度；180 rad/s双轴未通过，M2始终失能。',
           row=4, col=0, colspan=2, size='10pt')
w.show()
app.processEvents()
exporter = pg.exporters.ImageExporter(w.scene())
exporter.parameters()['width'] = 1700
exporter.export(str(folder / 'highspeed_results.png'))
w.close()
print(json.dumps({'modulation': evidence['modulation'],
                  'final_frequency': evidence['highspeed_dual_120_final']['frequency'],
                  'final_steady': evidence['highspeed_dual_120_final']['steady']}, indent=2))
