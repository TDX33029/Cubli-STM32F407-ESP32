import os
os.environ['QT_QPA_PLATFORM']='windows'
import json
from pathlib import Path
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import pyqtgraph.exporters
folder=Path(__file__).resolve().parent/'2026-10-14'
def read(name):return [json.loads(l) for l in (folder/name).read_text().splitlines()]
def curve(name,axis):
 rows=read(name);prefix=f'M{axis}_r';t0=next(r['t'] for r in rows if r['kind']=='tx' and r['phase']==f'M{axis}_r0_v5' and r['data'].startswith(f'M{axis} '))
 out=[]
 for r in rows:
  if r['phase'].startswith(prefix) and r['kind']=='rx' and r['data'].startswith('$TELE,') and r['data'].endswith('#'):
   d=dict(v.split(':',1) for v in r['data'][6:-1].split(','));out.append((r['t']-t0,float(d[f'S{axis}']),float(d[f'M{axis}'])))
 return out
app=QApplication([]);app.setFont(QFont('Microsoft YaHei',9));pg.setConfigOption('background','w');pg.setConfigOption('foreground','#334155');pg.setConfigOption('antialias',True)
w=pg.GraphicsLayoutWidget();w.setAttribute(Qt.WA_DontShowOnScreen,True);w.resize(1450,900)
w.addLabel('12V供电 / Uq限幅2.5V：M4（程序M1）与顶层M3实测',colspan=2,size='16pt',bold=True)
w.nextRow()
for col,axis in enumerate((1,3)):
 plot=w.addPlot(row=1,col=col,title=('侧面 M4（程序M1）' if axis==1 else '顶层 M3')+'：速度阶跃对比')
 plot.addLegend(offset=(10,10));plot.showGrid(x=True,y=True,alpha=.2);plot.setLabel('left','速度',units='rad/s');plot.setLabel('bottom','时间',units='s')
 old=curve('steps_fast_telemetry_baseline.jsonl',axis);new=curve('steps_identified_feedforward.jsonl',axis)
 plot.plot([x[0] for x in old],[x[1] for x in old],pen=pg.mkPen('#94a3b8',width=2),name='原 PI / 前馈参数')
 plot.plot([x[0] for x in new],[x[1] for x in new],pen=pg.mkPen('#2563eb',width=2),name='本次整定后')
 plot.plot([x[0] for x in new],[x[2] for x in new],pen=pg.mkPen('#ef4444',width=1.3,style=Qt.DashLine),name='速度设定')
 plot.setXRange(0,4.1,padding=0);plot.setYRange(-17,17,padding=0)
rows=read('final_dual_100Hz.jsonl');fast=[]
for r in rows:
 if r['phase'].startswith('dual_pulse') and r['kind']=='rx' and r['data'].startswith('$FAST,') and r['data'].endswith('#'):
  fast.append({k:float(v) for k,v in (item.split(':',1) for item in r['data'][6:-1].split(','))})
t0=fast[0]['T']
for col,axis in enumerate((1,3)):
 plot=w.addPlot(row=2,col=col,title=('M4（程序M1）' if axis==1 else '顶层M3')+'：双轴同时5Hz变向（100Hz抓数）')
 plot.addLegend(offset=(10,10));plot.showGrid(x=True,y=True,alpha=.2);plot.setLabel('left','速度',units='rad/s');plot.setLabel('bottom','时间',units='s')
 plot.plot([(r['T']-t0)*1e-6 for r in fast],[r[f'R{axis}'] for r in fast],pen=pg.mkPen('#ef4444',width=1.5,style=Qt.DashLine),name='斜坡后参考')
 plot.plot([(r['T']-t0)*1e-6 for r in fast],[r[f'V{axis}'] for r in fast],pen=pg.mkPen('#059669',width=2),name='实测速度')
 plot.setXRange(0,2,padding=0);plot.setYRange(-12,12,padding=0)
w.addLabel('上排：25Hz遥测，原/新参数均已使用快速格式化；下排：两轴同步、每100ms换向。测试不能替代实机平衡或长期温升验证。',row=3,col=0,colspan=2,size='10pt')
w.show();app.processEvents();exporter=pg.exporters.ImageExporter(w.scene());exporter.parameters()['width']=1600;exporter.export(str(folder/'step_comparison.png'));w.close()
print('Exported step_comparison.png')
