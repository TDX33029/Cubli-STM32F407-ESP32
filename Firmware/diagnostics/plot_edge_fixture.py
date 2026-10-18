"""Export fixture response evidence without treating support as free balance."""
import json,os
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='windows'
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
import pyqtgraph.exporters
folder=Path(__file__).resolve().parent/'2026-10-14'
app=QApplication([]);app.setFont(QFont('Microsoft YaHei',9))
pg.setConfigOptions(background='w',foreground='#334155',antialias=True)
w=pg.GraphicsLayoutWidget();w.setAttribute(Qt.WA_DontShowOnScreen,True);w.resize(1400,900)
w.addLabel('M4/程序M1：限位下加力与参考偏置响应，自由平衡尚未验证',colspan=2,size='15pt',bold=True)
metrics={}
for col,name in enumerate(['edge_fixture_gain70_dirminus','edge_fixture_gain70_offset_plus1']):
    points=[]
    for l in (folder/(name+'.jsonl')).read_text().splitlines():
        r=json.loads(l)
        if r['kind']=='rx' and r['data'].startswith('$FAST,') and r['data'].endswith('#'):
            d=dict(i.split(':',1) for i in r['data'][6:-1].split(','))
            if d.get('Bal')=='3':points.append(d)
    t0=int(points[0]['T']);t=[((int(d['T'])-t0)&0xffffffff)*1e-6 for d in points]
    metrics[name]={'frames':len(points),'duration_mcu_s':t[-1]}
    title='零偏置，静止12秒' if col==0 else '1°目标偏置，4秒响应'
    for row,key,label,unit in [(1,'Tilt','控制误差','°'),(2,'V1','飞轮速度','rad/s'),(3,'U1','q轴电压','V')]:
        values=[float(d[key]) for d in points]
        plot=w.addPlot(row=row,col=col,title=title+'：'+label);plot.showGrid(x=True,y=True,alpha=.2)
        plot.setLabel('left',label,units=unit);plot.setLabel('bottom','时间',units='s')
        plot.plot(t,values,pen=pg.mkPen('#2563eb' if col==0 else '#059669',width=1.8))
        metrics[name][key]={'min':min(values),'max':max(values),'last':values[-1]}
w.addLabel('Kθ=70，Kω机体=2.4，飞轮反馈=0.02，Uq上限2.5V；右列误差仍约0.94°，需排除手扶/限位支撑与摩擦影响。',row=4,col=0,colspan=2)
w.show();app.processEvents();exporter=pg.exporters.ImageExporter(w.scene());exporter.parameters()['width']=1600;exporter.export(str(folder/'edge_fixture_response.png'));w.close()
(folder/'edge_fixture_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n',encoding='utf-8');print(json.dumps(metrics,indent=2))
