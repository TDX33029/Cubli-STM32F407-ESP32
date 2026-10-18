"""Held M1-only direction identification; always stop, no balance start."""
import json
import time
from pathlib import Path
import serial
import argparse

p=argparse.ArgumentParser();p.add_argument('--no-align',action='store_true');p.add_argument('--no-calibrate',action='store_true');p.add_argument('--tag',default='edge_direction_probe_held')
p.add_argument('--speed',type=float,default=5);p.add_argument('--accel',type=float,default=100);p.add_argument('--voltage',type=float,default=1);p.add_argument('--pulse',type=float,default=.18);p.add_argument('--cycles',type=int,default=8);a=p.parse_args()
if not(0<a.speed<=20 and 100<=a.accel<=800 and 0<a.voltage<=3 and .05<=a.pulse<=.2 and 2<=a.cycles<=8):raise SystemExit('Probe limits exceeded')

folder=Path(__file__).resolve().parent/'2026-10-14'
rows=[];frames=[];buf=bytearray();phase='connect';start=time.monotonic();ping=0
s=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.3)
s.dtr=False;s.rts=False;s.port='COM9'

def record(kind,data):
    rows.append({'t':time.monotonic()-start,'phase':phase,'kind':kind,'data':data})
def cmd(c):
    s.write((c+'\r\n').encode());record('tx',c)
def pump(duration):
    global buf,ping
    end=time.monotonic()+duration
    while time.monotonic()<end:
        if time.monotonic()>=ping:cmd('PING');ping=time.monotonic()+.25
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            line,_,buf=buf.partition(b'\n');line=line.decode('ascii','replace').strip()
            if not line:continue
            record('rx',line)
            if line.startswith(('$TELE,','$STATE,')) and line.endswith('#'):
                d=dict(item.split(':',1) for item in line.split(',',1)[1][:-1].split(','))
                frames.append((time.monotonic()-start,phase,d))
                if d.get('EN','000')[1:]!='00':raise RuntimeError('Unexpected M2/M3 enable')
                if phase.startswith('pulse') and (int(d.get('Fault',0)) or int(d.get('Trip',0))):raise RuntimeError('Safety fault/trip')
                if abs(float(d.get('S1',0)))>max(10,a.speed*1.5):raise RuntimeError('Probe speed limit')
                if phase.startswith('pulse') and any(abs(float(d.get(k,0)))>150 for k in ('Gx','Gy','Gz')):raise RuntimeError('Probe body rate limit')
def wait(c,t=.12):cmd(c);pump(t)
try:
    s.open();wait('PING');wait('STOP');wait('APP 0');wait('MODE 1');wait('U 2.5');wait('L 10')
    if not a.no_align:
        phase='align';cmd('TEST 1');pump(7)
        if not any(r['phase']=='align' and r['data']=='OK ALIGN' for r in rows):raise RuntimeError('Alignment failed')
    if not a.no_calibrate:
        phase='calibrate';pump(1);cmd('CAL_IMU');pump(3)
        if not any(r['data']=='OK CAL_IMU:DONE' for r in rows):raise RuntimeError('Hold still for IMU calibration')
    wait('STATE?')
    state=next(d for t,p,d in reversed(frames) if 'Aligned' in d)
    if state.get('Aligned','000')[0]!='1' or state.get('Cal')!='1':raise RuntimeError('M1 alignment and static IMU calibration required')
    wait('BAXIS1 2');wait(f'U {a.voltage}');wait(f'L {a.speed}');wait(f'ACC {a.accel}');wait('STATE?')
    wait('EN1 1')
    for i in range(a.cycles):
        phase=f'baseline{i}';wait('M1 0',.5)
        phase=f'pulse{i}';cmd('M1 '+str(a.speed if i%2==0 else -a.speed));pump(a.pulse)
        phase=f'brake{i}';wait('M1 0',.4)
    phase='finish';wait('STOP');wait('U 2.5');wait('L 15');wait('ACC 400');wait('STATE?')
finally:
    if s.is_open:
        s.write(b'STOP\r\n');s.close()
    (folder/(a.tag+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')
out=[]
for i in range(a.cycles):
    values=[(t,float(d['Gy'])) for t,p,d in frames if p==f'pulse{i}' and 'Gy' in d]
    baseline=[float(d['Gy']) for t,p,d in frames if p==f'baseline{i}' and 'Gy' in d][-4:]
    out.append({'cycle':i,'target':a.speed if i%2==0 else -a.speed,'baseline':baseline,'response':values})
(folder/(a.tag+'_summary.json')).write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print(json.dumps(out,indent=2),flush=True)
