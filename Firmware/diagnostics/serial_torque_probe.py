"""Fixture-only single-axis torque identification. Each pulse times out inside the MCU."""
import argparse,json,time,statistics
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('--tag',required=True)
p.add_argument('--port',default='COM9')
p.add_argument('--axis',type=int,choices=(1,2,3),default=1)
p.add_argument('--body-axis',type=int,choices=(1,2,3),help='Measured IMU axis X=1/Y=2/Z=3; required for M2/M3, legacy M1 defaults to Y')
p.add_argument('--single-voltage',type=float);p.add_argument('--duration-ms',type=int,default=80)
p.add_argument('--baseline-angle',type=float,help='Expected signed STATE angle at the stationary fixture endpoint')
p.add_argument('--pose-tolerance',type=float,default=1)
p.add_argument('--observe-seconds',type=float,default=.35);a=p.parse_args()
if a.body_axis is None:
    if a.axis!=1:p.error('--body-axis is required for M2/M3')
    a.body_axis=2
if not (0<a.pose_tolerance<=2 and .35<=a.observe_seconds<=2):raise SystemExit('Fixture observation bounds exceeded')
if a.single_voltage is not None and not (0<abs(a.single_voltage)<=6 and 20<=a.duration_ms<=120):raise SystemExit('MCU probe bounds exceeded')
plan=[(a.single_voltage,a.duration_ms)] if a.single_voltage is not None else [(sign*level,duration) for level,duration in [(1,80),(1.5,80),(2,60),(2.5,50)] for sign in (1,-1)]
folder=Path(__file__).resolve().parent/'2026-10-14'
log=(folder/(a.tag+'.jsonl')).open('x',encoding='utf-8')
rows=[];frames=[];buf=bytearray();phase='connect';start=time.monotonic();ping=0;state={};state_revision=0;guard=False
s=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.3);s.dtr=False;s.rts=False;s.port=a.port
speed_key=f'V{a.axis}';other_axes=[i for i in range(3) if i!=a.axis-1]
summary={'port':a.port,'motor':a.axis,'body_axis':a.body_axis,'pulses':[],'result':'incomplete'}
def record(kind,data):
    r={'t':time.monotonic()-start,'phase':phase,'kind':kind,'data':data};rows.append(r);log.write(json.dumps(r)+'\n');log.flush()
def cmd(c):s.write((c+'\r\n').encode());record('tx',c)
def pump(duration):
    global buf,ping,state,state_revision
    end=time.monotonic()+duration
    while time.monotonic()<end:
        if time.monotonic()>=ping:cmd('PING');ping=time.monotonic()+.25
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            line,_,buf=buf.partition(b'\n');line=line.decode('ascii','replace').strip()
            if not line:continue
            record('rx',line)
            if line.startswith('ERR') and phase!='finish' and line!='ERR SAFETY_TRIP:1':raise RuntimeError(line)
            if not (line.startswith(('$STATE,','$FAST,')) and line.endswith('#')):continue
            d=dict(x.split(':',1) for x in line.split(',',1)[1][:-1].split(','));frames.append((time.monotonic()-start,phase,d))
            if 'App' in d:state=d;state_revision+=1
            if guard and (int(d.get('Fault',0)) or int(d.get('Trip',0)) or d.get('Bal')=='4' or ('EN' in d and any(d['EN'][i]!='0' for i in other_axes))):raise RuntimeError('Torque probe protection: '+line)
def wait(c,t=.1):cmd(c);pump(t)
try:
    s.open();wait('PING');wait('STOP');wait('APP 0');wait('MODE 1');wait(f'U {max(3,max(abs(v) for v,ms in plan))}');wait('L '+('110' if any(abs(v)>3 and ms>80 for v,ms in plan) else '90'));wait('ACC 400');wait(f'BAXIS{a.axis} {a.body_axis}');wait('STATE?')
    if state.get('Cal')!='1' or state.get('Aligned','000')[a.axis-1]!='1':raise RuntimeError(f'M{a.axis} alignment/static IMU calibration required')
    wait(f'FAST {1<<(a.axis-1)}')
    for pulse_voltage,duration in plan:
        level=abs(pulse_voltage);sign=1 if pulse_voltage>0 else -1
        phase=f'brake_{level}_{sign}';wait('APP 0');wait(f'EN{a.axis} 1');wait(f'M{a.axis} 0',.8)
        latest=next(d for t,p,d in reversed(frames) if speed_key in d)
        if abs(float(latest[speed_key]))>2:raise RuntimeError('Wheel has not stopped')
        wait(f'EN{a.axis} 0');wait('APP 1');wait(f'BMODE {a.axis}');phase=f'baseline_{level}_{sign}';pump(.4)
        baseline=[float(d['Rate']) for t,p,d in frames if p==phase and 'Rate' in d]
        previous=state_revision;cmd('STATE?');deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            pump(.02)
            if state_revision>previous:break
        else:raise RuntimeError('Fresh endpoint state not received')
        latest=next(d for t,p,d in reversed(frames) if speed_key in d and 'Rate' in d)
        if state.get('EN')!='000' or len(baseline)<5 or max(abs(v) for v in baseline)>.1 or abs(float(latest['Rate']))>.1 or abs(float(latest[speed_key]))>2:
            raise RuntimeError('Endpoint/wheel is not stationary; refuse pulse')
        if a.baseline_angle is not None and abs(float(state['Angle'])-a.baseline_angle)>a.pose_tolerance:
            raise RuntimeError('Endpoint moved from expected angle; refuse pulse')
        baseline_state=dict(state)
        phase=f'pulse_{level}_{sign}';guard=True;cmd(f'BPROBE {sign*level} {duration}');pump(a.observe_seconds);guard=False
        active=[d for t,p,d in frames if p==phase and d.get('Bal')=='7' and 'T' in d]
        if len(active)<2:raise RuntimeError('Insufficient active probe samples')
        if state.get('EN')!='000' or state.get('Bal')!='0':raise RuntimeError('MCU did not end probe/disarm')
        rates=[float(d['Rate']) for d in active]
        d={'voltage':sign*level,'duration_ms':duration,'expected_wheel_guard_rad_s':100 if level>3 and duration>80 else 60,'samples':len(active),'baseline_rate':statistics.mean(baseline) if baseline else None,'baseline_state':baseline_state,
           'rates':rates,'speed_peak':max(abs(float(d[speed_key])) for d in active),
           'wx':[float(d['Wx']) for d in active],'wy':[float(d['Wy']) for d in active],'wz':[float(d['Wz']) for d in active]}
        summary['pulses'].append(d);print(json.dumps(d),flush=True)
    summary['result']='completed'
except BaseException as e:
    summary['result']='stopped';summary['error']=repr(e);print(repr(e),flush=True)
finally:
    guard=False;phase='finish'
    if s.is_open:
        try:
            cmd('STOP');pump(.2);wait('FAST 0');wait('U 2.5');wait('L 15');wait('STATE?')
        finally:s.close()
    summary['last_state']=state
    (folder/(a.tag+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');log.close()
