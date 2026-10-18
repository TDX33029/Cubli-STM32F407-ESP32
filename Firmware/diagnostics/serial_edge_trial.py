"""One supervised edge attempt: wait pose, MCU countdown, bounded run, STOP on exit."""
import argparse,json,time
from pathlib import Path
import serial

p=argparse.ArgumentParser()
p.add_argument('--port',default='COM9')
p.add_argument('--axis',type=int,choices=(1,2,3),default=1)
p.add_argument('--body-axis',type=int,choices=(1,2,3),help='Measured IMU axis X=1/Y=2/Z=3; required for M2/M3, legacy M1 defaults to Y')
p.add_argument('--direction',type=int,choices=(-1,1),required=True)
p.add_argument('--tag',required=True)
p.add_argument('--run-seconds',type=float,default=5)
p.add_argument('--reuse-reference',action='store_true')
p.add_argument('--speed-bound',type=float,default=40)
p.add_argument('--wait-seconds',type=float,default=60)
p.add_argument('--wide-window',action='store_true')
p.add_argument('--update-direction',action='store_true')
p.add_argument('--voltage',type=float,default=1.5);p.add_argument('--kp',type=float,default=20);p.add_argument('--kd',type=float,default=1.2);p.add_argument('--kw',type=float,default=.02)
p.add_argument('--wheel-limit',type=float,default=60);p.add_argument('--tilt-bound',type=float,default=8)
p.add_argument('--trip-angle',type=float,default=20)
p.add_argument('--zero-offset',type=float,default=0)
p.add_argument('--reference-json',type=Path)
p.add_argument('--preserve-fall',action='store_true')
p.add_argument('--launch-probe',type=float,help='Fixture only: timed pulse towards reference from verified endpoint, then guarded BAL 1')
p.add_argument('--launch-ms',type=int,default=80)
p.add_argument('--brake-wheel',action='store_true',help='Stop selected wheel in debug velocity mode before configuring the trial')
a=p.parse_args()
if a.body_axis is None:
    if a.axis!=1:p.error('--body-axis is required for M2/M3')
    a.body_axis=2
run_limit=45 if a.kp==a.kd==a.kw==0 and a.voltage<=.5 else 15
if not (0<a.run_seconds<=run_limit and 20<=a.speed_bound<=a.wheel_limit<=360 and 15<=a.wait_seconds<=120 and 0<a.voltage<=6 and 0<=a.kp<=120 and 0<=a.kd<=10 and abs(a.kw)<=.1 and 10<=a.trip_angle<=60 and 3<=a.tilt_bound<=a.trip_angle and abs(a.zero_offset)<=2):raise SystemExit('Fixture trial bounds exceeded')
if a.launch_probe is not None and not (a.axis==1 and a.body_axis==2 and 0<abs(a.launch_probe)<=6 and 20<=a.launch_ms<=(80 if abs(a.launch_probe)>3 else 120) and a.direction==-1 and (a.reference_json or a.reuse_reference)):
    raise SystemExit('Fixture launch requires verified direction -1, a saved reference and a bounded pulse')
folder=Path(__file__).resolve().parent/'2026-10-14'
log=(folder/(a.tag+'.jsonl')).open('x',encoding='utf-8')
rows=[];frames=[];buf=bytearray();phase='connect';start=time.monotonic();ping=0;state={};state_revision=0;watch=False;last_rx=0;run_started=None;result='incomplete';freq_queried=False
s=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.3);s.dtr=False;s.rts=False;s.port=a.port
other_axes=[i for i in range(3) if i!=a.axis-1]
speed_key=f'V{a.axis}';tele_speed_key=f'S{a.axis}';mask=1<<(a.axis-1)
def record(kind,data):
    r={'t':time.monotonic()-start,'phase':phase,'kind':kind,'data':data};rows.append(r);log.write(json.dumps(r)+'\n');log.flush()
def cmd(c):s.write((c+'\r\n').encode());record('tx',c)
def pump(duration):
    global buf,ping,state,state_revision,last_rx,run_started,result
    end=time.monotonic()+duration
    while time.monotonic()<end:
        now=time.monotonic()
        if now>=ping:cmd('PING');ping=now+.25
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            line,_,buf=buf.partition(b'\n');line=line.decode('ascii','replace').strip()
            if not line:continue
            record('rx',line)
            if line.startswith('ERR') and phase!='finish' and not (phase=='capture_target' and line.startswith('ERR BZERO')) and not (not watch and line=='ERR SAFETY_TRIP:1'):
                raise RuntimeError(line)
            if not(line.startswith(('$STATE,','$FAST,','$TELE,')) and line.endswith('#')):continue
            d=dict(item.split(':',1) for item in line.split(',',1)[1][:-1].split(','));frames.append((time.monotonic()-start,d));last_rx=now
            if 'App' in d:state=d;state_revision+=1
            if watch:
                if 'EN' in d and any(d['EN'][i]!='0' for i in other_axes):raise RuntimeError('Unexpected other-axis enable')
                if int(d.get('Fault',0)) or int(d.get('Trip',0)):raise RuntimeError('Firmware fault/trip')
                if int(d.get('Bal',0))==4:
                    result='firmware_fall_protection';raise RuntimeError('Firmware fall protection; already disarmed')
                if int(d.get('Bal',0))==3:
                    if run_started is None:run_started=now;cmd(f'FREQ {mask}');print('BALANCING started; PG6 on',flush=True)
                    if abs(float(d.get('Tilt',0)))>a.tilt_bound:raise RuntimeError('Fixture trial host tilt bound')
                    if abs(float(d.get(speed_key,d.get(tele_speed_key,0))))>a.speed_bound:raise RuntimeError('Trial wheel speed bound')
                    if abs(float(d.get('Rate',0)))>2:raise RuntimeError('First trial body rate bound')
        if watch and now-last_rx>.4:raise RuntimeError('Telemetry timeout')
def wait(c,t=.12):cmd(c);pump(t)
def confirm_state(predicate,label,timeout=1):
    previous=state_revision;cmd('STATE?');deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        pump(.02)
        if state_revision>previous and predicate(state):return
    raise RuntimeError(label+' fresh state confirmation timeout')
try:
    s.open();wait('PING');wait('STOP');wait('APP 0');wait('STATE?')
    if state.get('Aligned','000')[a.axis-1]!='1' or state.get('Cal')!='1':raise RuntimeError('Selected motor alignment/static calibration required')
    if a.launch_probe is not None or a.brake_wheel:
        phase='fixture_brake';wait('MODE 1');wait('U 2.5');wait('L 15');wait(f'EN{a.axis} 1');wait(f'M{a.axis} 0',.8);wait(f'EN{a.axis} 0')
    if a.reuse_reference and not (state.get('Ref')=='1' and state.get('BMode')==str(a.axis) and state.get(f'Axis{a.axis}')==str(a.body_axis) and (a.update_direction or state.get(f'D{a.axis}')==str(a.direction)) and int(state.get('AxisMask',0))&mask and int(state.get('DirMask',0))&mask):
        raise RuntimeError('Saved reference/configuration does not match selected edge')
    wait(f'U {a.voltage}');wait(f'L {a.wheel_limit}');wait('APP 1');wait(f'BMODE {a.axis}');wait(f'BTRIP {a.trip_angle}')
    confirm_state(lambda d:abs(float(d.get('TripDeg',-1))-a.trip_angle)<=.11,'Firmware trip angle')
    confirm_state(lambda d:abs(float(d.get('SLimit',-1))-a.wheel_limit)<=.11,'Firmware wheel limit')
    if not a.reuse_reference:
        wait(f'BAXIS{a.axis} {a.body_axis}');wait(f'BDIR{a.axis} {a.direction}')
    elif a.update_direction:wait(f'BDIR{a.axis} {a.direction}')
    wait(f'LQR{a.axis} {a.kp} {a.kd} {a.kw} 0');wait(f'BU {a.voltage}')
    if a.wide_window:wait('BCAP 10')
    phase='capture_target';pump(.5)
    if not a.reuse_reference:
        if a.reference_json:
            g=json.loads(a.reference_json.read_text(encoding='utf-8'))['gravity']
            if len(g)!=3 or not .9<sum(float(v)**2 for v in g)**.5<1.1:raise RuntimeError('Invalid saved gravity reference')
            wait('BREF '+' '.join(f'{float(v):.8f}' for v in g),.2)
        else:
            for attempt in range(10):
                wait('BZERO',.3)
                if state.get('Ref')=='1':break
    confirm_state(lambda d:d.get('Ref')=='1' and d.get('EN')=='000','Target reference')
    wait(f'ZERO{a.axis} {a.zero_offset}')
    wait(f'FAST {mask}');wait(f'FREQ {mask}')
    if a.launch_probe is not None:
        phase='fixture_launch_precheck';wait('STATE?')
        latest=next(d for t,d in reversed(frames) if 'Rate' in d)
        angle=float(state.get('Angle',0))
        if state.get('EN')!='000' or not 4<=abs(angle)<=12 or angle*a.launch_probe>=0 or abs(float(latest['Rate']))>.1 or abs(float(latest[speed_key]))>2:
            raise RuntimeError('Fixture launch requires a still endpoint, pulse towards reference and stopped wheel')
        wait(f'U {max(a.voltage,abs(a.launch_probe))}')
        phase='fixture_launch_probe';first_probe_frame=len(frames);watch=True;last_rx=time.monotonic()
        cmd(f'BPROBE {a.launch_probe} {a.launch_ms}')
        probe_deadline=time.monotonic()+.5
        while time.monotonic()<probe_deadline:
            pump(.01)
            probe_frames=[d for t,d in frames[first_probe_frame:]]
            if any(d.get('Bal')=='7' for d in probe_frames) and probe_frames[-1].get('Bal')=='0':break
        else:raise RuntimeError('MCU probe completion not observed; refuse handover')
        phase='fixture_launch_capture';wait(f'U {a.voltage}',.02);wait('BAL 1',.05)
        deadline=time.monotonic()+2
    else:
        phase='wait_pose_and_countdown';wait('BSTART 8',.15)
        watch=True;last_rx=time.monotonic();deadline=time.monotonic()+a.wait_seconds
    previous=None
    while True:
        if run_started is None and time.monotonic()>=deadline:
            result='launch_capture_timeout' if a.launch_probe is not None else 'pose/countdown_timeout';break
        pump(.05)
        if state.get('Bal')!=previous:
            previous=state.get('Bal');print('State',previous,'CountMs',state.get('CountMs'),'Tilt',state.get('Tilt'),flush=True)
        if run_started is not None and not freq_queried and time.monotonic()-run_started>=a.run_seconds-.3:
            cmd('FREQ?');freq_queried=True
        if run_started is not None and time.monotonic()-run_started>=a.run_seconds:
            result='bounded_run_completed';break
except BaseException as e:
    if result=='incomplete':result='stopped_by_guard'
    record('error',repr(e));print(repr(e),flush=True)
finally:
    run_ended=time.monotonic();watch=False;phase='finish'
    if s.is_open:
        try:
            preserve=False
            if a.preserve_fall and result=='firmware_fall_protection':
                wait('FAST 0',.3);wait('STATE?',.2)
                preserve=state.get('EN')=='000' and state.get('Bal')=='4'
            if not preserve:cmd('STOP');pump(.2);wait('FAST 0');wait(f'ZERO{a.axis} 0')
            wait('U 2.5');wait('L 15');wait('STATE?')
        finally:s.close()
    summary={'settings':{k:(str(v) if isinstance(v,Path) else v) for k,v in vars(a).items()},'result':result,'run_duration':run_ended-run_started if run_started else 0,'last_state':state,
             'last_running_states':[d for t,d in frames if d.get('Bal') in ('3','4')][-10:],
             'frequency':[r['data'] for r in rows if r['kind']=='rx' and r['data'].startswith('$FREQ,')]}
    (folder/(a.tag+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');log.close()
    print(json.dumps(summary),flush=True)
