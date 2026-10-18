"""Low-voltage M1/M3 speed-step experiment with timeout, fault monitoring and STOP on exit."""
import argparse,json,time,statistics
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('--port',default='COM9');p.add_argument('--axes',type=int,nargs='+',default=[1,3]);p.add_argument('--align',action='store_true')
p.add_argument('--voltage',type=float,default=2.5);p.add_argument('--tf',type=float);p.add_argument('--coulomb',type=float);p.add_argument('--bemf',type=float,default=.045)
p.add_argument('--pulse',action='store_true');p.add_argument('--holds',action='store_true');p.add_argument('--p',type=float);p.add_argument('--i',type=float,default=1.5);p.add_argument('--tag',required=True)
a=p.parse_args()
if not (0<a.voltage<=3 and all(i in (1,3) for i in a.axes)):raise SystemExit('M1/M3 only; voltage must be <=3V')
folder=Path(__file__).resolve().parent/'2026-10-14';folder.mkdir(exist_ok=True)
log=(folder/(a.tag+'.jsonl')).open('w',encoding='utf-8');rows=[];frames=[];buf=bytearray();phase='connect';watch=False;start=time.monotonic();ping=0
s=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.3);s.dtr=False;s.rts=False;s.port=a.port

def record(kind,data):
 row={'t':round(time.monotonic()-start,6),'phase':phase,'kind':kind,'data':data};rows.append(row);log.write(json.dumps(row)+'\n');log.flush()
def cmd(text):s.write((text+'\r\n').encode());record('tx',text)
def pump(seconds):
 global ping,buf
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  now=time.monotonic()
  if now>=ping:cmd('PING');ping=now+.25
  buf.extend(s.read(s.in_waiting or 1))
  while b'\n' in buf:
   line,_,buf=buf.partition(b'\n');line=line.decode('ascii','replace').strip()
   if not line:continue
   record('rx',line)
   if line.startswith(('$TELE,','$STATE,')) and line.endswith('#'):
    try:d=dict(item.split(':',1) for item in line.split(',',1)[1][:-1].split(','))
    except ValueError:continue
    d['type']='tele' if line.startswith('$TELE,') else 'state';frames.append((time.monotonic()-start,phase,d))
    if watch and d['type']=='state' and (int(d['Trip']) or int(d['Fault'])):raise RuntimeError('Firmware safety trip/fault: '+line)
    if watch and d['type']=='tele' and any(abs(float(d.get('S'+str(i),0)))>22 for i in a.axes):raise RuntimeError('Speed outside bench bounds')

def last_state():return next((d for t,p,d in reversed(frames) if d['type']=='state'),{})
summary=[]
try:
 s.open();cmd('STOP');pump(.2);cmd('APP 0');pump(.1);cmd('U '+str(a.voltage));pump(.08);cmd('L 15');pump(.08);cmd('ACC 400');pump(.08)
 for axis in a.axes:
  phase=f'prepare_M{axis}';cmd('APP 0');pump(.1)
  if a.align:
   phase=f'align_M{axis}';cmd(f'TEST {axis}');deadline=time.monotonic()+9
   while time.monotonic()<deadline:
    pump(.1)
    if any(r['phase']==phase and r['kind']=='rx' and r['data'] in ('OK ALIGN','ERR ALIGN') for r in rows):break
   if not any(r['phase']==phase and r['kind']=='rx' and r['data']=='OK ALIGN' for r in rows):raise RuntimeError(f'M{axis} failed alignment')
  cmd('STATE?');pump(.12)
  if last_state().get('Aligned','000')[axis-1]!='1':raise RuntimeError(f'M{axis} is not aligned')
  if a.tf is not None:cmd(f'TF{axis} {a.tf}');pump(.05)
  if a.coulomb is not None:cmd(f'FF{axis} {a.coulomb} {a.bemf}');pump(.05)
  if a.p is not None:cmd(f'PID{axis} {a.p} {a.i} 0');pump(.05)
  cmd('MODE 1');pump(.08);cmd(f'EN{axis} 1');pump(.12);cmd('FREQ 1');pump(.04)
  watch=True
  for repeat in range(1 if a.holds else 2):
   for target,duration in (((5,1.8),(10,1.8),(-5,1.8),(-10,1.8),(0,.3)) if a.holds else ((5,.35),(-5,.35),(10,.5),(-10,.5),(0,.3))):
    phase=f'M{axis}_r{repeat}_v{target}';t0=time.monotonic()-start;cmd(f'M{axis} {target}');pump(duration)
    samples=[(t,float(d['S'+str(axis)])) for t,p,d in frames if p==phase and d['type']=='tele']
    steady=[v for t,v in samples if t-t0>=.15]
    uq=[float(d['U'+str(axis)]) for t,p,d in frames if p==phase and d['type']=='state' and t-t0>=.8]
    summary.append({'uq_mean_after_800ms':round(statistics.mean(uq),4) if uq else None,'axis':axis,'phase':phase,'target':target,'samples':len(samples),'min':min((v for t,v in samples),default=None),'max':max((v for t,v in samples),default=None),'mean_after_150ms':round(statistics.mean(steady),3) if steady else None,'last':samples[-1][1] if samples else None})
  if a.pulse:
   for cycle in range(20):
    target=10 if cycle%2==0 else -10;phase=f'M{axis}_pulse{cycle}';cmd(f'M{axis} {target}');pump(.1)
    samples=[float(d['S'+str(axis)]) for t,p,d in frames if p==phase and d['type']=='tele']
    summary.append({'axis':axis,'phase':phase,'target':target,'samples':len(samples),'min':min(samples,default=None),'max':max(samples,default=None),'last':samples[-1] if samples else None})
   phase=f'M{axis}_pulse_stop';cmd(f'M{axis} 0');pump(.3)
  phase=f'M{axis}_profile';cmd('FREQ?');pump(.15);cmd('STATE?');pump(.1)
  freq=[r['data'] for r in rows if r['phase']==phase and r['kind']=='rx' and r['data'].startswith('$FREQ,')]
  print(json.dumps({'axis':axis,'frequency':freq,'steps':[v for v in summary if v['axis']==axis]},ensure_ascii=True),flush=True)
  watch=False;cmd(f'EN{axis} 0');pump(.3)
 phase='finish';cmd('STOP');pump(.15)
 (folder/(a.tag+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
finally:
 watch=False
 if s.is_open:
  try:cmd('STOP');pump(.1)
  finally:s.close()
 log.close()
