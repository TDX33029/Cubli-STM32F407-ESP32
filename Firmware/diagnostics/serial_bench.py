"""Bounded serial bench experiment. Always STOP/disable on exit; no flashing."""
import argparse,json,time,statistics
from pathlib import Path
import serial
parser=argparse.ArgumentParser()
parser.add_argument('--port',default='COM9')
parser.add_argument('--voltage',type=float,default=1.5)
parser.add_argument('--speed',type=float,default=2.0)
parser.add_argument('--seconds',type=float,default=1.0)
parser.add_argument('--axes',type=int,nargs='+',default=[1,2,3])
parser.add_argument('--align',action='store_true')
parser.add_argument('--closed',action='store_true')
parser.add_argument('--tag',default='low_voltage_openloop')
args=parser.parse_args()
if not (0<args.voltage<=3 and abs(args.speed)<=10 and 0<args.seconds<=2 and all(a in (1,2,3) for a in args.axes)):
 raise SystemExit('Bench limits exceeded')
folder=Path(__file__).resolve().parent/'2026-10-14';folder.mkdir(exist_ok=True)
log=(folder/(args.tag+'.jsonl')).open('w',encoding='utf-8')
s=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.3);s.dtr=False;s.rts=False;s.port=args.port
start=time.monotonic();next_ping=0;buffer=bytearray();frames=[];messages=[];phase='connect'
def record(kind,data):
 log.write(json.dumps({'t':round(time.monotonic()-start,5),'phase':phase,'kind':kind,'data':data},ensure_ascii=False)+'\n');log.flush()
def cmd(text):
 s.write((text+'\r\n').encode('ascii'));record('tx',text)
def pump(seconds):
 global buffer,next_ping
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  now=time.monotonic()
  if now>=next_ping:cmd('PING');next_ping=now+.25
  chunk=s.read(s.in_waiting or 1)
  if chunk:buffer.extend(chunk)
  while b'\n' in buffer:
   line,_,buffer=buffer.partition(b'\n');line=line.decode('ascii','replace').strip()
   if not line:continue
   record('rx',line);messages.append((time.monotonic()-start,phase,line))
   if line.startswith(('$TELE,','$STATE,')) and line.endswith('#'):
    try:
     data=dict(item.split(':',1) for item in line.split(',',1)[1][:-1].split(','));data['kind']='tele' if line.startswith('$TELE') else 'state'
     frames.append((time.monotonic()-start,phase,data))
    except ValueError:pass
  if len(buffer)>65536:raise RuntimeError('RX overflow')
def summary(label,axis):
 data=[d for t,p,d in frames if p==label and d['kind']=='tele']
 travel=[]
 for i in (1,2,3):
  vals=[float(d['H'+str(i)]) for d in data if 'H'+str(i) in d]
  travel.append(round(sum((b-a+180)%360-180 for a,b in zip(vals,vals[1:])),2))
 speeds=[float(d['S'+str(axis)]) for d in data if 'S'+str(axis) in d]
 states=[d for t,p,d in frames if p==label and d['kind']=='state']
 return {'phase':label,'tele_samples':len(data),'encoder_travel_deg':travel,'speed_min':min(speeds,default=None),'speed_max':max(speeds,default=None),'speed_mean':round(statistics.mean(speeds),3) if speeds else None,'last_state':states[-1] if states else None,'frequency':[line for t,p,line in messages if p==label and line.startswith('$FREQ,')]}
try:
 s.open();cmd('STOP');pump(.2);cmd('APP 0');pump(.15);cmd('STATE?');pump(.2)
 original=next((d for t,p,d in reversed(frames) if d['kind']=='state'),{})
 cmd('U '+str(args.voltage));pump(.1);cmd('L 10');pump(.1);cmd('MODE 0');pump(.1)
 results=[]
 for axis in args.axes:
  phase='prepare_M'+str(axis)
  cmd('APP 0');pump(.15)
  if args.align:
   phase='align_M'+str(axis);cmd('TEST '+str(axis));align_deadline=time.monotonic()+15
   while time.monotonic()<align_deadline:
    pump(.1)
    if any(p==phase and line in ('OK ALIGN','ERR ALIGN') for t,p,line in messages):break
   align_lines=[line for t,p,line in messages if p==phase and not line.startswith(('$TELE','$STATE'))]
   print(json.dumps({'phase':phase,'messages':align_lines},ensure_ascii=True),flush=True)
   if not any(line=='OK ALIGN' for line in align_lines):
    results.append({'phase':phase,'failed':True});continue
  cmd('U '+str(args.voltage));pump(.08);cmd('MODE '+('1' if args.closed else '0'));pump(.1)
  phase='run_M'+str(axis)
  cmd('EN'+str(axis)+' 1');pump(.08);cmd('FREQ 1');pump(.06);cmd('M'+str(axis)+' '+str(args.speed));pump(args.seconds)
  cmd('FREQ?');pump(.1)
  cmd('STATE?');pump(.1)
  results.append(summary(phase,axis));print(json.dumps(results[-1],ensure_ascii=True),flush=True)
  phase='stop_M'+str(axis);cmd('M'+str(axis)+' 0');pump(.15);cmd('EN'+str(axis)+' 0');pump(.3)
 phase='finish';cmd('STOP');pump(.2)
 if original:
  cmd('U '+original.get('VLimit','5'));pump(.06);cmd('L '+original.get('SLimit','180'));pump(.06)
 record('summary',results)
 (folder/(args.tag+'_summary.json')).write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
finally:
 if s.is_open:
  try:cmd('STOP');pump(.1)
  finally:s.close()
 log.close()
