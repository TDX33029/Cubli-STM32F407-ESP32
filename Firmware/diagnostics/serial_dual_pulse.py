"""Record both healthy axes at 100Hz while applying low-voltage reversal patterns."""
from pathlib import Path
# Reuse the bounded recorder and parser; do not execute serial_steps' experiment body.
source=(Path(__file__).parent/'serial_steps.py').read_text(encoding='utf-8').split('summary=[]\ntry:',1)[0]
source=source.replace("('$TELE,','$STATE,')","('$TELE,','$STATE,','$FAST,')")
source=source.replace("d['type']='tele' if line.startswith('$TELE,') else 'state'","d['type']='fast' if line.startswith('$FAST,') else ('tele' if line.startswith('$TELE,') else 'state')")
exec(compile(source,'serial_recorder','exec'),globals())
summary={}
try:
 s.open();cmd('STOP');pump(.2);cmd('APP 0');pump(.1);cmd('U 2.5');pump(.1);cmd('L 15');pump(.1);cmd('ACC 400');pump(.1)
 for axis in (1,3):
  if a.align:
   phase=f'align_M{axis}';cmd(f'TEST {axis}');deadline=time.monotonic()+9
   while time.monotonic()<deadline:
    pump(.1)
    if any(r['phase']==phase and r['kind']=='rx' and r['data'] in ('OK ALIGN','ERR ALIGN') for r in rows):break
   if not any(r['phase']==phase and r['kind']=='rx' and r['data']=='OK ALIGN' for r in rows):raise RuntimeError(f'M{axis} alignment failed')
 cmd('STATE?');pump(.15)
 if last_state().get('Aligned','000')[0]!='1' or last_state().get('Aligned','000')[2]!='1':raise RuntimeError('Both healthy axes must be aligned')
 phase='arm';cmd('MODE 1');pump(.1);cmd('EN1 1');pump(.05);cmd('EN3 1');pump(.15);cmd('FREQ 1');pump(.03);cmd('FAST 5');pump(.05);watch=True
 for cycle in range(4):
  target=10 if cycle%2==0 else -10;phase=f'dual_step_{cycle}';cmd(f'M1 {target}');cmd(f'M3 {-target}');pump(.5)
 for cycle in range(40):
  target=10 if cycle%2==0 else -10;phase=f'dual_pulse_{cycle}';cmd(f'M1 {target}');cmd(f'M3 {-target}');pump(.1)
  recent=[d for t,p,d in frames if p==phase and d['type']=='fast']
  if any(abs(float(d.get(key,0)))>22 for d in recent for key in ('V1','V3')):raise RuntimeError('Speed outside bench bounds')
 phase='stop';cmd('M1 0');cmd('M3 0');pump(.4);cmd('FREQ?');pump(.15);cmd('STATE?');pump(.15)
 watch=False;cmd('FAST 0');pump(.05);cmd('STOP');pump(.15)
 fast=[d for t,p,d in frames if d['type']=='fast'];summary={'frames':len(fast),'frequency':[r['data'] for r in rows if r['kind']=='rx' and r['data'].startswith('$FREQ,')],'last_state':last_state()}
 for axis in (1,3):
  values=[float(d['V'+str(axis)]) for d in fast if 'V'+str(axis) in d]
  summary[f'M{axis}']={'min':min(values,default=None),'max':max(values,default=None)}
 print(json.dumps(summary,ensure_ascii=True),flush=True)
 (folder/(a.tag+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
finally:
 watch=False
 if s.is_open:
  try:cmd('FAST 0');cmd('STOP');pump(.1)
  finally:s.close()
 log.close()
