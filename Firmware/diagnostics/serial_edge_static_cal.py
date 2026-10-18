"""Calibrate stationary IMU only; never enable a motor."""
import argparse,json,time
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('--tag',required=True);p.add_argument('--port',default='COM9');a=p.parse_args()
rows=[];buf=bytearray();ping=0;start=time.monotonic()
s=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.3);s.dtr=False;s.rts=False;s.port=a.port
def cmd(c):s.write((c+'\r\n').encode());rows.append({'t':time.monotonic()-start,'kind':'tx','data':c})
def pump(t):
    global buf,ping
    end=time.monotonic()+t
    while time.monotonic()<end:
        if time.monotonic()>=ping:cmd('PING');ping=time.monotonic()+.25
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            l,_,buf=buf.partition(b'\n');line=l.decode('ascii','replace').strip()
            if line:rows.append({'t':time.monotonic()-start,'kind':'rx','data':line})
try:
    s.open();cmd('PING');pump(.1);cmd('STOP');pump(.1);cmd('APP 0');pump(.1);cmd('CAL_IMU');pump(3)
    for c in ['U 2.5','L 15','STATE?','STOP']:cmd(c);pump(.1)
finally:
    if s.is_open:s.write(b'STOP\r\n');s.close()
    (Path(__file__).resolve().parent/'2026-10-14'/(a.tag+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')
print([r['data'] for r in rows if r['data'].startswith('OK CAL')]);print([r['data'] for r in rows if r['data'].startswith('$STATE,')][-1])
assert any(r['data']=='OK CAL_IMU:DONE' for r in rows),'Static calibration did not finish'
