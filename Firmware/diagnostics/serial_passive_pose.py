"""Record a candidate edge pose with all motors disabled; never capture BZERO."""
import argparse
import json
import statistics
import time
from pathlib import Path

import serial
from live_pose import PosePublisher

p = argparse.ArgumentParser()
p.add_argument('--tag', required=True)
p.add_argument('--seconds', type=float, default=5)
p.add_argument('--port', default='COM9')
p.add_argument('--live-state',type=Path)
p.add_argument('--target-pitch',type=float,default=23.4399)
p.add_argument('--target-roll',type=float,default=50)
p.add_argument('--capture-angle',type=float,default=.5)
p.add_argument('--stop-on-capture',type=Path,help='End disabled recording when the pose guide saves a new candidate')
a = p.parse_args()
if not 1 <= a.seconds <= 600 or not .5 <= a.capture_angle <= 10:
    raise SystemExit('Passive recording duration must be 1..600 seconds')
folder = Path(__file__).resolve().parent / '2026-10-14'
log = (folder / (a.tag + '.jsonl')).open('x', encoding='utf-8')
s = serial.Serial(port=None, baudrate=115200, timeout=.01, write_timeout=.3)
s.dtr = False
s.rts = False
s.port = a.port
start = time.monotonic()
started_at=time.time()
capture_check=0
captured_candidate=None
buf = b''
frames = []
state = {}
publisher=PosePublisher(a.live_state,[a.target_pitch,a.target_roll],capture_angle=a.capture_angle)

def record(kind, data):
    log.write(json.dumps({'t': time.monotonic() - start, 'kind': kind, 'data': data}) + '\n')
    log.flush()

def send(c):
    s.write((c + '\r\n').encode('ascii'))
    record('tx', c)

try:
    s.open()
    send('PING')
    send('STOP')
    send('FAST 0')
    send('STATE?')
    ping = time.monotonic() + .25
    deadline = time.monotonic() + a.seconds
    while time.monotonic() < deadline:
        if a.stop_on_capture and time.monotonic()>=capture_check:
            capture_check=time.monotonic()+.2
            try:
                candidate=json.loads(a.stop_on_capture.read_text(encoding='utf-8'))
                if float(candidate.get('captured_at',0))>=started_at:
                    captured_candidate=candidate;record('candidate',candidate);break
            except (OSError,ValueError,TypeError):
                pass
        if time.monotonic() >= ping:
            send('PING')
            ping = time.monotonic() + .25
        buf += s.read(s.in_waiting or 1)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            line = line.decode('ascii', 'replace').strip()
            if not line:
                continue
            record('rx', line)
            if not line.startswith(('$TELE,', '$STATE,')) or not line.endswith('#'):
                continue
            try:
                d = dict(v.split(':', 1) for v in line.split(',', 1)[1][:-1].split(','))
            except ValueError:
                continue
            if d.get('EN') != '000':
                raise RuntimeError('Motor enable seen during passive recording')
            publisher.update(d)
            if line.startswith('$STATE,'):
                state = d
            else:
                frames.append(d)
    if len(frames) < 10 or state.get('EN') != '000':
        raise RuntimeError('Insufficient disabled telemetry')
    summary = {'motor_motion_commanded': False, 'neutral_verified': False,
               'samples': len(frames), 'last_state': state, 'ranges': {},'captured_candidate':captured_candidate}
    for key in ('Pit', 'Rol', 'Ax', 'Ay', 'Az', 'Gx', 'Gy', 'Gz'):
        vals = [float(d[key]) for d in frames if key in d]
        if vals:
            summary['ranges'][key] = {'median': statistics.median(vals), 'min': min(vals), 'max': max(vals)}
    with (folder / (a.tag + '_summary.json')).open('x', encoding='utf-8') as f:
        json.dump(summary, f, indent=2)
        f.write('\n')
    print(json.dumps(summary), flush=True)
finally:
    try:
        if s.is_open:
            s.write(b'STOP\r\n')
    finally:
        s.close()
        log.close()
