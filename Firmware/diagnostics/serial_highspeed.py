"""Bounded M1/M3 high-speed bench; raw evidence, heartbeat and fail-stop cleanup."""
import argparse
import json
import statistics
import time
from pathlib import Path

import serial

p = argparse.ArgumentParser()
p.add_argument('--port', default='COM9')
p.add_argument('--axes', type=int, nargs='+', default=[1, 3])
p.add_argument('--speeds', type=float, nargs='+', required=True)
p.add_argument('--voltage', type=float, required=True)
p.add_argument('--dual', action='store_true')
p.add_argument('--modulation', action='store_true', help='20 steps between top speed and top-30, every 150ms')
p.add_argument('--hold', type=float, default=1.8)
p.add_argument('--tag', required=True)
a = p.parse_args()
if not (0 < a.voltage <= 5 and .8 <= a.hold <= 8
        and all(i in (1, 3) for i in a.axes)
        and all(0 < v <= 180 for v in a.speeds)):
    raise SystemExit('Only M1/M3; U<=5V; speeds 0..180rad/s; hold .8..8s')
if a.dual and a.axes != [1, 3]:
    raise SystemExit('Dual mode requires axes 1 3')
folder = Path(__file__).resolve().parent / '2026-10-14'
folder.mkdir(exist_ok=True)
log = (folder / (a.tag + '.jsonl')).open('x', encoding='utf-8')
s = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.3)
s.dtr = False
s.rts = False
s.port = a.port
start = time.monotonic()
phase = 'connect'
buf = bytearray()
frames = []
summary = {'settings': vars(a), 'steps': [], 'frequency': [], 'result': 'incomplete'}
watch = False
active = []
ping_due = state_due = 0
last_fast = last_state_time = 0
bad_since = {}
state = {}


def record(kind, data):
    row = {'t': round(time.monotonic() - start, 6), 'phase': phase, 'kind': kind, 'data': data}
    log.write(json.dumps(row) + '\n')
    log.flush()


def cmd(line):
    s.write((line + '\r\n').encode('ascii'))
    record('tx', line)


def pump(seconds):
    global buf, ping_due, state_due, state, last_fast, last_state_time
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= ping_due:
            cmd('PING')
            ping_due = now + .25
        if watch and now >= state_due:
            cmd('STATE?')
            state_due = now + .35
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            raw, _, buf = buf.partition(b'\n')
            line = raw.decode('ascii', 'replace').strip()
            if not line:
                continue
            record('rx', line)
            if line.startswith('ERR') and phase != 'finish' and not (
                    not watch and line == 'ERR SAFETY_TRIP:1'):
                raise RuntimeError(line)
            if not (line.startswith(('$STATE,', '$FAST,', '$FREQ,')) and line.endswith('#')):
                continue
            d = dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))
            if line.startswith('$STATE,'):
                state = d
                last_state_time = now
                if watch and (int(d['Fault']) or int(d['Trip']) or d['EN'][1] != '0'
                              or any(d['EN'][i - 1] != '1' for i in active)):
                    raise RuntimeError('Safety/enable state: ' + line)
            elif line.startswith('$FREQ,'):
                summary['frequency'].append({'phase': phase, **d})
            else:
                last_fast = now
                frames.append((time.monotonic() - start, phase, d))
                if watch:
                    for i in active:
                        v, ref = float(d['V' + str(i)]), float(d['R' + str(i)])
                        if abs(v) > min(210, max(a.speeds) * 1.25 + 10):
                            raise RuntimeError(f'M{i}: overspeed {v}')
                        if abs(v - ref) > max(35, abs(ref) * .35):
                            bad_since.setdefault(i, now)
                            if now - bad_since[i] > .3:
                                raise RuntimeError(f'M{i}: persistent tracking loss V={v} R={ref}')
                        else:
                            bad_since.pop(i, None)
        if watch and (now - last_fast > .35 or now - last_state_time > 1):
            raise RuntimeError('Telemetry/status timeout')


def command_wait(line, delay=.08):
    cmd(line)
    pump(delay)


def move(target, label):
    global phase
    phase = label
    before = time.monotonic() - start
    for i in active:
        cmd(f'M{i} {target if i == active[0] else -target}')
    # Includes sufficient ramp time, followed by an independent steady hold.
    pump(2 * abs(target) / 400 + a.hold)
    step = {'phase': phase, 'target': target, 'axes': {}}
    for i in active:
        desired = target if i == active[0] else -target
        data = [d for t, ph, d in frames if ph == phase
                and t >= before + 2 * abs(target) / 400 + a.hold - .7]
        values = [float(d[f'V{i}']) for d in data]
        volts = [float(d[f'U{i}']) for d in data]
        if len(values) < 35:
            raise RuntimeError('Insufficient steady telemetry')
        mean = statistics.mean(values)
        metrics = {'target': desired, 'n': len(values), 'mean': round(mean, 3),
                   'std': round(statistics.pstdev(values), 3), 'min': min(values), 'max': max(values),
                   'uq_mean': round(statistics.mean(volts), 3),
                   'uq_peak': max(abs(v) for v in volts),
                   'saturation_fraction': round(sum(abs(v) >= a.voltage - .05 for v in volts) / len(volts), 3)}
        step['axes'][str(i)] = metrics
    summary['steps'].append(step)
    print(json.dumps(step), flush=True)
    command_wait('FREQ?', .12)
    for m in step['axes'].values():
        if abs(m['mean'] - m['target']) > max(2, abs(m['target']) * .05) or m['std'] > max(2, abs(m['target']) * .05):
            raise RuntimeError('Steady tracking/ripple outside stage criteria')


try:
    s.open()
    command_wait('PING', .03)
    command_wait('STOP', .15)
    command_wait('APP 0')
    command_wait('MODE 1')
    command_wait(f'U {a.voltage}')
    command_wait('L 180')
    command_wait('ACC 400')
    command_wait('STATE?', .15)
    if state.get('Trip') != '0' or state.get('Fault') != '0' or state.get('EN') != '000':
        raise RuntimeError('Invalid starting state')
    if any(state.get('Aligned', '000')[i - 1] != '1' for i in a.axes):
        raise RuntimeError('Alignment required; no automatic high-speed alignment')
    for group in ([a.axes] if a.dual else [[i] for i in a.axes]):
        active = group
        for i in active:
            command_wait(f'EN{i} 1')
        command_wait('FAST ' + str(sum(1 << (i - 1) for i in active)), .12)
        command_wait('FREQ 1')
        last_fast = last_state_time = time.monotonic()
        watch = True
        for speed in a.speeds:
            move(speed, f'axes{group}_v{speed:g}')
        move(-a.speeds[-1], f'axes{group}_reverse{-a.speeds[-1]:g}')
        move(a.speeds[-1], f'axes{group}_return{a.speeds[-1]:g}')
        if a.modulation:
            for cycle in range(20):
                phase = f'axes{group}_modulation{cycle}'
                target = max(0, a.speeds[-1] - 30) if cycle % 2 == 0 else a.speeds[-1]
                for i in active:
                    cmd(f'M{i} {target if i == active[0] else -target}')
                pump(.15)
        move(0, f'axes{group}_zero')
        command_wait('FREQ?', .15)
        watch = False
        for i in active:
            command_wait(f'EN{i} 0')
        command_wait('FAST 0')
        phase = 'cooldown'
        pump(2)
    summary['result'] = 'passed'
except BaseException as e:
    summary['result'] = 'stopped'
    summary['error'] = repr(e)
    print(json.dumps({'error': repr(e)}), flush=True)
    raise
finally:
    watch = False
    phase = 'finish'
    if s.is_open:
        try:
            cmd('STOP')
            pump(.2)
            command_wait('FAST 0')
            command_wait('U 2.5')
            command_wait('L 15')
            command_wait('STATE?', .15)
            summary['final_state'] = state
        except Exception as e:
            summary['cleanup_error'] = repr(e)
        finally:
            s.close()
    (folder / (a.tag + '_summary.json')).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    log.close()
