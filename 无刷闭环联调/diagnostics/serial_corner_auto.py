"""Long-run automatic corner (single-point) balance; STOP on every exit.

Arms BAUTO for corner mode: the MCU waits for the still corner pose, counts
down, balances, and after a 20 deg trip or 3 s full-speed event disarms and
waits for the next entry by itself. Tilt overspeed/body-rate/hard faults exit
automatic mode and need a manual restart. This script only observes, records
every attempt and re-entry, and guards the run with backstops above the
firmware thresholds; it never lengthens the firmware protections.
"""
import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
from live_pose import PosePublisher
from serial_monitor_handoff import SerialMonitorHandoff

VERIFIED_REFERENCE = [-0.7028287890572913, 0.3977869310324257, 0.5897433770480731]


def gravity(data):
    pitch, roll = math.radians(float(data['Pit'])), math.radians(float(data['Rol']))
    return [-math.sin(roll) * math.cos(pitch), math.sin(pitch), math.cos(roll) * math.cos(pitch)]


def tilt(left, right):
    dot = sum(a * b for a, b in zip(left, right))
    return math.degrees(math.acos(max(-1, min(1, dot))))


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='COM7')
parser.add_argument('--tag', required=True)
parser.add_argument('--countdown', type=int, default=2, help='BAUTO countdown seconds, 2..15')
parser.add_argument('--capture-angle', type=float, default=.5, help='Total entry tilt in degrees, 0.5..3')
parser.add_argument('--duration-minutes', type=float, default=0, help='0 keeps running until interrupted')
parser.add_argument('--wait-seconds', type=float, default=180, help='Deadline for the first auto entry')
parser.add_argument('--voltage', type=float, default=6)
parser.add_argument('--wheel-limit', type=float, default=330, help='Firmware L overspeed exit; must exceed --full-speed')
parser.add_argument('--full-speed', type=float, default=250, help='BFULL three-second protection threshold')
parser.add_argument('--wheel-guard', type=float, default=360, help='Host backstop above the firmware overspeed exit')
parser.add_argument('--tilt-guard', type=float, default=28, help='Host backstop during balancing, above the 20 deg trip')
parser.add_argument('--rate-guard', type=float, default=4.0, help='Host backstop during balancing, above the 3.0 rad/s trip')
parser.add_argument('--kp', type=float, default=80)
parser.add_argument('--kd', type=float, default=5)
parser.add_argument('--kw', type=float, nargs=3, default=[-.030, -.030, -.030],
                    help='Wheel-speed feedback per axis, -0.5..0.5. Default -0.03 retains the confirmed '
                         'five-second corner configuration. Later Kw=0/+0.03 trials failed; the withdrawn '
                         'offline model is not a tuning authority.')
parser.add_argument('--ki', type=float, default=0.0,
                    help='Wheel-speed integral for all axes, -0.5..0.5. Default 0 preserves the confirmed '
                         'configuration; other values remain experimental.')
parser.add_argument('--yaw-damping', type=float, default=3)
parser.add_argument('--tilt-trim', type=float, default=0.0,
                    help='Experimental corner attitude integral BTRIM, 0..10 V/(rad*s). Default 0 matches '
                         'the confirmed controller. Does not identify the neutral COM pose. Requires TrimTune:1.')
parser.add_argument('--body-axes', type=int, nargs=3, choices=(1, 2, 3), default=[2, 3, 1])
parser.add_argument('--directions', type=int, nargs=3, choices=(-1, 1), default=[-1, -1, -1])
parser.add_argument('--reference-gravity', type=float, nargs=3, default=list(VERIFIED_REFERENCE),
                    help='Fixed corner target; pass --capture-pose to BZERO instead')
parser.add_argument('--capture-pose', action='store_true', help='Capture the hand-held pose instead of the saved reference')
parser.add_argument('--prepare-only', action='store_true', help='Configure and verify, then STOP without arming')
parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent / '2026-10-15')
parser.add_argument('--live-state', type=Path)
args = parser.parse_args()
if not (2 <= args.countdown <= 15 and .5 <= args.capture_angle <= 3
        and args.duration_minutes >= 0 and 30 <= args.wait_seconds <= 1800
        and 0 < args.voltage <= 6 and 30 <= args.full_speed < args.wheel_limit <= 360
        and args.wheel_limit < args.wheel_guard <= 500
        and 20 < args.tilt_guard <= 40 and 3 <= args.rate_guard <= 6
        and 0 <= args.kp <= 120 and 0 <= args.kd <= 10 and all(-.5 <= v <= .5 for v in args.kw)
        and -0.5 <= args.ki <= 0.5
        and 0 <= args.yaw_damping <= 10 and 0 <= args.tilt_trim <= 10 and sorted(args.body_axes) == [1, 2, 3]):
    parser.error('Corner auto bounds exceeded')
if args.reference_gravity is not None:
    norm = math.sqrt(sum(v * v for v in args.reference_gravity))
    if not all(math.isfinite(v) for v in args.reference_gravity) or not .9 <= norm <= 1.1:
        parser.error('Reference must be finite and normalized')
    args.reference_gravity = [v / norm for v in args.reference_gravity]
    if min(abs(v) for v in args.reference_gravity) < .25:
        parser.error('Reference too close to an edge/face')
if args.capture_pose:
    args.reference_gravity = None

folder = args.output_dir
folder.mkdir(parents=True, exist_ok=True)
log = (folder / (args.tag + '.jsonl')).open('x', encoding='utf-8')
summary = {'utc_started': datetime.now(timezone.utc).isoformat(),
           'settings': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
           'result': 'incomplete', 'reference': None, 'attempts': [],
           'hands_off_verified': False, 'cable_free_verified': False}
link = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.3)
link.dtr = False
link.rts = False
link.port = args.port
start = time.monotonic()
phase = 'connect'
buffer = bytearray()
next_ping = 0
next_state_poll = 0
state = {}
revision = 0
tele = {}
segments = []
current = None
last_ended = None
messages = []
reference = None
watch = False
run_started = None
first_entry_deadline = None
run_deadline = None
last_tele = 0
last_report = 0
publisher = PosePublisher(args.live_state, capture_angle=args.capture_angle)


def record(kind, data):
    log.write(json.dumps({'t': time.monotonic() - start, 'phase': phase,
                          'kind': kind, 'data': data}, ensure_ascii=False) + '\n')
    log.flush()


def send(command):
    link.write((command + '\r\n').encode('ascii'))
    record('tx', command)


def begin_segment(now):
    global current, run_started, phase
    current = {'started_t': now - start, 'started_mono': now, 'max_tilt': 0.0, 'max_rate': 0.0,
               'wheel_abs_max': [0.0, 0.0, 0.0], 'samples': 0}
    segments.append(current)
    if run_started is None:
        run_started = now
        phase = 'corner_run'
    print('BALANCING: PG6 on, release now', flush=True)


def end_segment(now):
    global current, last_ended, next_state_poll
    if current is None:
        return
    current['duration_s'] = now - current.pop('started_mono')
    current['ended_t'] = now - start
    current['end_reason'] = None
    last_ended = current
    current = None
    next_state_poll = 0


def close_segment_data(data):
    if current is None:
        return
    current['samples'] += 1
    current['max_tilt'] = max(current['max_tilt'], tilt(gravity(data), reference))
    current['max_rate'] = max(current['max_rate'],
                              math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180)
    for i in range(3):
        current['wheel_abs_max'][i] = max(current['wheel_abs_max'][i], abs(float(data[f'S{i+1}'])))
        if f'T{i+1}' in data:
            current.setdefault('trim_last', [0.0, 0.0, 0.0])[i] = float(data[f'T{i+1}'])


def pump(duration):
    global buffer, next_ping, next_state_poll, state, revision, tele, last_tele, last_report, last_ended
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_ping:
            send('PING')
            next_ping = now + .25
        if watch and now >= next_state_poll:
            send('STATE?')
            next_state_poll = now + 1
        buffer.extend(link.read(link.in_waiting or 1))
        while b'\n' in buffer:
            line, _, buffer = buffer.partition(b'\n')
            line = line.decode('ascii', 'replace').strip()
            if not line:
                continue
            record('rx', line)
            messages.append(line)
            if phase != 'finish' and line.startswith('ERR') and line != 'ERR SAFETY_TRIP:1':
                raise RuntimeError(line)
            if not line.startswith(('$STATE,', '$TELE,', '$FAST,')) or not line.endswith('#'):
                continue
            data = dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))
            target_vector = reference or args.reference_gravity
            if target_vector is not None:
                publisher.target = [math.degrees(math.asin(target_vector[1])),
                                    math.degrees(math.atan2(-target_vector[0], target_vector[2]))]
            publisher.update(data)
            if line.startswith('$STATE,'):
                state = data
                revision += 1
                if last_ended is not None:
                    last_ended['end_reason'] = int(data.get('BReason', 0))
                    last_ended['end_wheel_max_ms'] = int(data.get('FullMs', 0))
                    last_ended['retry_ms'] = int(data.get('RetryMs', 0))
                    last_ended = None
            elif line.startswith('$TELE,'):
                tele = data
                last_tele = now
                recent_frames.append((now, data))
                while recent_frames and now - recent_frames[0][0] > 1:
                    recent_frames.pop(0)
            else:
                if not all(k in data for k in ('V1', 'V2', 'V3', 'Pit', 'Rol', 'Wx', 'Wy', 'Wz', 'Bal')):
                    raise RuntimeError('Incomplete corner FAST attitude')
                last_tele = now
                for i, key in enumerate(('Wx', 'Wy', 'Wz'), 1):
                    data[f'S{i}'] = data[f'V{i}']
                    data[('Gx', 'Gy', 'Gz')[i - 1]] = str(float(data[key]) * 180 / math.pi)
            if watch:
                if int(data.get('Fault', 0)) or int(data.get('Trip', 0)):
                    raise RuntimeError('Firmware fault/trip: ' + line)
                if line.startswith('$STATE,') and data.get('Bal') == '4':
                    raise RuntimeError('Firmware fall/fault protection; automatic mode exited')
                if data.get('EN', '000') not in ('000', '111'):
                    raise RuntimeError('Partial three-axis enable: ' + line)
                if line.startswith('$FAST,'):
                    if not all(math.isfinite(float(data[k])) for k in ('Pit', 'Rol', 'Gx', 'Gy', 'Gz', 'S1', 'S2', 'S3')):
                        summary['guard_frame'] = data
                        raise RuntimeError('Non-finite corner telemetry')
                    balancing = data.get('Bal') == '3'
                    if balancing and current is None:
                        begin_segment(now)
                    elif not balancing and current is not None:
                        end_segment(now)
                    if current is not None:
                        close_segment_data(data)
                        wheel_max = max(abs(float(data[f'S{i}'])) for i in (1, 2, 3))
                        rate = math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180
                        if wheel_max > args.wheel_guard:
                            summary['guard_frame'] = data
                            raise RuntimeError('Corner wheel speed backstop')
                        if tilt(gravity(data), reference) > args.tilt_guard:
                            summary['guard_frame'] = data
                            raise RuntimeError('Corner tilt backstop')
                        if rate > args.rate_guard:
                            summary['guard_frame'] = data
                            raise RuntimeError('Corner body rate backstop')
                if now - last_report >= 5:
                    last_report = now
                    duration = (now - run_started) if run_started else 0
                    print(now - start, 's | Bal', state.get('Bal'), '| Auto', state.get('Auto'),
                          '| BReason', state.get('BReason'), '| attempts', len(segments),
                          '| balance time', duration, flush=True)
        if len(buffer) > 65536:
            raise RuntimeError('Serial overflow')
        if watch and now - last_tele > .35:
            raise RuntimeError('Corner attitude telemetry timeout')


def command(text, seconds=.12):
    send(text)
    pump(seconds)


def confirm(predicate, label, timeout=2):
    # STATE? responses are silently dropped by the firmware when the TX buffer
    # is congested (FAST telemetry), so re-query until the deadline instead of
    # relying on a single request.
    deadline = time.monotonic() + timeout
    next_query = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_query:
            send('STATE?')
            next_query = now + .2
        before = revision
        pump(.05)
        if revision > before and predicate(state):
            return
    raise RuntimeError(label + ': fresh STATE confirmation failed')


def still_corner():
    samples = [d for t, d in recent_frames if time.monotonic() - t <= .5]
    if len(samples) < 5 or time.monotonic() - last_tele > .15:
        return False
    ref = gravity(samples[-1])
    if min(abs(v) for v in ref) < .25:
        return False
    for data in samples:
        g = gravity(data)
        rate = math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180
        if (data.get('EN') != '000' or tilt(g, ref) > 2
                or rate > .1 or max(abs(float(data[f'S{i}'])) for i in (1, 2, 3)) > 2):
            return False
    return True


recent_frames = []


handoff = SerialMonitorHandoff(args.port)
try:
    handoff.open(link)
    command('PING')
    command('STOP')
    command('FAST 0')
    command('TELE 1')
    command('APP 0')
    confirm(lambda d: d.get('Aligned') == '111' and d.get('Cal') == '1'
            and d.get('IMU') == '1' and d.get('EN') == '000' and d.get('Fault') == '0',
            'Three alignments/static IMU calibration')
    if state.get('NTune') in ('1','2','3'):
        command('BNEUTRAL 0')
        confirm(lambda d: d.get('NOn') == '0', 'Neutral search disabled for baseline auto')
    if state.get('CornerStart') != '1' or state.get('YawTune') != '1':
        raise RuntimeError('Corner automatic firmware (CornerStart/YawTune) required')
    if state.get('TrimTune') != '1':
        raise RuntimeError('Explicit tilt trim configuration requires updated firmware (TrimTune)')
    for text in ('APP 1', 'BMODE 4', 'BAXIS ' + ' '.join(map(str, args.body_axes)),
                 'BDIR ' + ' '.join(map(str, args.directions)), f'U {args.voltage}',
                 f'L {args.wheel_limit}', f'BU {args.voltage}', 'BTRIP 20', f'BCAP {args.capture_angle:g}'):
        command(text)
    for i, kw in enumerate(args.kw, 1):
        command(f'LQR{i} {args.kp} {args.kd} {kw} {args.ki}')
    if state.get('ATune') == '1':
        command('BATTAU .25')
        confirm(lambda d: abs(float(d.get('AttTau', -1))-.25) < .005, 'Baseline attitude correction')
    command(f'BYAW {args.yaw_damping}')
    command(f'BTRIM {args.tilt_trim}')
    command(f'BFULL {args.full_speed}')
    confirm(lambda d: d.get('BMode') == '4' and d.get('AxisMask') == '7' and d.get('DirMask') == '7'
            and all(d.get(f'Axis{i}') == str(args.body_axes[i - 1])
                    and d.get(f'D{i}') == str(args.directions[i - 1]) for i in (1, 2, 3))
            and abs(float(d.get('VLimit', 0)) - args.voltage) < .11
            and abs(float(d.get('SLimit', 0)) - args.wheel_limit) < .11
            and abs(float(d.get('FullSpeed', 0)) - args.full_speed) < .11
            and abs(float(d.get('YawD', 0)) - args.yaw_damping) < .011
            and d.get('TrimTune') == '1'
            and abs(float(d.get('BTrim', -1)) - args.tilt_trim) < .011
            and abs(float(d.get('CapDeg', -1)) - args.capture_angle) < .011
            and d.get('TripDeg') == '20.0' and d.get('EN') == '000', 'Corner automatic configuration')
    phase = 'capture_reference'
    if args.reference_gravity is not None:
        command('BREF ' + ' '.join(f'{v:.9f}' for v in args.reference_gravity), .2)
        reference = args.reference_gravity
    else:
        command('FAST 7')
        deadline = time.monotonic() + 60
        while not still_corner():
            if time.monotonic() >= deadline:
                raise RuntimeError('Corner pose not stationary within 60s')
            pump(.05)
        command('BZERO', .2)
        reference = gravity(tele)
    confirm(lambda d: d.get('Ref') == '1' and d.get('BMode') == '4' and d.get('EN') == '000', 'Corner reference')
    summary['reference'] = {'gravity': reference,
                            'pitch_deg': math.degrees(math.asin(reference[1])),
                            'roll_deg': math.degrees(math.atan2(-reference[0], reference[2])),
                            'neutral_verified': False}
    if args.prepare_only:
        summary['result'] = 'prepared_disabled'
    else:
        phase = 'arm_auto'
        # Arm with FAST off so the STATE confirmation is reliable, then enable
        # the corner FAST stream for the run itself.
        command(f'BAUTO 1 {args.countdown}')
        confirm(lambda d: d.get('Auto') == '1' and d.get('Bal') == '6' and d.get('EN') == '000', 'Corner BAUTO')
        command('FAST 7')
        watch = True
        last_tele = time.monotonic()
        first_entry_deadline = time.monotonic() + args.wait_seconds
        run_deadline = time.monotonic() + args.duration_minutes * 60 if args.duration_minutes else None
        print(f'Auto armed: hold the cube at the corner until PG8 is steady, then PG5 counts {args.countdown}s; '
              'PG6 on means release. Protections re-arm automatically after tilt/full-speed stops.', flush=True)
        while True:
            pump(.01)
            if run_started is None:
                if time.monotonic() >= first_entry_deadline:
                    raise RuntimeError('Automatic corner entry not achieved')
                if state.get('Bal') == '3':
                    run_started = time.monotonic()
                    phase = 'corner_run'
            elif run_deadline is not None and time.monotonic() >= run_deadline:
                summary['result'] = 'timed_run_completed'
                break
except KeyboardInterrupt:
    summary['result'] = 'user_stopped'
    record('error', 'KeyboardInterrupt')
    print('Interrupted by user; stopping safely', flush=True)
except BaseException as error:
    if summary['result'] == 'incomplete':
        summary['result'] = 'stopped'
    summary['error'] = repr(error)
    record('error', repr(error))
    print(repr(error), flush=True)
finally:
    watch = False
    now = time.monotonic()
    if current is not None:
        end_segment(now)
    if last_ended is not None and last_ended.get('end_reason') is None:
        last_ended['end_reason'] = int(state.get('BReason', 0))
        last_ended['end_wheel_max_ms'] = int(state.get('FullMs', 0))
        last_ended = None
    phase = 'finish'
    if link.is_open:
        try:
            command('STOP', .2)
            command('FAST 0')
            command('U 2.5')
            command('L 15')
            confirm(lambda d: d.get('EN') == '000' and d.get('Bal') == '0', 'Final disarm')
            command('FREQ?', .15)
        except BaseException as error:
            summary['result'] = 'stopped'
            summary['final_disarm_error'] = repr(error)
            record('error', repr(error))
        finally:
            link.close()
    handoff.release()
    summary['last_state'] = state
    summary['armed_countdown_s'] = args.countdown
    summary['auto_attempts'] = len(segments)
    summary['attempts'] = segments
    summary['balance_seconds_total'] = sum(s.get('duration_s', 0) for s in segments)
    summary['balance_seconds_longest'] = max([s.get('duration_s', 0) for s in segments] or [0])
    summary['frequency'] = [line for line in messages if line.startswith('$FREQ,')]
    (folder / (args.tag + '_summary.json')).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    log.close()
    print(json.dumps(summary), flush=True)
if summary['result'] not in ('timed_run_completed', 'user_stopped', 'prepared_disabled'):
    raise SystemExit(2)
