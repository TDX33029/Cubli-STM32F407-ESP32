"""Supervised, bounded three-wheel corner trial; STOP on every exit.

Uses MCU countdown and full corner FAST when CornerStart=1 is reported.
Older firmware uses standard TELE and an eight-second host wait without PG5.
Axes/signs are the independently identified installation, not encoder signs.
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
from neutral_resume import validate_previous, matches_saved_state


def gravity(data):
    pitch, roll = math.radians(float(data['Pit'])), math.radians(float(data['Rol']))
    return [-math.sin(roll) * math.cos(pitch), math.sin(pitch), math.cos(roll) * math.cos(pitch)]


def tilt(left, right):
    dot = sum(a * b for a, b in zip(left, right))
    return math.degrees(math.acos(max(-1, min(1, dot))))


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='COM7')
parser.add_argument('--tag', required=True)
parser.add_argument('--seconds', type=float, default=3)
parser.add_argument('--wait-seconds', type=float, default=180, help='Disabled entry wait, 10..600 seconds')
parser.add_argument('--countdown', type=int, default=2)
parser.add_argument('--capture-angle', type=float, default=.5, help='Total entry tilt in degrees, 0.5..3')
parser.add_argument('--voltage', type=float, default=3.5)
parser.add_argument('--wheel-bound', type=float, default=100)
parser.add_argument('--body-rate-bound', type=float, default=1.5, help='Host body angular-rate stop, 1.5..2rad/s; default remains1.5')
parser.add_argument('--kp', type=float, default=80)
parser.add_argument('--kd', type=float, default=5)
parser.add_argument('--kw', type=float, nargs=3, default=[-.030, -.030, -.030])
parser.add_argument('--yaw-damping', type=float, help='Independent corner yaw damping, 0..10; updated firmware required')
parser.add_argument('--attitude-tau', type=float, help='Accelerometer correction time constant, .25..2s; optional observer experiment')
parser.add_argument('--body-axes', type=int, nargs=3, choices=(1, 2, 3), required=True)
parser.add_argument('--directions', type=int, nargs=3, choices=(-1, 1), required=True)
parser.add_argument('--expected-pose', type=Path, required=True, help='Disabled passive pose summary JSON')
parser.add_argument('--reference-gravity', type=float, nargs=3, help='Saved normalized corner target; requires CornerStart firmware')
parser.add_argument('--prepare-only', action='store_true', help='Capture/configure, then STOP without arming')
parser.add_argument('--neutral-learning', action='store_true', help='Bounded slow corner reference search, updated firmware required')
parser.add_argument('--neutral-limit', type=float, default=2, help='Total search cone about original reference, 2..5 degrees')
parser.add_argument('--seed-neutral-from', type=Path, help='Restore saved learned target after reset, retaining the original base')
parser.add_argument('--resume-neutral-from', type=Path, help='Previous summary; retain MCU learned offset and original base after strict fresh-state checks')
parser.add_argument('--neutral-gain', type=float, default=.000025, help='Slow search gain, .000005.. .00025')
parser.add_argument('--neutral-rate', type=float, default=.1, help='Maximum search speed, .01.. .5 degrees/s')
parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent / '2026-10-14')
parser.add_argument('--live-state',type=Path)
parser.add_argument('--probe-motor', type=int, choices=(1, 2, 3), help='Zero-output release then one MCU-timed motor pulse')
parser.add_argument('--probe-voltage', type=float, default=3)
parser.add_argument('--probe-ms', type=int, default=80)
parser.add_argument('--probe-delay', type=float, default=.35, help='Zero-output release interval before timed probe, .1.. .35s')
args = parser.parse_args()
if not (0 < args.seconds <= 15 and 10 <= args.wait_seconds <= 600 and 2 <= args.countdown <= 15 and .5 <= args.capture_angle <= 3
        and 0 < args.voltage <= 6 and 30 <= args.wheel_bound <= 250
        and 1.5 <= args.body_rate_bound <= 2
        and 0 <= args.kp <= 120 and 0 <= args.kd <= 10 and all(abs(v) <= .1 for v in args.kw)
        and sorted(args.body_axes) == [1, 2, 3]):
    parser.error('Corner diagnostic bounds exceeded')
if args.yaw_damping is not None and not 0 <= args.yaw_damping <= 10:
    parser.error('Yaw damping must be 0..10')
if args.attitude_tau is not None and not .25 <= args.attitude_tau <= 2:
    parser.error('Attitude time constant must be .25..2s')
if args.probe_motor and args.attitude_tau not in (None, .25):
    parser.error('Observer experiment cannot be combined with timed probe mode changes')
if args.probe_motor and not (args.reference_gravity is not None and args.kp == 0 and args.kd == 0
        and all(v == 0 for v in args.kw) and args.yaw_damping == 0 and 0 < abs(args.probe_voltage) <= 3
        and abs(args.probe_voltage) <= args.voltage and 20 <= args.probe_ms <= 80
        and .1 <= args.probe_delay <= .35
        and args.wheel_bound <= 60 and args.body_rate_bound == 1.5 and .5 <= args.seconds <= 1.5):
    parser.error('Probe requires fixed reference, all gains/yaw zero, <=3V/80ms and wheel guard <=60rad/s')
expected = json.loads(args.expected_pose.read_text(encoding='utf-8'))
if args.neutral_learning and (args.reference_gravity is None or args.probe_motor):
    parser.error('Neutral search requires a saved base reference and cannot be combined with a probe')
if args.resume_neutral_from and args.seed_neutral_from:
    parser.error('Choose resume or seed, not both')
if (args.resume_neutral_from or args.seed_neutral_from) and not args.neutral_learning:
    parser.error('Resume requires neutral learning and the same original reference/configuration')
if not (.000005 <= args.neutral_gain <= .00025 and .01 <= args.neutral_rate <= .5 and 2 <= args.neutral_limit <= 5):
    parser.error('Neutral tuning bounds exceeded')
expected_g = gravity({key: expected['ranges'][key]['median'] for key in ('Pit', 'Rol')})
publisher=PosePublisher(args.live_state, capture_angle=args.capture_angle)
publisher.trial_id = args.tag
if args.probe_motor:
    publisher.trial_text = f'M{args.probe_motor} {args.probe_voltage:+g}V / {args.probe_ms}ms 短脉冲'
elif args.neutral_learning:
    publisher.trial_text = f'{args.seconds:g}秒单点自动寻点 · 总寻点范围{args.neutral_limit:g}°'
else:
    tau = args.attitude_tau if args.attitude_tau is not None else .25
    publisher.trial_text = f'{args.seconds:g}秒单点对照 · 姿态校正{tau:g}秒'
if args.body_rate_bound != 1.5:
    publisher.trial_text += f' · 机体角速度上限{args.body_rate_bound:g}rad/s'
if min(abs(v) for v in expected_g) < .25:
    parser.error('Expected pose is too close to a face/edge; capture a corner pose first')
if args.reference_gravity is not None:
    norm = math.sqrt(sum(v*v for v in args.reference_gravity))
    if not all(math.isfinite(v) for v in args.reference_gravity) or not .9 <= norm <= 1.1:
        parser.error('Saved reference must be finite and normalized')
    args.reference_gravity = [v / norm for v in args.reference_gravity]
    if min(abs(v) for v in args.reference_gravity) < .25:
        parser.error('Saved corner reference too close to an edge/face')

folder = args.output_dir
folder.mkdir(parents=True, exist_ok=True)
log = (folder / (args.tag + '.jsonl')).open('x', encoding='utf-8')
summary = {'utc_started': datetime.now(timezone.utc).isoformat(),
           'settings': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
           'result': 'incomplete', 'reference': None, 'hands_off_verified': False,
           'cable_free_verified': False, 'active_samples': 0}
link = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.3)
link.dtr = False
link.rts = False
link.port = args.port
start = time.monotonic()
phase = 'connect'
buffer = bytearray()
next_ping = 0
state = {}
revision = 0
tele = {}
recent = []
active = []
probe_samples = []
messages = []
reference = None
watch = False
run_started = None
last_tele = 0
last_pose = None
last_pose_time = 0


def record(kind, data):
    log.write(json.dumps({'t': time.monotonic() - start, 'phase': phase,
                          'kind': kind, 'data': data}, ensure_ascii=False) + '\n')
    log.flush()


def send(command):
    link.write((command + '\r\n').encode('ascii'))
    record('tx', command)


def pump(duration):
    global buffer, next_ping, state, revision, tele, last_tele, run_started, phase, last_pose, last_pose_time
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_ping:
            send('PING')
            next_ping = now + .25
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
            if phase == 'finish' and line.startswith('$FAST,'):
                # Faulted edge FAST intentionally omits gyro/pose. A queued
                # packet must never interrupt STOP/FAST0/fresh STATE cleanup.
                continue
            data = dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))
            if 'Pit' in data and 'Rol' in data:
                last_pose = {key:data[key] for key in ('Pit','Rol')}
                last_pose_time = now
            target_vector = reference or args.reference_gravity or expected_g
            publisher.target=[math.degrees(math.asin(target_vector[1])),math.degrees(math.atan2(-target_vector[0],target_vector[2]))]
            publisher.remaining_s = (max(0, args.seconds - (now-run_started)) if run_started is not None
                                     else args.seconds) if data.get('Bal') == '3' else None
            publisher.update(data)
            if line.startswith('$STATE,'):
                state = data
                revision += 1
            elif line.startswith('$TELE,'):
                tele = data
                last_tele = now
                recent.append((now, data))
                while recent and now - recent[0][0] > 1:
                    recent.pop(0)
            else:
                if watch and args.neutral_learning and not all(k in data and math.isfinite(float(data[k])) for k in ('NPit','NRol')):
                    raise RuntimeError('Missing/non-finite learned reference telemetry')
                if watch and args.neutral_learning and tilt(reference, gravity({'Pit':data['NPit'],'Rol':data['NRol']}))>args.neutral_limit+.03:
                    summary['guard_frame'] = data
                    raise RuntimeError('Neutral reference bound')
                probing = args.probe_motor is not None and (phase.startswith('probe_') or phase == 'finish')
                if probing and data.get('Bal') == '0':
                    # Disabled edge FAST has no full gyro; TELE still supplies pose.
                    continue
                required = ('V1','V2','V3','Wx','Wy','Wz','Bal') if probing else ('V1','V2','V3','Pit','Rol','Wx','Wy','Wz','Bal')
                if not all(k in data for k in required):
                    raise RuntimeError('Incomplete corner FAST attitude')
                if probing and not all(k in data for k in ('Pit','Rol')):
                    # Corner FAST can still be queued while switching to edge
                    # probe mode. Preserve its actual pose; never overwrite it
                    # with a TELE frame from before the supported-corner entry.
                    if last_pose is None or now-last_pose_time > .2:
                        raise RuntimeError('Timed probe pose telemetry timeout')
                    data.update(last_pose)
                last_tele = now
                for i, key in enumerate(('Wx','Wy','Wz'), 1):
                    data[f'S{i}'] = data[f'V{i}']
                    data[('Gx','Gy','Gz')[i-1]] = str(float(data[key]) * 180 / math.pi)
                if probing and data.get('Bal') == '7':
                    probe_samples.append(dict(data))
            if watch:
                if int(data.get('Fault', 0)) or int(data.get('Trip', 0)) or data.get('Bal') == '4':
                    raise RuntimeError('Firmware protection: ' + line)
                allowed_en = ('000', '111')
                if args.probe_motor and phase.startswith('probe_'):
                    mask = ''.join('1' if i == args.probe_motor else '0' for i in (1, 2, 3))
                    allowed_en = ('000', mask, '111') if phase == 'probe_prepare' else ('000', mask)
                if data.get('EN', '000') not in allowed_en:
                    raise RuntimeError('Partial three-axis enable: ' + line)
                if line.startswith(('$TELE,', '$FAST,')):
                    if not all(math.isfinite(float(data[k])) for k in ('Pit','Rol','Gx','Gy','Gz','S1','S2','S3')):
                        summary['guard_frame']=data
                        raise RuntimeError('Non-finite corner telemetry')
                    if max(abs(float(data[f'S{i}'])) for i in (1, 2, 3)) > args.wheel_bound:
                        summary['guard_frame'] = data
                        raise RuntimeError('Corner wheel speed bound')
                    rate = math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180
                    # Moving a disabled cube towards the target is expected.
                    # The MCU owns pose/stillness checks during its countdown.
                    if (data.get('Bal') in ('3', '7') or data.get('EN') == '111') and (rate > args.body_rate_bound or tilt(gravity(data), reference) > 10):
                        summary['guard_frame']=data
                        raise RuntimeError('Corner body rate/tilt bound')
                    if data.get('Bal') == '3':
                        if run_started is None:
                            run_started = now
                            phase='corner_run'
                            if not args.probe_motor:
                                send('FREQ 7')
                            print('BALANCING: PG6 on, release now', flush=True)
                        active.append(data)
        if len(buffer) > 65536:
            raise RuntimeError('Serial overflow')
        if watch and now - last_tele > .35:
            raise RuntimeError('Corner attitude telemetry timeout')


def command(text, seconds=.12):
    send(text)
    pump(seconds)


def confirm(predicate, label, timeout=1):
    previous = revision
    next_query = 0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if time.monotonic() >= next_query:
            send('STATE?')
            next_query = time.monotonic() + .1
        pump(.02)
        if revision > previous and predicate(state):
            return
    raise RuntimeError(label + ': fresh STATE confirmation failed')


def ready_for_reference():
    samples = [d for t, d in recent if time.monotonic() - t <= .5]
    if len(samples) < 5 or time.monotonic() - last_tele > .15:
        return False
    for data in samples:
        if data.get('EN') != '000' or max(abs(float(data[f'S{i}'])) for i in (1, 2, 3)) > 2:
            return False
        if args.reference_gravity is not None:
            # A saved target does not need to be recaptured from a hand-held
            # pose. Keep idle-wheel/fresh-data checks, then let BSTART gate entry.
            continue
        g = gravity(data)
        rate = math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180
        if tilt(g, expected_g) > 6 or min(abs(v) for v in g) < .25 or rate > .1:
            return False
    return True


handoff = SerialMonitorHandoff(args.port)
try:
    handoff.open(link)
    command('PING')
    command('STOP')
    command('FAST 0')
    command('TELE 1')
    if not args.resume_neutral_from:
        command('APP 0')
    confirm(lambda d: d.get('Aligned') == '111' and d.get('Cal') == '1'
            and d.get('IMU') == '1' and d.get('EN') == '000' and d.get('Fault') == '0',
            'Three alignments/static IMU calibration')
    if args.neutral_learning and (state.get('NTune') not in ('2','3') or
            ((args.neutral_limit != 2 or args.seed_neutral_from) and state.get('NTune') != '3')):
        raise RuntimeError('Neutral search requires compatible firmware (NTune:3 for range/seed)')
    previous_search = None
    if args.resume_neutral_from:
        previous_search = json.loads(args.resume_neutral_from.read_text(encoding='utf-8'))
        validate_previous(previous_search,args)
        if not matches_saved_state(state,previous_search):
            raise RuntimeError('Resume device state changed/reset; learned offset not reused')
        summary['resumed_from']=str(args.resume_neutral_from)
        summary['neutral_start_state']=dict(state)
    elif state.get('NTune') in ('1','2','3'):
        command('BNEUTRAL 0')
        confirm(lambda d: d.get('NOn') == '0', 'Neutral search disabled before configuration')
    mapping_commands = () if previous_search else ('BMODE 4','BAXIS ' + ' '.join(map(str,args.body_axes)),
                                                   'BDIR ' + ' '.join(map(str,args.directions)))
    for text in ('APP 1', *mapping_commands, f'U {args.voltage}',
                 f'L {args.wheel_bound + 20}', f'BU {args.voltage}', 'BTRIP 20', f'BCAP {args.capture_angle:g}', 'FREQ 7'):
        command(text)
    for i, kw in enumerate(args.kw, 1):
        command(f'LQR{i} {args.kp} {args.kd} {kw} 0')
    if args.attitude_tau is not None and state.get('ATune') != '1':
        raise RuntimeError('Attitude tuning requires updated firmware (ATune)')
    if state.get('ATune') == '1':
        expected_tau = args.attitude_tau if args.attitude_tau is not None else .25
        command(f'BATTAU {expected_tau:g}')
        confirm(lambda d: abs(float(d.get('AttTau', -1))-expected_tau) < .005, 'Attitude correction time constant')
    if args.yaw_damping is not None:
        if state.get('YawTune') != '1':
            raise RuntimeError('Independent yaw tuning requires updated firmware')
        command(f'BYAW {args.yaw_damping}')
        confirm(lambda d: abs(float(d.get('YawD', -1)) - args.yaw_damping) < .011, 'Yaw damping')
    # A finite baseline trial must also clear an integral left by a previous
    # experimental session. Legacy firmware predates this feature entirely.
    if state.get('TrimTune') == '1':
        command('BTRIM 0')
        confirm(lambda d: abs(float(d.get('BTrim', -1))) < .011, 'Tilt trim disabled')
    confirm(lambda d: d.get('BMode') == '4' and d.get('AxisMask') == '7' and d.get('DirMask') == '7'
            and all(d.get(f'Axis{i}') == str(args.body_axes[i-1])
                    and d.get(f'D{i}') == str(args.directions[i-1]) for i in (1, 2, 3))
            and abs(float(d.get('VLimit', 0)) - args.voltage) < .11
            and abs(float(d.get('SLimit', 0)) - args.wheel_bound - 20) < .11
            and (d.get('CornerStart') != '1' or abs(float(d.get('CapDeg', -1))-args.capture_angle) < .011)
            and d.get('TripDeg') == '20.0' and d.get('EN') == '000', 'Corner configuration')
    phase = 'wait_idle_wheels' if args.reference_gravity is not None else 'wait_still_corner'
    print('Waiting for idle flywheels; motors disabled' if args.reference_gravity is not None
          else 'Waiting for stationary supported corner; motors disabled', flush=True)
    deadline = time.monotonic() + args.wait_seconds
    while not ready_for_reference():
        if time.monotonic() >= deadline:
            raise RuntimeError('Reference preparation not ready within entry wait')
        pump(.05)
    phase = 'capture_reference'
    firmware_countdown = state.get('CornerStart') == '1'
    if args.reference_gravity is not None:
        if not firmware_countdown:
            raise RuntimeError('Saved corner reference requires updated firmware')
        if not previous_search:
            command('BREF ' + ' '.join(f'{v:.9f}' for v in args.reference_gravity), .2)
        reference = args.reference_gravity
    else:
        command('BZERO', .2)
        reference = gravity(tele)
    confirm(lambda d: d.get('Ref') == '1' and d.get('BMode') == '4'
            and d.get('EN') == '000', 'Corner BZERO')
    summary['reference'] = {'gravity': reference,
                            'pitch_deg': math.degrees(math.asin(reference[1])),
                            'roll_deg': math.degrees(math.atan2(-reference[0],reference[2])),
                            'neutral_verified': False}
    if args.neutral_learning:
        seed_angles = None
        neutral_command = f'BNEUTRAL 1 {args.neutral_gain:g} {args.neutral_rate:g}'
        if state.get('NTune') == '3':
            neutral_command += f' {args.neutral_limit:g}'
        if args.seed_neutral_from:
            seed_summary = json.loads(args.seed_neutral_from.read_text(encoding='utf-8'))
            seed_angles = validate_previous(seed_summary,args,seed=True)
            mapping_keys = ('Map1','Map2','Map3','PP1','PP2','PP3','Dir1','Dir2','Dir3')
            if any(state.get(k)!=seed_summary['last_state'].get(k) for k in mapping_keys):
                raise RuntimeError('Seed motor calibration mapping changed')
            neutral_command += ' ' + ' '.join(f'{v:.6f}' for v in seed_angles)
            summary['seeded_from'] = str(args.seed_neutral_from)
        command(neutral_command)
        confirm(lambda d: d.get('NTune') in ('2','3') and d.get('NOn') == '1'
                and abs(float(d.get('NLimit', -1))-args.neutral_limit) < .011
                and abs(float(d.get('NRate', -1))-args.neutral_rate) < .001
                and abs(float(d.get('NGain', -1))-args.neutral_gain) < .0000006, 'Bounded neutral search enabled')
        if seed_angles:
            confirm(lambda d: all(abs(float(d.get(key,999))-angle)<.011
                    for key,angle in zip(('NPit','NRol'),seed_angles)), 'Restored learned reference')
        if previous_search:
            confirm(lambda d: matches_saved_state(d,previous_search), 'Retained neutral target and original boundary')
        summary['neutral_start_state'] = dict(state)
    if args.prepare_only:
        summary['result'] = 'prepared_disabled'
    else:
        if firmware_countdown:
            phase = 'mcu_countdown'
            command('FAST 7')
            watch = True
            command(f'BSTART {args.countdown}')
            print('MCU wait pose/countdown; PG5 flashes, hold until PG6', flush=True)
            deadline = time.monotonic() + args.wait_seconds
        else:
            phase = 'host_countdown'
            print(f'Host wait {args.countdown}s; PG5 does not blink in legacy corner firmware; hold until PG6', flush=True)
            deadline = time.monotonic() + args.countdown
            while time.monotonic() < deadline:
                pump(.05)
                if (time.monotonic() - last_tele > .2 or tilt(gravity(tele), reference) >= args.capture_angle
                        or tele.get('EN') != '000'):
                    raise RuntimeError('Corner moved during host countdown; not armed')
            phase = 'corner_run'
            watch = True
            command('BAL 1', .05)
            deadline = time.monotonic() + 8
        while run_started is None or time.monotonic() - run_started < args.seconds:
            if run_started is None and time.monotonic() >= deadline:
                raise RuntimeError('Corner takeover not achieved')
            pump(.01)
            if args.probe_motor and run_started is not None and time.monotonic() - run_started >= args.probe_delay:
                summary['probe_baseline'] = [dict(d) for d in active[-6:]]
                phase = 'probe_prepare'
                send('FAST 0')
                send('BAL 0')
                send(f'BMODE {args.probe_motor}')
                confirm(lambda d: d.get('EN') == '000' and d.get('Bal') == '0'
                        and d.get('BMode') == str(args.probe_motor), 'Disarm before timed probe')
                send('FAST 7')
                phase = 'probe_pulse'
                command(f'BPROBE {args.probe_voltage:g} {args.probe_ms}', (args.probe_ms + 40)/1000)
                pump(.1)
                if len(probe_samples) < 3:
                    raise RuntimeError('Insufficient timed probe gyro samples')
                summary['result'] = 'probe_completed'
                break
        if not args.probe_motor:
            summary['result'] = 'bounded_run_completed'
        summary['run_duration_s'] = time.monotonic() - run_started
except BaseException as error:
    summary['result'] = 'stopped'
    summary['error'] = repr(error)
    if run_started is not None:
        summary['run_duration_s'] = time.monotonic() - run_started
    record('error', repr(error))
    print(repr(error), flush=True)
finally:
    watch = False
    publisher.remaining_s = None
    if summary['result'] == 'bounded_run_completed':
        publisher.outcome_text = f'定时结束（{args.seconds:g}秒）'
    elif summary['result'] == 'probe_completed':
        publisher.outcome_text = '短脉冲结束'
    elif summary['result'] == 'prepared_disabled':
        publisher.outcome_text = '配置完成'
    else:
        error_text = summary.get('error', '')
        if 'wheel speed bound' in error_text:
            publisher.outcome_text = '轮速保护停机'
        elif 'body rate/tilt bound' in error_text:
            frame = summary.get('guard_frame', {})
            rate = math.sqrt(sum(float(frame.get(k, 0))**2 for k in ('Gx','Gy','Gz')))*math.pi/180
            publisher.outcome_text = '角速度保护停机' if rate > args.body_rate_bound else '倾角保护停机'
        elif 'Firmware protection' in error_text or 'ERR' in error_text:
            publisher.outcome_text = '固件保护停机'
        elif 'not achieved' in error_text or 'not ready' in error_text:
            publisher.outcome_text = '入位等待结束'
        else:
            publisher.outcome_text = '试验异常停机'
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
            publisher.outcome_text = '停机未确认'
            publisher.remaining_s = None
            publisher.last_write = -1
            publisher.update(state)
            record('error', repr(error))
        finally:
            link.close()
    handoff.release()
    summary['last_state'] = state
    summary['active_samples'] = len(active)
    if args.probe_motor:
        summary['probe'] = {'motor': args.probe_motor, 'voltage': args.probe_voltage,
                            'duration_ms': args.probe_ms, 'samples': probe_samples,
                            'body_axis': args.body_axes[args.probe_motor-1],
                            'direction_verified': False}
    if active:
        summary['tilt_deg_max'] = max(tilt(gravity(d), reference) for d in active)
        if args.neutral_learning:
            learned = [d for d in active if 'NPit' in d and 'NRol' in d]
            if learned:
                effective = lambda d: gravity({'Pit': d['NPit'], 'Rol': d['NRol']})
                summary['effective_tilt_deg_max'] = max(tilt(gravity(d), effective(d)) for d in learned)
                summary['neutral_offset_deg_max'] = max(tilt(reference, effective(d)) for d in learned)
                summary['learned_reference'] = {'gravity': effective(learned[-1]),
                                               'pitch_deg': float(learned[-1]['NPit']),
                                               'roll_deg': float(learned[-1]['NRol']),
                                               'neutral_verified': False}
        summary['wheel_ranges'] = {f'M{i}': [min(float(d[f'S{i}']) for d in active),
                                            max(float(d[f'S{i}']) for d in active)] for i in (1, 2, 3)}
    summary['frequency'] = [line for line in messages if line.startswith('$FREQ,')]
    (folder / (args.tag + '_summary.json')).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    log.close()
    print(json.dumps(summary), flush=True)
if summary['result'] not in ('bounded_run_completed', 'prepared_disabled', 'probe_completed'):
    raise SystemExit(2)
