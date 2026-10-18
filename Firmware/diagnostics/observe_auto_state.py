"""Read the upper computer's state snapshot; never issue hardware commands."""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--state-file', type=Path, required=True)
p.add_argument('--seconds', type=float, default=60)
p.add_argument('--tag', required=True)
a = p.parse_args()
if not 1 <= a.seconds <= 120:
    raise SystemExit('Observation duration must be 1..120 seconds')
folder = Path(__file__).resolve().parent / '2026-10-14'
rows = []
last_stamp = None
started = time.monotonic()
error = None
read_errors = 0
last_read = started
with (folder / (a.tag + '.jsonl')).open('x', encoding='utf-8') as log:
    try:
        while time.monotonic() - started < a.seconds:
            try:
                data = json.loads(a.state_file.read_text(encoding='utf-8'))
                last_read = time.monotonic()
            except (OSError, json.JSONDecodeError):
                read_errors += 1
                if time.monotonic() - last_read > 2:
                    raise RuntimeError('State snapshot could not be read for2 seconds')
                time.sleep(.05)
                continue
            if time.time() - data['recorded_at'] > 2:
                raise RuntimeError('Upper computer state is stale')
            if data['recorded_at'] != last_stamp:
                last_stamp = data['recorded_at']
                data['observer_elapsed_s'] = time.monotonic() - started
                rows.append(data)
                log.write(json.dumps(data) + '\n')
                log.flush()
            time.sleep(.05)
    except Exception as exc:
        error = repr(exc)
summary = {'hardware_commands_issued': False, 'error': error,'transient_read_errors':read_errors,
           'observation_seconds': time.monotonic() - started, 'samples': len(rows),
           'states': dict(Counter(str(x['state']['Bal']) for x in rows)),
           'all_auto_enabled': bool(rows) and all(x['state'].get('Auto') == 1 for x in rows),
           'm2_m3_always_disabled': bool(rows) and all(x['state']['EN'][1:] == '00' for x in rows),
           'last_state': rows[-1]['state'] if rows else None,
           'ranges': {}}
for key in ('Angle', 'PoseTilt', 'Wheel', 'Rate', 'U1', 'FullMs'):
    values = [float(x['state'][key]) for x in rows if key in x['state']]
    if values:
        summary['ranges'][key] = {'min': min(values), 'max': max(values)}
with (folder / (a.tag + '_summary.json')).open('x', encoding='utf-8') as f:
    json.dump(summary, f, indent=2)
    f.write('\n')
print(json.dumps(summary), flush=True)
if error:
    raise SystemExit(1)
