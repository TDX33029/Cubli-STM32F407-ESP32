"""Summarize active FAST telemetry; contact/hand support cannot be inferred here."""
import argparse
import json
import statistics
from pathlib import Path


def analyze(path, axis):
    active = []
    for row in map(json.loads, path.read_text(encoding='utf-8').splitlines()):
        line = row.get('data', '')
        if row.get('kind') != 'rx' or not isinstance(line, str):
            continue
        if not line.startswith('$FAST,') or not line.endswith('#'):
            continue
        data = dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))
        if data.get('Bal') == '3' and f'V{axis}' in data:
            active.append((row['t'], data))
    if not active:
        raise ValueError('No active balance FAST samples for selected motor')
    result = {'motor': axis, 'samples': len(active),
              'observed_duration_s': active[-1][0] - active[0][0],
              'hands_off_verified': False, 'fixture_free_verified': False,
              'cable_free_verified': False}
    for name, key in [('angle_deg', 'Angle'), ('wheel_rad_s', f'V{axis}'),
                      ('voltage_v', f'U{axis}'), ('body_rate_rad_s', 'Rate')]:
        values = [float(data[key]) for _, data in active]
        result[name] = {'min': min(values), 'max': max(values),
                        'mean': statistics.mean(values), 'last': values[-1]}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--axis', type=int, choices=(1, 2, 3), required=True)
    args = parser.parse_args()
    result = analyze(args.log, args.axis)
    args.log.with_name(args.log.stem + '_metrics.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result))
