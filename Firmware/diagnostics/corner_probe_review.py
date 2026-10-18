"""Offline paired corner-pulse response review. Never opens a serial port.

Ratios are approximate (filtered encoder speeds, gravity drift, contact torque).
Physical consistency checks can reject a model; passing is not a calibration.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def measure(summary):
    probe = summary.get('probe', {})
    if summary.get('result') != 'probe_completed':
        raise ValueError('Pulse did not complete; exclude protected/partial trials')
    motor, axis = int(probe['motor']), int(probe['body_axis'])
    if motor not in (1, 2, 3) or axis not in (1, 2, 3):
        raise ValueError('Invalid motor/axis')
    volts, duration = float(probe['voltage']), int(probe['duration_ms'])
    if not 0 < abs(volts) <= 3 or not 20 <= duration <= 80:
        raise ValueError('Outside supervised pulse limits')
    if summary.get('last_state', {}).get('EN') != '000':
        raise ValueError('Final disarm not confirmed')
    baseline, samples = summary.get('probe_baseline', []), probe.get('samples', [])
    if len(baseline) < 3 or len(samples) < 3:
        raise ValueError('Insufficient baseline/pulse samples')
    gyro = lambda data: np.array([float(data[k]) for k in ('Wx', 'Wy', 'Wz')])
    before = np.mean([gyro(d) for d in baseline], axis=0)
    after = np.mean([gyro(d) for d in samples[-2:]], axis=0)
    wheel_before = np.mean([float(d[f'V{motor}']) for d in baseline])
    wheel_after = np.mean([float(d[f'V{motor}']) for d in samples[-2:]])
    delta = wheel_after-wheel_before
    if not np.all(np.isfinite([*before, *after, delta])) or abs(delta) < 3:
        raise ValueError('Non-finite or insufficient wheel excitation')
    if delta*volts <= 0:
        raise ValueError('Wheel response opposes commanded voltage; mapping needs review')
    return {'motor': motor, 'body_axis': axis, 'voltage': volts, 'duration_ms': duration,
            'gyro_baseline_rad_s': before.tolist(), 'gyro_delta_rad_s': (after-before).tolist(),
            'wheel_delta_rad_s': float(delta),
            'baseline_body_rate_rad_s': float(np.linalg.norm(before)), 'samples': len(samples)}


def paired_review(measurements):
    pairs, missing = [], []
    for motor in (1, 2, 3):
        positive = [m for m in measurements if m['motor'] == motor and m['voltage'] > 0]
        negative = [m for m in measurements if m['motor'] == motor and m['voltage'] < 0]
        if not positive or not negative:
            missing.append(motor)
            continue
        runs = positive+negative
        axes = {m['body_axis'] for m in runs}
        settings = {(abs(m['voltage']), m['duration_ms']) for m in runs}
        if len(axes) != 1 or len(settings) != 1:
            raise ValueError('Paired pulses need identical axis, amplitude and duration')
        gy = lambda values: np.mean([m['gyro_delta_rad_s'] for m in values], axis=0)
        w = lambda values: np.mean([m['wheel_delta_rad_s'] for m in values])
        response = (gy(positive)-gy(negative))/(w(positive)-w(negative))
        axis = axes.pop()
        diagonal = response[axis-1]
        if abs(diagonal) < .001:
            raise ValueError('Controlled-axis excitation too small to infer direction')
        direction = -1 if diagonal > 0 else 1
        ratios = [np.asarray(m['gyro_delta_rad_s'])/m['wheel_delta_rad_s'] for m in runs]
        spread = max(float(np.linalg.norm(r-response)) for r in ratios)
        pairs.append({'motor': motor, 'body_axis': axis, 'response_rad_per_wheel_rad': response.tolist(),
                      'candidate_body_direction': direction,
                      'single_pulse_ratio_spread_norm': spread, 'runs': len(runs)})
    result = {'paired_motors': pairs, 'missing_paired_motors': missing, 'complete_three_axis_pairs': not missing}
    if missing:
        return result
    if sorted(p['body_axis'] for p in pairs) != [1, 2, 3]:
        raise ValueError('Body axes must form a permutation')
    response = np.zeros((3, 3))
    for p in pairs:
        # Convert wheel signs to positive body torque convention before checking
        # reciprocity. Do not change firmware directions based on this review.
        response[:, p['body_axis']-1] = -p['candidate_body_direction']*np.asarray(p['response_rad_per_wheel_rad'])
    symmetric = (response+response.T)/2
    eigen = np.linalg.eigvalsh(symmetric)
    skew = float(np.linalg.norm(response-response.T)/max(np.linalg.norm(response), 1e-12))
    result.update(response_body_torque_convention=response.tolist(),
                  relative_antisymmetry=skew, symmetric_response_eigenvalues=eigen.tolist(),
                  inertia_consistency_passed=False)
    if np.min(eigen) > 0:
        inertia = np.linalg.inv(symmetric)
        moments = np.linalg.eigvalsh(inertia)
        triangle = bool(moments[-1] <= moments[0]+moments[1])
        result.update(approximate_inertia_over_wheel_inertia=inertia.tolist(),
                      principal_moments=moments.tolist(), inertia_triangle_ok=triangle,
                      inertia_consistency_passed=triangle and skew < .25)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summaries', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    accepted, excluded = [], []
    for path in args.summaries:
        try:
            data = measure(json.loads(path.read_text(encoding='utf-8')))
            data['source'] = str(path)
            accepted.append(data)
        except (ValueError, KeyError, TypeError) as error:
            excluded.append({'source': str(path), 'reason': str(error)})
    result = {'method': 'Positive/negative paired gyro changes divided by paired wheel-speed changes',
              'measurements': accepted, 'excluded': excluded, **paired_review(accepted),
              'limits': ['Gravity drift and hand release latency do not cancel exactly.',
                         'Different wheel filters and contact torsion bias the inferred matrix.',
                         'Passing physical checks is necessary, not sufficient, for a calibrated model.',
                         'No gains, references or mechanical directions are changed automatically.']}
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
