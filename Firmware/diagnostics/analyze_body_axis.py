"""Identify the dominant IMU rotation axis from a disabled passive serial log."""
import argparse
import json
from pathlib import Path

import numpy as np


def analyze(path):
    samples = []
    for row in map(json.loads, Path(path).read_text(encoding='utf-8').splitlines()):
        line = row.get('data', '')
        if row.get('kind') != 'rx' or not isinstance(line, str):
            continue
        if not line.startswith(('$TELE,', '$STATE,')) or not line.endswith('#'):
            continue
        data = dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))
        if data.get('EN') != '000':
            raise ValueError('Motor enable present: this must be a passive recording')
        if line.startswith('$TELE,'):
            samples.append([float(data[key]) for key in ('Gx', 'Gy', 'Gz')])
    gyro = np.asarray(samples, dtype=float)
    if len(gyro) < 100 or not np.isfinite(gyro).all():
        raise ValueError('Insufficient valid gyro samples')
    covariance = np.cov(gyro, rowvar=False)
    values, vectors = np.linalg.eigh(covariance)
    principal = vectors[:, -1]
    axis = int(np.argmax(np.abs(principal)))
    if principal[axis] < 0:
        principal = -principal
    rms = np.sqrt(np.maximum(np.diag(covariance), 0))
    purity = float(values[-1] / max(values.sum(), 1e-12))
    accepted = bool(rms[axis] >= 2 and purity >= .95 and abs(principal[axis]) >= .97)
    return {'samples': len(gyro), 'gyro_std_dps': rms.tolist(),
            'principal_axis_xyz': principal.tolist(), 'variance_fraction': purity,
            'candidate_body_axis': 'XYZ'[axis], 'candidate_axis_command': axis + 1,
            'axis_accepted': accepted, 'body_torque_direction_verified': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    result = analyze(args.log)
    args.log.with_name(args.log.stem + '_axis.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result))
