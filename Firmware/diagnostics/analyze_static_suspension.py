"""Summarize disabled static-suspension telemetry; offline, no serial access."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from corner_com_geometry import unit


def angle(left, right):
    return math.degrees(math.acos(float(np.clip(np.dot(left, right), -1, 1))))


def summarize(path):
    rows, states = [], []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        record = json.loads(line)
        text = record.get('data', '')
        if record.get('kind') != 'rx' or not text.startswith(('$TELE,','$STATE,')) or not text.endswith('#'):
            continue
        fields = dict(item.split(':',1) for item in text.split(',',1)[1][:-1].split(','))
        if fields.get('EN') != '000' or fields.get('Bal') != '0':
            raise ValueError('Static analysis requires every motor disabled and balance off')
        if text.startswith('$STATE,'):
            if fields.get('Auto','0') != '0':
                raise ValueError('Automatic control must be off')
            states.append(fields)
            continue
        values = {k:float(fields[k]) for k in ('Pit','Rol','Ax','Ay','Az','Gx','Gy','Gz','S1','S2','S3')}
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('Non-finite telemetry')
        rows.append((float(record['t']),values))
    if len(rows) < 20:
        raise ValueError('Insufficient disabled samples')
    accel = np.array([[v[k] for k in ('Ax','Ay','Az')] for _,v in rows])
    norms = np.linalg.norm(accel,axis=1)
    raw = np.array([unit(a) for a in accel])
    filtered = []
    for _,v in rows:
        p,r = map(math.radians,(v['Pit'],v['Rol']))
        filtered.append([-math.sin(r)*math.cos(p),math.sin(p),math.cos(r)*math.cos(p)])
    raw_mean, filtered_mean = unit(raw.mean(axis=0)), unit(np.mean(filtered,axis=0))
    scatter = np.array([angle(g,filtered_mean) for g in filtered])
    blocks = []
    first,last = rows[0][0],rows[-1][0]
    for start in np.arange(first,last,5):
        samples = [raw[i] for i,(t,_) in enumerate(rows) if start <= t < start+5]
        if len(samples) >= 20:
            blocks.append({'start_s':float(start),'samples':len(samples),
                           'gravity_up_imu':unit(np.mean(samples,axis=0)).tolist()})
    spread = max((angle(a['gravity_up_imu'],b['gravity_up_imu']) for a in blocks for b in blocks),default=0)
    wheel = max(abs(v[k]) for _,v in rows for k in ('S1','S2','S3'))
    rms = float(np.sqrt(np.mean(scatter**2)))
    return {'source_log':str(path),'samples':len(rows),'duration_s':last-first,
            'all_samples_disabled':True,'last_state':states[-1] if states else {},
            'gravity_up_raw_mean_imu':raw_mean.tolist(),
            'gravity_up_filtered_mean_imu':filtered_mean.tolist(),
            'raw_filtered_difference_deg':angle(raw_mean,filtered_mean),
            'filtered_scatter_rms_deg':rms,'filtered_scatter_p95_deg':float(np.percentile(scatter,95)),
            'five_second_raw_mean_spread_deg':spread,'blocks':blocks,
            'acceleration_norm_mean_g':float(norms.mean()),
            'acceleration_norm_range_g':[float(norms.min()),float(norms.max())],
            'raw_gyro_norm_max_dps':max(float(np.linalg.norm([v[k] for k in ('Gx','Gy','Gz')])) for _,v in rows),
            'wheel_abs_max_rad_s':wheel,
            'mean_repeatability_ok':rms<.25 and spread<.5 and .9<float(norms.mean())<1.1 and wheel<1,
            'physical_contact_status_requires_operator_confirmation':True,
            'accelerometer_bias_corrected':False,
            'physical_balance_verified':False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    result = summarize(args.log)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))
