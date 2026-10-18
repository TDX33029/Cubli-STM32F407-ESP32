"""Offline corner geometry and COM from static suspension; never opens hardware.

Motor-face coordinates are projections onto the outward normals of M1/M2/M3.
Their ordering need not be right-handed; this module uses Euclidean line
projections, then maps positions into the actual IMU frame before reporting
firmware Pitch/Roll. BDIR/encoder signs are not face-normal measurements.

At rest the firmware's normalized accelerometer vector points UP. For a freely
suspended body its COM lies BELOW the suspension point: C=P-lambda*g_up.
For corner balance the COM must lie ABOVE the supporting corner instead.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError('Expected a finite three-vector')
    return result


def unit(value):
    result = vector(value)
    length = np.linalg.norm(result)
    if length < 1e-9:
        raise ValueError('Direction has zero length')
    return result / length


def face_basis(normals):
    basis = np.column_stack([unit(value) for value in normals])
    if basis.shape != (3, 3) or not np.allclose(basis.T @ basis, np.eye(3), atol=1e-3):
        raise ValueError('Three measured motor-face normals must be orthogonal')
    return basis


def fit_suspension(points_mm, gravity_up):
    """Least-squares intersection of gravity lines; reports geometry quality.

    All positions and directions must use the SAME body frame. At least two
    nonparallel gravity lines are necessary; three or more check consistency.
    """
    if len(points_mm) != len(gravity_up) or len(points_mm) < 2:
        raise ValueError('At least two distinct suspension observations required')
    points = [vector(p) for p in points_mm]
    directions = [unit(g) for g in gravity_up]
    projections = [np.eye(3) - np.outer(g, g) for g in directions]
    lhs = sum(projections)
    condition = float(np.linalg.cond(lhs))
    if not math.isfinite(condition) or condition > 1000:
        raise ValueError('Gravity lines nearly parallel; COM is not identifiable')
    center = np.linalg.solve(lhs, sum(q @ p for q, p in zip(projections, points)))
    residuals = [float(np.linalg.norm(q @ (center - p))) for p, q in zip(points, projections)]
    downward = [float(-np.dot(center - p, g)) for p, g in zip(points, directions)]
    rms = math.sqrt(sum(r*r for r in residuals) / len(residuals))
    return {'center_mm': center.tolist(), 'condition_number': condition,
            'line_residuals_mm': residuals, 'rms_residual_mm': rms,
            'com_below_suspension_mm': downward,
            'geometry_quality_ok': rms <= 2 and all(d > 0 for d in downward)}


def firmware_angles(gravity_up):
    x, y, z = unit(gravity_up)
    return {'pitch_deg': math.degrees(math.atan2(y, math.hypot(x, z))),
            'roll_deg': math.degrees(math.atan2(-x, z))}


def support_corner_reference(gravity_up_imu, attachment_offset_imu_mm=None, com_distance_mm=None):
    """Candidate direction from suspension at the actual balance support point.

    Assumes free static suspension, the same force/contact point, and negligible
    accelerometer bias. This identifies a direction, not COM distance or inertia.
    Measurement repeatability does not bound attachment-point or sensor bias.
    """
    measured = unit(gravity_up_imu)
    offset = np.zeros(3) if attachment_offset_imu_mm is None else vector(attachment_offset_imu_mm)
    shifted = np.linalg.norm(offset) > 1e-9
    if shifted:
        if com_distance_mm is None or not math.isfinite(float(com_distance_mm)) or float(com_distance_mm) <= 0:
            raise ValueError('Nonzero attachment offset requires COM distance or multiple suspension points')
        reference = unit(offset-float(com_distance_mm)*measured)
    else:
        reference = -measured
    return {'method': ('offset_compensated_suspension' if shifted
                       else 'same_support_point_suspension_reversal'),
            'measured_gravity_up_imu': measured.tolist(),
            'attachment_offset_imu_mm': offset.tolist(),
            'reference_gravity_imu': reference.tolist(),
            'reference_angles': firmware_angles(reference),
            'com_position_identified': False,
            'physical_balance_verified': False,
            'assumptions': [('known attachment offset and COM distance' if shifted
                             else 'same physical suspension and balance support point'),
                            'free static suspension without other supporting torque',
                            'accelerometer bias negligible or independently corrected']}


def support_frame_reference(points_imu_mm, gravity_up_imu, support_imu_mm=(0,0,0)):
    """Fit COM in IMU coordinates; the support point may be the origin.

    This bypasses a separate face-normal basis when all measured attachment
    positions are already expressed in the same IMU frame.
    """
    fit = fit_suspension(points_imu_mm, gravity_up_imu)
    result = {'method': 'multiple_suspension_points_in_imu_frame',
              'suspension_fit': fit, 'reference_direction_available': False,
              'physical_balance_verified': False}
    if fit['geometry_quality_ok']:
        relative_center = vector(fit['center_mm'])-vector(support_imu_mm)
        reference = unit(relative_center)
        result.update(reference_direction_available=True,
                      support_to_com_imu_mm=relative_center.tolist(),
                      reference_gravity_imu=reference.tolist(),
                      reference_angles=firmware_angles(reference))
    return result


def calculate(data):
    side = float(data['cube_side_mm'])
    if not math.isfinite(side) or side <= 0:
        raise ValueError('Invalid cube size')
    support = vector(data['support_corner_motor_face_mm'])
    ideal = unit(-support)
    result = {'support_corner_motor_face_mm': support.tolist(),
              'geometric_center_reference_motor_face': ideal.tolist(),
              'geometric_case_only': True,
              'board_normal_to_vertical_deg_if_com_at_center':
                  math.degrees(math.acos(float(np.clip(-ideal[1], -1, 1)))),
              'actual_com_reference_available': False,
              'physical_balance_verified': False}
    normals = data.get('signed_motor_face_normals_imu')
    if normals is None:
        result['missing'] = ['signed motor-face normals in the IMU frame',
                             'two or more static free-suspension observations']
        return result
    basis = face_basis(normals)
    result['geometric_case_imu_angles'] = firmware_angles(basis @ ideal)
    observations = data.get('suspension_observations', [])
    if len(observations) < 2:
        result['missing'] = ['two or more static free-suspension observations']
        return result
    # Fit in the motor-face projection coordinates, whose metric is Euclidean.
    fit = fit_suspension([o['point_motor_face_mm'] for o in observations],
                         [basis.T @ unit(o['gravity_up_imu']) for o in observations])
    result['suspension_fit'] = fit
    if not fit['geometry_quality_ok']:
        result['missing'] = ['consistent freely suspended static measurements']
        return result
    gravity = unit(basis @ (vector(fit['center_mm']) - support))
    result.update(actual_com_reference_available=True,
                  center_motor_face_mm=fit['center_mm'],
                  reference_gravity_imu=gravity.tolist(),
                  reference_angles=firmware_angles(gravity))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input', type=Path)
    source.add_argument('--same-corner-up', type=float, nargs=3,
                        help='Measured upward acceleration direction in the IMU frame')
    source.add_argument('--support-frame-input', type=Path,
                        help='JSON with points_imu_mm, gravity_up_imu and optional support_imu_mm')
    parser.add_argument('--attachment-offset-imu-mm', type=float, nargs=3)
    parser.add_argument('--com-distance-mm', type=float)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.same_corner_up is not None:
        result = support_corner_reference(args.same_corner_up, args.attachment_offset_imu_mm, args.com_distance_mm)
    elif args.support_frame_input is not None:
        data = json.loads(args.support_frame_input.read_text(encoding='utf-8-sig'))
        result = support_frame_reference(data['points_imu_mm'], data['gravity_up_imu'],
                                         data.get('support_imu_mm', (0,0,0)))
    else:
        result = calculate(json.loads(args.input.read_text(encoding='utf-8-sig')))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
