import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'diagnostics'))
from corner_probe_review import measure, paired_review


class PulseReviewTests(unittest.TestCase):
    def measurements(self, inertia, reverse_x=False):
        response = np.linalg.inv(inertia)
        result = []
        for motor, axis in ((1, 2), (2, 3), (3, 1)):
            for sign in (1, -1):
                delta = sign*25
                column = response[:, axis-1]*(-1 if reverse_x and axis == 1 else 1)
                result.append({'motor':motor,'body_axis':axis,'voltage':3*sign,'duration_ms':80,
                               'gyro_delta_rad_s':(column*delta).tolist(),'wheel_delta_rad_s':delta})
        return result

    def test_axis_order_and_direction_recovered_from_physical_response(self):
        inertia = np.array([[30.,8,-6],[8,40,9],[-6,9,35]])
        report = paired_review(self.measurements(inertia, reverse_x=True))
        self.assertTrue(report['inertia_consistency_passed'])
        self.assertEqual([p['candidate_body_direction'] for p in report['paired_motors']],[-1,-1,1])
        np.testing.assert_allclose(report['approximate_inertia_over_wheel_inertia'],inertia,atol=1e-9)

    def test_nonphysical_principal_moments_are_rejected(self):
        report = paired_review(self.measurements(np.diag([2.,3,12])))
        self.assertFalse(report['inertia_triangle_ok'])
        self.assertFalse(report['inertia_consistency_passed'])

    def test_incomplete_trials_and_pairs_are_not_models(self):
        with self.assertRaises(ValueError):
            measure({'result':'stopped'})
        report = paired_review(self.measurements(np.eye(3)*20)[:2])
        self.assertFalse(report['complete_three_axis_pairs'])
        self.assertEqual(report['missing_paired_motors'],[2,3])


if __name__ == '__main__':
    unittest.main()
