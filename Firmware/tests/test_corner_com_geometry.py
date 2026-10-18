"""Physical geometry invariants for static suspension; no hardware access."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('com_geometry',
    Path(__file__).resolve().parents[1]/'diagnostics/corner_com_geometry.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class SuspensionGeometryTests(unittest.TestCase):
    def test_known_com_is_above_balance_support_and_below_hanging_support(self):
        support = np.array([35.,35.,-35.])
        center = np.array([8.,-4.,5.])
        hanging_up = (support-center)/np.linalg.norm(support-center)
        result = geometry.support_corner_reference(hanging_up)
        reference = np.array(result['reference_gravity_imu'])
        self.assertGreater(float(np.dot(center-support,reference)),0)
        np.testing.assert_allclose(np.cross(center-support,reference),0,atol=1e-12)
        self.assertFalse(result['com_position_identified'])
        self.assertFalse(result['physical_balance_verified'])

    def test_single_direction_cannot_identify_com_height(self):
        direction = np.array([-.7,.4,.6])
        a = geometry.support_corner_reference(-direction*10)
        b = geometry.support_corner_reference(-direction*40)
        np.testing.assert_allclose(a['reference_gravity_imu'],b['reference_gravity_imu'])
        self.assertFalse(a['com_position_identified'])

    def test_same_corner_solution_is_rotation_invariant(self):
        rotation = np.array([[0,-1,0],[0,0,1],[-1,0,0]])
        up = np.array([.6,-.5,-.7])
        a = geometry.support_corner_reference(up)
        b = geometry.support_corner_reference(rotation@up)
        np.testing.assert_allclose(b['reference_gravity_imu'],rotation@a['reference_gravity_imu'])

    def test_two_gravity_lines_recover_known_center(self):
        center = np.array([8.,-4.,5.])
        points = np.array([[35.,35.,35.],[-35.,35.,-35.]])
        up = [(p-center)/np.linalg.norm(p-center) for p in points]
        result = geometry.fit_suspension(points,up)
        np.testing.assert_allclose(result['center_mm'],center,atol=1e-10)
        self.assertTrue(result['geometry_quality_ok'])

    def test_parallel_lines_and_wrong_up_down_convention_are_rejected(self):
        points = [[35,35,35],[-35,35,-35]]
        with self.assertRaises(ValueError):
            geometry.fit_suspension(points,[[0,0,1],[0,0,-1]])
        center = np.array([8.,-4.,5.])
        down = [(center-np.array(p))/np.linalg.norm(center-p) for p in points]
        self.assertFalse(geometry.fit_suspension(points,down)['geometry_quality_ok'])

    def test_invalid_vectors_cannot_generate_reference(self):
        for value in ([0,0,0],[float('nan'),0,1],[1,2]):
            with self.assertRaises(ValueError):
                geometry.support_corner_reference(value)

    def test_nonzero_attachment_offset_needs_distance(self):
        with self.assertRaises(ValueError):
            geometry.support_corner_reference([.6,-.4,-.7],[-3,0,0])

    def test_known_attachment_offset_is_compensated(self):
        center = np.array([-40.,22.,34.])
        attachment = np.array([-3.,0,0])
        distance = np.linalg.norm(center-attachment)
        up = (attachment-center)/distance
        result = geometry.support_corner_reference(up,attachment,distance)
        np.testing.assert_allclose(result['reference_gravity_imu'],center/np.linalg.norm(center))

    def test_two_edge_points_recover_support_to_com_vector(self):
        center = np.array([-40.,22.,34.])
        points = np.array([[-3.,0,0],[-67.,0,0]])
        up = [(p-center)/np.linalg.norm(p-center) for p in points]
        result = geometry.support_frame_reference(points,up)
        self.assertTrue(result['reference_direction_available'])
        np.testing.assert_allclose(result['support_to_com_imu_mm'],center,atol=1e-10)
        np.testing.assert_allclose(result['reference_gravity_imu'],center/np.linalg.norm(center))

    def test_inconsistent_third_attachment_does_not_generate_reference(self):
        points = [[-3,0,0],[-70,0,0],[-35,0,0]]
        directions = [[.6624,-.4126,-.6253],[-.5553,-.5031,-.6622],[-.7566,-.6538,.0085]]
        result = geometry.support_frame_reference(points,directions)
        self.assertFalse(result['reference_direction_available'])
        self.assertNotIn('reference_gravity_imu',result)


if __name__ == '__main__':
    unittest.main()
