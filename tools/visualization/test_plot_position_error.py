#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_position_error as plot


class GroundTruthLeverArmTest(unittest.TestCase):
    def test_converts_ground_truth_reference_to_body_reference_with_full_attitude(self):
        truth = {
            "utc_tow": 1.0,
            "lon": 121.0,
            "lat": 31.0,
            "height": 10.0,
            "heading": 0.0,
            "pitch": -10.0,
            "roll": -160.0,
        }
        t_b_gt = {
            "rotation": plot.identity_matrix(),
            "translation": [-0.0016315261, -0.3204949122, 0.0665710843],
        }

        adjusted = plot.ground_truth_to_body_reference(truth, t_b_gt)
        east, north, up = plot.enu_offset(adjusted, truth)
        r_enu_body = plot.ground_truth_body_rotation_enu(truth, t_b_gt)
        expected = plot.mat_vec_mul(r_enu_body, t_b_gt["translation"])

        self.assertAlmostEqual(east, -expected[0], places=4)
        self.assertAlmostEqual(north, -expected[1], places=4)
        self.assertAlmostEqual(up, -expected[2], places=4)

    def test_attitude_error_uses_so3_angle_after_t_b_gt_rotation(self):
        truth = {
            "heading": -25.3508612322,
            "pitch": -0.574936278137,
            "roll": -179.833867895,
        }
        t_b_gt = {
            "rotation": [
                [-0.9998711924420876, -0.012827221268790408, 0.00964680874480093],
                [-0.012815686501750663, 0.9999170861579333, 0.0012565782574573774],
                [-0.009662127298174172, 0.0011327859240649932, -0.9999526789264197],
            ],
            "translation": [-0.0016315261232187732, -0.32049491219831694, 0.06657108425640941],
        }
        solution_rotation = plot.euler_rpy_matrix(
            -0.447399128833,
            0.99625205967,
            24.4020677437,
        )

        error = plot.rotation_error_deg(solution_rotation, plot.ground_truth_body_rotation_enu(truth, t_b_gt))

        self.assertAlmostEqual(error, 0.4070763020, places=6)


if __name__ == "__main__":
    unittest.main()
