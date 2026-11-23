"""Run with the rebuilt sire Python module: python -m unittest discover -s test/python."""
import math
import unittest

import sire


class BodyVelocityTest(unittest.TestCase):
    def assertVectorClose(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=12)

    def test_translation_and_yaw(self):
        # p=(1,2,3), omegaW=(1,0,0), v_originW=(0,1,0).
        # uW=v_originW-omegaW cross p=(0,4,-2).
        q = math.sqrt(0.5)
        result = sire.vs2bodyVa([1, 2, 3, 0, 0, q, q], [0, 4, -2, 1, 0, 0])
        self.assertVectorClose(result, [1, 0, 0, 0, -1, 0])

    def test_rotating_at_fixed_body_origin(self):
        result = sire.vs2bodyVa([1, 2, 0, 0, 0, 0, 1], [2, -1, 0, 0, 0, 1])
        self.assertVectorClose(result, [0, 0, 0, 0, 0, 1])

    def test_quaternion_normalization_and_sign(self):
        expected = [1, 2, 3, 4, 5, 6]
        for qw in [1, 2, -2]:
            self.assertVectorClose(sire.vs2bodyVa([0, 0, 0, 0, 0, 0, qw], expected), expected)

    def test_invalid_input(self):
        for pose in [[0]*6, [0]*7, [0, 0, 0, 0, 0, 0, float("nan")]]:
            with self.assertRaises(ValueError):
                sire.vs2bodyVa(pose, [0]*6)
        with self.assertRaises(ValueError):
            sire.vs2bodyVa([0, 0, 0, 0, 0, 0, 1], [0]*5)


if __name__ == "__main__":
    unittest.main()
