import unittest
import math

import cv2

from subt.main import Trace, ray


class SubTChallengeTest(unittest.TestCase):

    def test_trace(self):
        st = Trace()
        self.assertEqual(len(st.trace), 1)  # initialized with (0, 0, 0)

        st.update_trace((0, 0, 0))
        self.assertEqual(len(st.trace), 1)  # duplicities removed

        st.update_trace((1.0, 0, 0))
        self.assertEqual(len(st.trace), 2)

        # now move in 3D up
        st.update_trace((1.0, 0, 1.0))
        self.assertEqual(len(st.trace), 3)

        # and down ...
        st.update_trace((1.0, 0, 0.0))
        self.assertEqual(len(st.trace), 4)

        st.prune()
        self.assertEqual(len(st.trace), 2)

        # now longer loop
        for i in range(10):
            st.update_trace((1, i, 0))

        for i in range(10):
            st.update_trace((1, 9 - i, 0))

        st.update_trace((2, 0, 0))
        st.update_trace((3, 0, 0))
        self.assertEqual(len(st.trace), 22)

        st.prune()
        self.assertEqual(len(st.trace), 4)

    def test_ray(self):
        self.assertAlmostEqual(ray(0.0, 1, 0, -5), 5)
        self.assertAlmostEqual(ray(math.radians(45), 1, 0, -5), 5*math.sqrt(2))
        self.assertIsNone(ray(math.radians(90), 1, 0, -5))
        self.assertIsNone(ray(math.radians(135), 1, 0, -5))

        self.assertAlmostEqual(ray(math.radians(90), 0, 1, -5), 5)

# vim: expandtab sw=4 ts=4

