import math
import unittest

from . import local_planner

class LocalPlannerTest(unittest.TestCase):

    def test_no_scan(self):
        lp = local_planner.LocalPlanner()
        safety, safe_direction = lp.recommend(0)
        self.assertEqual(safety, 1.0)
        self.assertEqual(safe_direction, 0.0)

    def test_empty_scan(self):
        lp = local_planner.LocalPlanner()
        lp.update([])
        r = lp.recommend(0)
        rm = lp.recommend(0, map=True)
        self.assertEqual(r, rm)
        self.assertEqual(r, (1.0, 0.0))

    def test_3_scan_out_of_range(self):
        lp = local_planner.LocalPlanner()
        lp.update([0, 1501, 0])
        r = lp.recommend(0)
        rm = lp.recommend(0, map=True)
        self.assertEqual(r, rm)
        self.assertEqual(r, (1.0, 0.0))

    def test_3_scan_in_range(self):
        lp = local_planner.LocalPlanner()
        lp.update([0, 1400, 1400])
        desired = math.radians(10)
        r = lp.recommend(desired)
        rm = lp.recommend(desired, map=True)
        self.assertEqual(r, rm)
        self.assertEqual(r, (1.0, 0.0))
