import unittest
import math

import cv2

from artifacts import count_red


class ArtifactDetectorTest(unittest.TestCase):

    def test_count_red(self):
        img = cv2.imread('test_data/artf-backpack.jpg')
        self.assertEqual(count_red(img), (1576, 47, 87, 272, 319))

        img = cv2.imread('test_data/artf-extinguisher.jpg')
        self.assertEqual(count_red(img), (527, 19, 56, 0, 19))

        img = cv2.imread('test_data/artf-extinguisher-hd.jpg')
        self.assertEqual(count_red(img), (8221, 84, 221, 23, 107))

        img = cv2.imread('test_data/artf-valve-hd.jpg')
        self.assertEqual(count_red(img), (187, 21, 30, 322, 343))

        img = cv2.imread('test_data/artf-valve2-hd.jpg')
        self.assertEqual(count_red(img), (116, 1, 34, 344, 345))

# vim: expandtab sw=4 ts=4

