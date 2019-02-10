import unittest
import math

import cv2

from artifacts import count_red


class ArtifactDetectorTest(unittest.TestCase):

    def test_count_red(self):
        img = cv2.imread('test_data/artf-backpack.jpg')
        self.assertEqual(count_red(img), (1576, 47, 87))

        img = cv2.imread('test_data/artf-extinguisher.jpg')
        self.assertEqual(count_red(img), (527, 19, 56))

# vim: expandtab sw=4 ts=4

