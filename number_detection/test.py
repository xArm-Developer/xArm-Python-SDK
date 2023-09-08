#!/usr/bin/env python3

import cv2
import unittest
from OCR import OCR

class UnitTests(unittest.TestCase):
    image = []
    def setUp(self):
        self.image = cv2.imread("image_processing_cv/ex3.jpeg")

    def test_ocr(self):
        obj = OCR(self.image)
        self.assertEqual(obj.get_number(), ("The number is: ", 17.5))


if __name__ == '__main__':
    unittest.main()