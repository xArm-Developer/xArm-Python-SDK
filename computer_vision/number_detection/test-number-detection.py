import unittest
import os
import number_detection

filePath = './cropped'

class TestNumberDetection(unittest.TestCase):
    def test_function_multiple_times(self):
        # go through each file in the cropped directory
        # parse through the name and extract the expected value from the file name
        # anything in ()'s is diregarded
        # number will be xx-x = xx.x 
        for imgFile in os.listdir(filePath):
            #only get the first three digits with hyphen
            imgExpected = imgFile.split('(',1)[0]
            print("File Name: " + imgExpected)

            print("Expected Value: " + imgExpected)
            with self.subTest(input_data=imgFile, expected=imgExpected):
                result = read_img(input_data)
                self.assertEqual(result,expected,
                                 input_data+" FAILED! RESULT: " + result + " EXPECTED: " + expected)
                
            #END OF METHOD


if __name__ == '__main__':
    unittest.main()