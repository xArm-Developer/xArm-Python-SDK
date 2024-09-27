import unittest
import os
from number_detection import read_img
import contextlib
import time

filePath = './test/cropped/'
txtPath = './test/testResults/'

class TestNumberDetection(unittest.TestCase):
    def test_function_multiple_times(self):
        # go through each file in the cropped directory
        # parse through the name and extract the expected value from the file name
        # anything in ()'s is diregarded
        for imgFile in os.listdir(filePath):
            #only get the first three digits with hyphen
            imgExpected = imgFile.split('(')[0].split('.jpg')[0].split('.png')[0]

            print("File Name: " + imgFile)
            print("Expected Value: " + imgExpected)

            #not completely sure about this syntax
            with self.subTest(imgFile=imgFile, imgExpected=imgExpected):
                result = read_img(filePath+imgFile)
                self.assertEqual(result,imgExpected,
                                imgFile +" FAILED! RESULT: " + result + " EXPECTED: " + imgExpected)
            
            print('\n')
            #END OF METHOD


# NOTE: to run this block of code, run 'python test-number-detection.py'

if __name__ == '__main__':
    # unittest.main()
    
    # Ensure the directory exists
    os.makedirs(txtPath, exist_ok=True)

    # Get the current time and format it
    currentTime = time.strftime('%m-%d_%H-%M-%S')
    print(f"Current time: {currentTime}")

    # Open the file with the current time in the filename
    file_path = f'{txtPath}result_{currentTime}.txt'
    with open(file_path, 'w') as f:
        f.write(f"Test started at: {currentTime}\n")
        # Create a test suite and add the test case
        suite = unittest.TestLoader().loadTestsFromTestCase(TestNumberDetection)
        
        # Create a test runner that writes to the file
        runner = unittest.TextTestRunner(stream=f, verbosity=2)
        
        # Run the tests
        runner.run(suite)