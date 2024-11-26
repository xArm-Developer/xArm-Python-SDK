import unittest
import os
from number_detection import read_img
import time
import argparse

filePath = './test/cropped/'
txtPath = './test/testResults/'



'''

TEST NOTES:

    - the gray scaling and binary contrast leads to some of the numbers being obscured
        likely due to glare

    - the morphology applied is definitely making small improvements,
        maybe run this in a loop. How many times?

    - the dot is consistently being judged as a '0' does the OCR model 
        not recognize non-numbers? If a string is the size of four and if at index size-2
        is 0, then just replace as a '.'?


    - find images with glare or blurriness

    - for thresholding 

'''

class TestNumberDetection(unittest.TestCase):
    def test_function_multiple_times(self):
        # go through each file in the cropped directory
        # parse through the name and extract the expected value from the file name
        # anything in ()'s is diregarded
        fails = 0
        passes = 0
        results = []

        writeFile = self.writeFile

        for imgFile in os.listdir(filePath):
            #only get the first three digits with hyphen
            imgExpected = imgFile.split('(')[0].split('.jpg')[0].split('.png')[0]

            # print("File Name: " + imgFile)
            # print("Expected Value: " + imgExpected)

            #not completely sure about this syntax
            with self.subTest(imgFile=imgFile, imgExpected=imgExpected):
                result = read_img(filePath+imgFile)

                try:
                    self.assertEqual(result,imgExpected)
                    passes += 1
                    results.append(f"{imgFile:<15} PASSED! RESULT: {result:<8}")
                except AssertionError as e:
                    fails += 1
                    results.append(f"{imgFile:<15} FAILED! RESULT: {result:<8} EXPECTED: {imgExpected:<5}")
            
        # Sort results: FAILED first, then PASSED
        results.sort(key=lambda x: "PASSED" in x)
        
        pass_rate = passes / (passes + fails) if (passes + fails) > 0 else 0
        summary = f"PASSES: {passes} \t FAILS: {fails} \t RATE: {pass_rate:.2f}\n"
        
        with open(writeFile, "w") as f:
            f.write(f"Test started at: {currentTime}\n")
            f.write("\nTest Results:\n")
            for result in results:
                f.write(result + "\n")
            f.write("\nSummary:\n")
            f.write(summary)
            #END OF METHOD
        
       

    def test_function_once(self):
        imgFile = self.imgFile
        imgExpected = imgFile.split('(')[0].split('.jpg')[0].split('.png')[0]

        print("File Name: " + imgFile)
        print("Expected Value: " + imgExpected)

        #not completely sure about this syntax
        with self.subTest(imgFile=imgFile, imgExpected=imgExpected):
            result = read_img(filePath+imgFile)
            self.assertEqual(result,imgExpected,
                            imgFile +" FAILED! RESULT: " + result + " EXPECTED: " + imgExpected)
        
        print('\n')


# NOTE: to run this block of code, run 'python test-number-detection.py'

if __name__ == '__main__':
    # unittest.main()
    
    # Ensure the directory exists
    os.makedirs(txtPath, exist_ok=True)

    # Get the current time and format it
    currentTime = time.strftime('%m-%d_%H-%M-%S')
    print(f"Current time: {currentTime}")

    parser = argparse.ArgumentParser(description="Run single test")
    parser.add_argument("imgFile", type=str, nargs="?", default="15.5.jpg", help="The image file we wish to crop")
    arg = parser.parse_args()

    # Open the file with the current time in the filename
    # file_path = f'{txtPath}{arg.imgFile}_result_{currentTime}.txt'
    file_path = f'{txtPath}FULL_result_{currentTime}.txt'
    # Create a test suite and add the test case
    suite = unittest.TestSuite()
    # test_case = TestNumberDetection('test_function_once')
    # test_case.imgFile = arg.imgFile
    test_case = TestNumberDetection('test_function_multiple_times')
    test_case.writeFile = file_path
    suite.addTest(test_case)
    
    # Create a test runner that writes to the file
    runner = unittest.TextTestRunner(verbosity=0)
    
    # Run the tests
    result = runner.run(suite)

        #  # Write summary to the file
        # f.write(f"\n\nSummary:\n")
        # f.write(f"Ran {result.testsRun} tests\n")
        # f.write(f"Failures: {len(result.failures)}\n")
        # f.write(f"Errors: {len(result.errors)}\n")
        # f.write(f"Skipped: {len(result.skipped)}\n")
        # f.write(f"Passes: {result.testsRun - len(result.failures) - len(result.errors)}\n")