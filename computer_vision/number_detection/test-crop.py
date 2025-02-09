import unittest
import cv2
import os
import glob
import argparse
import time
from parallelPrefix import crop_image



class TestCropping(unittest.TestCase):

    def setUp(self):
        # Read the image
        self.rd_dir = "./test/UncroppedImagesSet2/"
        #NOTE: change once ready back to "/cropped/"
        self.wr_dir = "./test/test-cropped/"
        os.makedirs(self.rd_dir, exist_ok=True)
        os.makedirs(self.wr_dir, exist_ok=True)

    # test a single file and display on window
    def test_single(self):
        imgFile = self.imgFile
        img = cv2.imread(self.rd_dir+imgFile)

        if img is None:
            print("Failed to load image")
            exit()
        
        start_time = time.time()
        cropped_img,_ = crop_image(img)
        end_time = time.time()

        exec_time = (end_time - start_time) * 1000

        print(f"Execution time: {exec_time} ms")
        
        cv2.imshow(imgFile, cropped_img)
        cv2.waitKey(0)
        return

    # test all files by looping through and writing out
    def test_all(self):
        image_files = glob.glob(os.path.join(self.rd_dir,"*.jpg"))

        for filename in image_files:

            img = cv2.imread(filename)

            # Check if the image was successfully loaded
            if img is None:
                print("Failed to load the image")
                exit()
            
            cropped_img = crop_image(img)[0]

            # Save the cropped image to the output directory
            base_filename = os.path.basename(filename)
            output_path = os.path.join(self.wr_dir, base_filename)
            cv2.imwrite(output_path, cropped_img)
            print(f"Cropped image saved to {output_path}")
        return

#NOTE: change testAll if you want to test all files or not
def main(testAll=True):
    # user should give us an image file arg
    suite = unittest.TestSuite()
    if testAll:
        test_case = TestCropping('test_all')
    else:    
        parser = argparse.ArgumentParser(description="Run single test")
        parser.add_argument("imgFile", type=str, nargs="?", default="00.0.jpg", help="The image file we wish to crop")
        arg = parser.parse_args()


        test_case = TestCropping('test_single')
        
        #default value should be to 0.0.jpg
        test_case.imgFile = arg.imgFile

    suite.addTest(test_case)

    runner = unittest.TextTestRunner()
    runner.run(suite)

if __name__ == "__main__":
    main()
    