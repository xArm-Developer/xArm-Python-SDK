import cv2
import os
import numpy as np
#import matplotlib.pyplot as plt

import time

def contrast(gray_image):
    
    if gray_image.dtype != np.uint16:
        gray_image = gray_image.astype(np.uint16)

    print(__name__)
    # maybe use this later: np.full((3, 3), False, dtype=bool)
    rows = gray_image.shape[0]
    cols = gray_image.shape[1]
    # matrix = np.full((rows, cols), np.nan)
    outline = np.ones((gray_image.shape[0],gray_image.shape[1], 1), dtype=np.uint16) * 255

    outline[0,:] = 255
    outline[rows-1,:] = 255
    outline[:,0] = 255
    outline[:,cols-1] = 255
    
    for i in range(rows):
        for j in range(cols):
            diff = []
            if (i > 0): # top
                diff.append(int(gray_image[i-1, j]) - int(gray_image[i, j]))
            if (i < rows - 1): # bottom
                diff.append(int(gray_image[i+1, j]) - int(gray_image[i, j]))
            if (j > 0): # left
                diff.append(int(gray_image[i, j-1]) - int(gray_image[i, j]))
            if (j < cols - 1): # right
                diff.append(int(gray_image[i, j+1]) - int(gray_image[i, j]))

            difference = max(diff)
            #print(diff)
            # Threshold, higher = more difference between neighboring pixels
            if difference >= 30: 
                outline[i, j] = 0

    outline = outline.astype(np.uint8)

    return outline


if __name__ == "__main__":

    print(__name__)

    start_time = time.perf_counter()

    # where new image is saved at
    save_dir = 'test/test_contrast'
    save_path = os.path.join(save_dir, '00.0.jpeg')

    # access uncropped file
    file_dir = "test/UncroppedImagesSet2"
    filename = os.path.join(file_dir, "00.0.jpg")
    image = cv2.imread(filename)

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #gray_image = gray_image.astype(np.uint16)

    # Show image
    print(image.shape)
    outline = contrast(gray_image)
    #gray_image = gray_image.astype(np.uint8)
    #outline = outline.astype(np.uint8)

    #cv2.namedWindow('gray image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('outline', cv2.WINDOW_NORMAL)
    #cv2.imshow('gray image', gray_image)
    cv2.imshow('outline', outline)

    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    print(f"Program ran in {elapsed_time:.5f} seconds")

    # Wait for a key press and close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()