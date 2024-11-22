import cv2
import os
import glob
import numpy as np
import timeit
import threading
import multiprocessing

'''
We are going to implement a prefix-minimum solution to the cropping problem


TODO:
    0) Maybe try making the cropping out of the white first before cropping the dark
    1) create a single pass prefix-minimum algorithm starting from left to right on the matrix

    RUNTIMES:
        OG: 0.028703362499800277 seconds
        THREAD: 0.021070941699872493 seconds
        Process: 0.25637598340035767

    PROBLEM FILES:
        - 0.0 (up dark value tolerance to +3, passes)
        - 1.0 (tolerance to 1, doesn't crop at all)
        - 1.5 (tolerance to 2, bottom right bias)
        - 2.5 (tolerance to 3, crops not enough, but O.W number isn't visible,
            bottom right)
        - 3.0 (tolerance to 3), crops not enough, but O.W number isn't completely visible
        - 4.0 (tolerance to 1, bottom right)
        - 4.5 (tolerance to 1, top left bias)
        - 7.0  (tolerance to 2, no bias)
        
        
        ARYAAN 
        - 7.5 (0, no bias)
        - 8.5 (+1, no bias)
        - 11.5 (N/A, larger problem, 
            bottom right bias cuts off number from left)
        - 12.0 (0, bottom right)
        - 13.0 (0, no bias)
        - 13.5 (1, no bias)
        - 16.0 (2, bottom right)
        - 16.5 (0, top left bias)
        - 18.5 (0, bottom right )
        - 19.5 (1, bottom left bias)

'''
toleranceOffst = 1
# Global variables


def prefix_min(arr, results, axis=0):
    # Initialize the prefix minimum array with the same shape as the input array
    prefix_min_arr = np.empty_like(arr)

    row_bound = arr.shape[0]
    col_bound = arr.shape[1]
    
    # Compute the prefix minimum along the specified axis
    # top -> bottom
    if axis == 0:
        prefix_min_arr[0, :] = arr[0, :]
        for i in range(1, row_bound):
            prefix_min_arr[i, :] = np.minimum(prefix_min_arr[i-1, :], arr[i, :])
    
    
    # left -> right
    elif axis == 1:
        prefix_min_arr[:, 0] = arr[:, 0]
        for j in range(1, col_bound):
            prefix_min_arr[:, j] = np.minimum(prefix_min_arr[:, j-1], arr[:, j])

    # bottom -> top
    elif axis == 2:
        prefix_min_arr[row_bound-1, :] = arr[row_bound-1, :]
        for i in range(row_bound-2, -1, -1):
            prefix_min_arr[i, :] = np.minimum(prefix_min_arr[i+1, :], arr[i, :])

    # right -> left
    elif axis == 3:
        prefix_min_arr[:, col_bound-1] = arr[:, col_bound-1]
        for j in range(col_bound-2, -1, -1):
            prefix_min_arr[:, j] = np.minimum(prefix_min_arr[:, j+1], arr[:, j])
    

    results[axis] = prefix_min_arr

def crop_image(img):
    # Call the function and display the result

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray',gray)

    results = {}
    #NOTE: changed to 4 to calculate all scan direction 
    NUM_THREADS = 4
    threads = [None] * NUM_THREADS

    # create and start threads
    for i in range(0,NUM_THREADS):
        threads[i] = threading.Thread(target=prefix_min, args=(gray,results,i))
        threads[i].start()

    # stop and join them back
    for i in range(0,NUM_THREADS):
        threads[i].join()

    #NOTE: try changing the mask to include right->left instead of 
    # left->right, will it change the crop bias????

    #NOTE: Doesn't matter.....

    prefix_min_tb = results[0]
    prefix_min_lr = results[1]
    prefix_min_bt = results[2]
    prefix_min_rl = results[3]

    shared_min_mask = ((prefix_min_lr == prefix_min_rl) == (prefix_min_bt == prefix_min_tb))

    # Find the darkest region in the shared minimum region
    darkest_value = np.min(gray[shared_min_mask])+toleranceOffst
    darkest_coords = np.argwhere((gray <= darkest_value) & shared_min_mask)

    # Find the bounding box of the darkest region
    min_row, min_col = np.min(darkest_coords, axis=0)
    max_row, max_col = np.max(darkest_coords, axis=0)

    # Crop the image based on the bounding box
    cropped_img = img[min_row:max_row+1, min_col:max_col+1]

    return cropped_img


def main():
     # Read the image
    rd_dir = "./UncroppedImagesSet2"
    wr_dir = "./cropped"

    image_files = glob.glob(os.path.join(rd_dir,"*.jpg"))

    for filename in image_files:

        img = cv2.imread(filename)

        # Check if the image was successfully loaded
        if img is None:
            print("Failed to load the image")
            exit()
        
        cropped_img = crop_image(img)
        # Save the cropped image to the output directory
        base_filename = os.path.basename(filename)
        output_path = os.path.join(wr_dir, base_filename)
        cv2.imwrite(output_path, cropped_img)
        print(f"Cropped image saved to {output_path}")
        

if __name__ == "__main__":
    main()