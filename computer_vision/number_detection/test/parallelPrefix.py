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
        - 11.0: still cuts off!

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

    
    lowerThresh = 0
    upperThresh = 25

    combined_mask = ((prefix_min_tb >= lowerThresh) & (prefix_min_tb <= upperThresh)) & (prefix_min_lr >= lowerThresh) & (prefix_min_lr <= upperThresh) & (prefix_min_rl >= lowerThresh) & (prefix_min_rl <= upperThresh) & (prefix_min_bt >= lowerThresh) & (prefix_min_bt <= upperThresh) 


    max_area = 0
    largest_bbox = None
    
    contours, _ = cv2.findContours(combined_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w*h
        if (area > max_area):
            max_area = area
            largest_bbox = (x,y,w,h)

    if largest_bbox is not None:
        x,y,w,h = largest_bbox
        cropped_img = img[y:y+h, x:x+w]
    else:
        cropped_img = img

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