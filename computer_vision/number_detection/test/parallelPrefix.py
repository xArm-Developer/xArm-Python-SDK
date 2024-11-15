import cv2
import os
import numpy as np

'''
We are going to implement a prefix-minimum solution to the cropping problem


TODO:
    0) Maybe try making the cropping out of the white first before cropping the dark
    1) create a single pass prefix-minimum algorithm starting from left to right on the matrix




'''

# Read the image
file_dir = "./UncroppedImagesSet2"
filename = os.path.join(file_dir, "18.0.jpg")
img = cv2.imread(filename)
height, width = img.shape[:2]

# Check if the image was successfully loaded
if img is not None:
    # Get the dimensions of the image
    height, width = img.shape[:2]
    
    print(f"Image width: {width} pixels")
    print(f"Image height: {height} pixels")
else:
    print("Failed to load the image")
    exit()

# def compute_prefix_sums(image):
#     # Compute prefix sums in all four directions
#     prefix_sum_lr = np.cumsum(image, axis=1)
#     prefix_sum_rl = np.cumsum(image[:, ::-1], axis=1)[:, ::-1]
#     prefix_sum_tb = np.cumsum(image, axis=0)
#     prefix_sum_bt = np.cumsum(image[::-1, :], axis=0)[::-1, :]
#     return prefix_sum_lr, prefix_sum_rl, prefix_sum_tb, prefix_sum_bt

def prefix_min(arr, axis=0):
    # Initialize the prefix minimum array with the same shape as the input array
    prefix_min_arr = np.empty_like(arr)
    
    # Compute the prefix minimum along the specified axis
    #top -> bottom
    if axis == 0:
        prefix_min_arr[0, :] = arr[0, :]
        for i in range(1, arr.shape[0]):
            prefix_min_arr[i, :] = np.minimum(prefix_min_arr[i-1, :], arr[i, :])
    # left -> right
    elif axis == 1:
        prefix_min_arr[:, 0] = arr[:, 0]
        for j in range(1, arr.shape[1]):
            prefix_min_arr[:, j] = np.minimum(prefix_min_arr[:, j-1], arr[:, j])

    
    return prefix_min_arr


# Call the function and display the result
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
prefix_min_tb = prefix_min(gray, axis=0)
prefix_min_lr = prefix_min(gray, axis=1)


shared_min_mask = (prefix_min_lr == prefix_min_tb)

# Get the coordinates of the shared minimum region
shared_min_coords = np.argwhere(shared_min_mask)

# Find the darkest region in the shared minimum region
darkest_value = np.min(gray[shared_min_mask])+1
print(darkest_value)
darkest_coords = np.argwhere((gray <= darkest_value) & shared_min_mask)

print("Coordinates of the darkest region in the shared minimum region:")
print(darkest_coords)

# Find the bounding box of the darkest region
min_row, min_col = np.min(darkest_coords, axis=0)

max_row, max_col = np.max(darkest_coords, axis=0)


# Crop the image based on the bounding box
cropped_img = img[min_row:max_row+1, min_col:max_col+1]

# Create an image to visualize the shared minimum region
shared_min_image = np.zeros_like(gray)
shared_min_image[shared_min_mask] = 255

cv2.imshow('Original Image', img)
cv2.imshow('left to right', prefix_min_lr)
cv2.imshow('top to bottom', prefix_min_tb)
# cv2.imshow('bottom to top', prefix_min_bt)
# cv2.imshow('right to left', prefix_min_rl)
cv2.imshow('Cropped Image', cropped_img)
cv2.waitKey(0)
cv2.destroyAllWindows()