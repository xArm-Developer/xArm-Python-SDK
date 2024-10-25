import cv2
import os
import numpy as np

import time

save_dir = 'computer_vision/number_detection/test/test_cropped'
save_path = os.path.join(save_dir, '08.0.jpeg')

file_dir = "computer_vision/number_detection/test/uncropped"
filename = os.path.join(file_dir, "08.0.jpeg")
image = cv2.imread(filename)


gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray_image = gray_image.astype(np.uint32)

class DotMatrix2:
    def __init__(self, bordh, bordw, boxh, boxw):
        self.bordh = bordh
        self.bordw = bordw
        self.boxh = boxh
        self.boxw = boxw
        self.h = bordh*2 + boxh
        self.w = bordw*2 + boxw

m = DotMatrix2(3,4,5,10)

def averageImage(image: np.ndarray, matrix: DotMatrix2) -> np.ndarray:
    img1 = image/matrix.bordw
    img2 = image/matrix.boxw
    (h, w) = img1.shape[:2]
    bordw_sum = np.empty((h, w-matrix.w))
    boxw_sum = np.empty((h, w-matrix.w))

    for idx, row in enumerate(bordw_sum):
        image_row = img1[idx]
        for i in range(0,len(row)):
            row[i] = sum(image_row[i:i+matrix.bordw])
    for idx, row in enumerate(boxw_sum):
        image_row = img2[idx]
        for i in range(0,len(row)):
            row[i] = sum(image_row[i:i+matrix.bordw])

    return (bordw_sum, boxw_sum)

def averageAverageImage(bordw_sum, boxw_sum, matrix):
    # sides, topbottom, middle
    h = len(bordw_sum)
    w = len(bordw_sum[0])
    sides = np.empty((h-matrix.h, w))
    topbottom = np.empty((h-matrix.bordh, w))
    middle = np.empty((h-matrix.boxh, w))

    for h in range(len(sides)):
        for w in range(len(sides[0])):
            sides[h, w] = np.sum(bordw_sum[h:h+matrix.h, w])
    for h in range(len(topbottom)):
        for w in range(len(topbottom[0])):
            topbottom[h, w] = np.sum(boxw_sum[h:h+matrix.h, w])
    for h in range(len(middle)):
        for w in range(len(middle[0])):
            middle[h, w] = np.sum(boxw_sum[h:h+matrix.h, w])
    
    return sides, topbottom, middle

a = averageImage(gray_image, m)
print(averageAverageImage(a[0], a[1], m))

# def computeDot(bordw_sum, boxw_sum, matrix: DotMatrix2, coords):
#     (h, w) = coords
#     sum = np.sum(bordw_sum[h:h+matrix.h, w], axis=0)
#     sum += np.sum(bordw_sum[h:h+matrix.h, w+matrix.w-matrix.bordw], axis=0)
#     sum += np.sum(boxw_sum[h:h+matrix.bordh, w+matrix.bordw], axis=0)
#     sum -= np.sum(boxw_sum[h+matrix.bordh:h-matrix.bordh+matrix.h, w+matrix.bordw], axis=0)
#     sum += np.sum(boxw_sum[h+matrix.h-matrix.bordh:h+matrix.h, w+matrix.bordw], axis=0)
#     return sum

# def maxLoc(image: np.ndarray, matrix: DotMatrix2):
#     start_time = time.time()
#     (bordw_sum, boxw_sum) = averageImage(image, matrix)
#     end_time = time.time()
#     maxh = image.shape[0] - matrix.h - 1
#     maxw = image.shape[1] - matrix.w - 1

#     print(end_time - start_time)

#     maxsum = [-128, (-1,-1)]
#     for h in range(maxh):
#         for w in range(maxw):
#             print(h,w)
#             sum = computeDot(bordw_sum, boxw_sum, matrix, (h, w))
#             if sum > maxsum[0]:
#                 maxsum = [sum, (h,w)]
    
#     return maxsum

# print(maxLoc(gray_image, m))

# print(a[-90:-80])
# print(gray_image[0][-90:-84])

# print(a)
# print(gray_image[0])


# v = gray_image[0][-4:]
# # print(v)
# # print(sum(v))

# for i in range(len(a[:-80])):
#     v = a[i]*4 - sum(gray_image[0][i:i+4])
#     print(v)

