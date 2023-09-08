import cv2
import numpy as np

# resize image
# load the example image
image = cv2.imread("ex1.jpeg")
crop_img = image[350:600, 200:450]
cv2.imwrite('cropped.png', crop_img)

# pre-process the image by resizing it, converting it to
# graycale, blurring it, and computing an edge map
gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
# Applying Gaussian blurring with a 5×5 kernel to reduce high-frequency noise

thresh1 = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
thresh1 = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel1)
cv2.imshow('cropped.png', thresh1)
cv2.waitKey(0)

blurred = cv2.GaussianBlur(thresh1, (5, 5), 0)
edged = cv2.Canny(blurred, 50, 200, 255)


# get black box contour: 
# find contours in the edge map, then sort them by their
# size in descending order

cnts, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

print(len(cnts))
cnts = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
displayCnt = cnts[0]
print(displayCnt.shape)
# mask = np.zeros(crop_img.shape, dtype='uint8')
# print(crop_img.shape)
# cv2.drawContours(mask, cnts, -1, (255),1)
# mask = mask[:, :, 1]

# isolated = cv2.bitwise_and(crop_img, crop_img, mask=mask)
x,y,w,h= cv2.boundingRect(displayCnt)
cropped_img=crop_img[y:y+h, x:x+w]
color = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)

cv2.imwrite('gray.png', color)


# separate digits -> ignore white hole between last 2 digits
img_name = "ex2.png"
# do binary color transform
# threshold the warped image, then apply a series of morphological
# operations to cleanup the thresholded image
thresh = cv2.threshold(color, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
print(thresh.shape)
cv2.imwrite('thresh.png', thresh[:, 0:170])
# find contours in the thresholded image, then initialize the
# digit contours lists

matrix = np.asarray(thresh[:, 0:170])
num_rows, num_cols = matrix.shape
block_tuples = []
blobs = []
l_cnt = 0
r_cnt = 0

row = int(num_rows / 2)
flag = False
# get black regions
for c in range(matrix.shape[1]):
    if (matrix[row, c] == 0):
        if not flag:
            l_cnt = r_cnt
            flag = not flag
    else:
        if flag:
            block_tuples.append(l_cnt)
            blobs.append(r_cnt - l_cnt)
            flag = not flag
        
    r_cnt = r_cnt + 1

top_three = sorted(zip(blobs, block_tuples), reverse=True) [:3]
VERT_PADDING = 10
HZ_PADDING = 3

for i in range(2):
    len, idx = top_three[i]
    post_len, post_idx = top_three[i+1]
    if i == 1:
        cv2.imwrite("dig_2.png", color[VERT_PADDING:num_rows-VERT_PADDING, idx+len - HZ_PADDING:post_idx-10 + HZ_PADDING])
    else: 
        cv2.imwrite("dig_1.png", color[VERT_PADDING:num_rows-VERT_PADDING, idx+len - HZ_PADDING:post_idx + HZ_PADDING])

idx_3, len_3 = top_three[2]
cv2.imwrite("dig_3.png", color[ VERT_PADDING:num_rows-VERT_PADDING, idx_3+len_3 - HZ_PADDING:num_cols - HZ_PADDING])
