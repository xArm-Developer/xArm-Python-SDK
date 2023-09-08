import cv2
import numpy as np

# resize image
# load the example image
image = cv2.imread("sample2.jpeg")
crop_img = image[350:600, 200:450]
cv2.imwrite('cropped.png', crop_img)

# pre-process the image by resizing it, converting it to
# graycale, blurring it, and computing an edge map
gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
# Applying Gaussian blurring with a 5×5 kernel to reduce high-frequency noise

thresh1 = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
thresh1 = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel1)
cv2.imwrite('gray.png', thresh1)

blurred = cv2.GaussianBlur(gray, (5, 5), 0)
edged = cv2.Canny(blurred, 50, 200, 255)

def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
            "otherwise OpenCV changed their cv2.findContours return "
            "signature yet again. Refer to OpenCV's documentation "
            "in that case"))

    # return the actual contours array
    return cnts

def order_points(pts):
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype = "float32")
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # return the ordered coordinates
    return rect

def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    # return the warped image
    return warped


# get black box contour: 
# find contours in the edge map, then sort them by their
# size in descending order
cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)

cnts = grab_contours(cnts)
cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
displayCnt = None
# loop over the contours
for c in cnts:
    # approximate the contour
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    # if the contour has four vertices, then we have found
    # the thermostat display
    if len(approx) == 4:
        displayCnt = approx
        break


# extract the thermostat display, apply a perspective transform
# to it
warped = four_point_transform(gray, displayCnt.reshape(4, 2))
output = four_point_transform(image, displayCnt.reshape(4, 2))
cv2.imwrite('warped.png', warped)
# separate digits -> ignore white hole between last 2 digits
img_name = "ex2.png"
# do binary color transform
# threshold the warped image, then apply a series of morphological
# operations to cleanup the thresholded image
thresh = cv2.threshold(warped, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

cv2.imwrite('thresh.png', thresh)
# find contours in the thresholded image, then initialize the
# digit contours lists

matrix = np.asarray(thresh)
print(matrix.shape)
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

print(blobs)
print(block_tuples)
top_three = sorted(zip(blobs, block_tuples), reverse=True) [:3]
print(top_three)
vert_padding = 7
hz_padding = 5

for i in range(2):
    len, idx = top_three[i]
    post_len, post_idx = top_three[i+1]
    if i == 1:
        cv2.imwrite('dig_{index}.png'.format(index = i + 1), warped[vert_padding:num_rows-vert_padding, idx+len - hz_padding:post_idx-10 + hz_padding])
    else: 
        cv2.imwrite('dig_{index}.png'.format(index = i + 1), warped[vert_padding:num_rows-vert_padding, idx+len - hz_padding:post_idx + hz_padding])

idx_3, len_3 = top_three[2]
cv2.imwrite('dig_3.png', warped[ vert_padding:num_rows-vert_padding, idx_3+len_3 - hz_padding:num_cols + hz_padding])
