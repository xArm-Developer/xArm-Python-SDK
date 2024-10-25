import cv2
import os
import numpy as np

save_dir = 'computer_vision/number_detection/test/test_cropped'
save_path = os.path.join(save_dir, '08.0.jpg')

file_dir = "computer_vision/number_detection/test/UncroppedImagesSet2"
filename = os.path.join(file_dir, "08.0.jpg")
print(filename)
image = cv2.imread(filename)

gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# g_image = gray_image[450:700, 750:1400]

# print(gray_image.shape)

# cv2.imshow('Loaded Image', g_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# cv2.imshow('Loaded Image', cropped_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

class DotMatrix(np.ndarray):
    def __new__(cls, white_border_top, white_border_side, black_box_height, black_box_width):
        # Create the matrix
        a = np.ones((2*white_border_top + black_box_height, 2*white_border_side + black_box_width))
        a[white_border_top:-white_border_top, white_border_side:-white_border_side] = -1 * a[white_border_top:-white_border_top, white_border_side:-white_border_side]

        # Use np.asarray to create the base object
        obj = np.asarray(a).view(cls)

        # Return the newly created object
        return obj

    def __init__(self, white_border_top, white_border_side, black_box_height, black_box_width):
        # Initialize additional attributes
        self.bordh = white_border_top
        self.bordw = white_border_side
        self.boxh = black_box_height
        self.boxw = black_box_width
        self.h = 2*self.bordh + self.boxh
        self.w = 2*self.bordw + self.boxw

    def __array_finalize__(self, obj):
        # If slicing or view is called, ensure attributes are propagated
        if obj is None:
            return
        self.bordh = getattr(obj, 'bordh', None)
        self.bordw = getattr(obj, 'bordw', None)
        self.boxh = getattr(obj, 'boxh', None)
        self.boxw = getattr(obj, 'boxw', None)

def multiplyImageAtCoords(matrix: DotMatrix, image: np.ndarray, coords) -> int:
    row = coords[1]
    height = coords[0]

    img = image[height:height+matrix.h, row:row+matrix.w]

    img_flat = img.ravel()
    matrix_flat = matrix.ravel()
    # print(coords)
    return np.dot(img_flat, matrix_flat)

def multiplyImage(matrix: DotMatrix, image: np.ndarray):
    (imgh, imgw) = image.shape[:2]
    maxh = imgh - matrix.h
    maxw = imgw - matrix.w

    print(imgh, imgw)
    print(maxh, maxw)
    print(matrix.h, matrix.w)

    max_product = [(0,0),-128]
    for h in range(maxh):
        for w in range(maxw):
            prod = multiplyImageAtCoords(matrix, image, (h,w))
            if prod>max_product[1]:
                max_product = [[h,w], prod]
    return max_product

# loc = multiplyImage(dot_matrix, gray_image)

# cropped_image = image[loc[0][0]:loc[0][0]+dot_matrix.h, loc[0][1]:loc[0][1]+dot_matrix.w]

def resizeImage(image: np.ndarray, resize_factor):
    (h, w) = image.shape[:2]
    h2 = int(h*resize_factor)
    w2 = int(w*resize_factor)

    resized_image = cv2.resize(image, (w2, h2), interpolation=cv2.INTER_AREA)
    return resized_image

def cropImage(filename, file_dir, save_dir, dot_matrix: DotMatrix, resize_factor = 0.7):
    filepath = os.path.join(file_dir, filename)
    image = cv2.imread(filepath)

    resized_image = resizeImage(image, resize_factor)

    gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    # target_image = gray_image - 127

    loc = multiplyImage(dot_matrix, gray_image)
    cropped_image = image[loc[0][0]:loc[0][0]+dot_matrix.h, loc[0][1]:loc[0][1]+dot_matrix.w]

    save_path = os.path.join(save_dir, filename)
    cv2.imwrite(save_path, cropped_image)

resize_factor = 0.6

dot_matrix = DotMatrix(int(50*resize_factor), int(75*resize_factor), int(150*resize_factor), int(250*resize_factor))

files = [f for f in os.listdir(file_dir) if os.path.isfile(os.path.join(file_dir, f))]
for f in files:
    cropImage(f, file_dir, save_dir, dot_matrix)

