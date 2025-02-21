# PCR Number Detection

### We are using the cv2 and pytesseract libraries from Python to extract the pipette measurement from a photo/webcam.

### The cv2 library helps us transform our image frames into a cropped black and white image with reduced noise. 

### The pytesseract library then helps us read specific characters from the image, with our goal being that it accurately reads the pipette measurement.

### Things to Remember
1. The pytesseract.pytesseract.tesseract_cmd field must be set to different paths depending on OS.
2. The 2nd argument for cv2.VideoCapture also depends on OS.

### If you need to compile prefixMin.c
- gcc -shared -o library/number-detection-pkg.dylib -fPIC -arch x86_64 prefixMin.c 

# parallelPrefix.py

This module implements a prefix-minimum solution to the cropping problem using OpenCV and NumPy. It includes functions for image processing, such as finding contours, rotating images, and cropping images based on certain heuristics.

## Functions

### `init()`
Initializes the module. This function is called once to set up any necessary configurations.

### `prefix_min(arr, results, axis=0)`
Calculates the prefix minimum of an array along a specified axis.

- `arr`: Input array.
- `results`: Dictionary to store the results.
- `axis`: Axis along which to calculate the prefix minimum.

### `get_mask(results: dict)`
Generates a combined mask based on the prefix minimum results.

- `results`: Dictionary containing prefix minimum results.

### `get_skew_angle(contour)`
Calculates the skew angle of a given contour.

- `contour`: Input contour.

### `rotateImage(cvImage, angle: float)`
Rotates an image around its center by a specified angle.

- `cvImage`: Input image.
- `angle`: Angle by which to rotate the image.

### `crop_image(img)`
Crops the input image based on contour selection heuristics.

- `img`: Input image.

# prefixMin.c

This C module implements the prefix minimum algorithm for a 2D array. The prefix minimum is calculated in different directions based on the specified case.

## Functions

### `void calculate_prefix_min(int *arr, int *prefix_min_arr, int rows, int cols, int direction)`
Calculates the prefix minimum of a 2D array in a specified direction.

- `arr`: Input 2D array (flattened).
- `prefix_min_arr`: Output array to store the prefix minimum results (flattened).
- `rows`: Number of rows in the input array.
- `cols`: Number of columns in the input array.
- `direction`: Direction in which to calculate the prefix minimum. The possible values are:
  - `1`: Left to right
  - `2`: Bottom to top
  - `3`: Right to left
  - `4`: Top to bottom

# number_detection.py

This module implements number detection using OpenCV and OCR (Optical Character Recognition). It processes video feed from a camera to detect and recognize numbers.

## Functions

### `preprocessing(img, debug=False, still=False)`

Preprocesses the input image for OCR (Optical Character Recognition).

#### Parameters:
- `img`: The input image to be processed.
- `debug` (optional): A boolean flag to enable debug mode. Default is `False`.
- `still` (optional): A boolean flag to indicate if the image is a still image. Default is `False`.

#### Returns:
- `img`: The preprocessed image. Returns `None` if the input image is `None` or empty.

#### Description:
1. Checks if the input image is `None` or empty. If so, returns `None`.
2. Inverts the colors of the image using bitwise not operation.
3. Converts the image to grayscale.
4. Applies a median blur to the image with a kernel size of 5.
5. Applies a binary threshold to the image with a threshold value of 200 and a maximum value of 240.
6. Dilates the image using a cross-shaped structuring element with a kernel size of 4x4 and 1 iteration.
7. Erodes the image using a cross-shaped structuring element with a kernel size of 3x3 and 3 iterations.


### `read_img(img)`

Processes an input image to detect and format numbers using OCR (Optical Character Recognition).

#### Parameters:
- `img`: The input image to be processed.

#### Returns:
- `filtered_data`: A string containing the detected and formatted numbers.

#### Description:
1. The function preprocesses the input image using the `preprocessing` function.
2. If the preprocessed image is `None`, it returns an empty string.
3. It uses Tesseract OCR to extract text from the preprocessed image, specifically looking for numbers and dots.
4. The extracted text is filtered to remove any characters that are not numbers or dots.
5. Heuristic corrections are applied to the filtered data:
   - If the length of the filtered data is 3, a dot is inserted after the second character.
   - If the length of the filtered data is 4, the second to last character is replaced with a dot.
6. The function returns the formatted string containing the detected numbers.

# Testing

For number detection we are using the UnitTest framework provided by python, this 
allows us to systematically evaluate our code while also giving us a great degree of 
control

# test-crop.py

This module contains unit tests for the `crop_image` function, which is used to crop images based on certain heuristics. The tests read images from a specified directory, process them, and save the results to another directory.

## Functions

### `setUp(self)`
Sets up the test environment by creating necessary directories for reading and writing images.

- `self.rd_dir`: Directory containing the uncropped images.
- `self.wr_dir`: Directory where the cropped images will be saved.

### `test_single(self)`
Tests the `crop_image` function on a single image file and displays the result.

- Reads an image file from `self.rd_dir`.
- If the image fails to load, prints an error message and exits.
- Measures the time taken to crop the image.
- Displays the cropped image.

## Usage

**NOTE:** Changing the variable `testAll` in `main(testAll=False)` allows you to 
control whether to test all the files or one specifically. Make it `True` in order 
to test all files

### test single file

In order to test a specific image input (e.g "00.0.jpg") evoke:
`python test-crop.py [FILENAME]`

A window will pop-up showing you the results, **press space on pop-up window to close results and end test**

### test all files

**NOTE:** Set `testAll` to True

In order to test all images in `test/UncroppedImagesSet2/` evoke:
`python test-crop.py`

You can find all cropped results in `test/test-cropped`