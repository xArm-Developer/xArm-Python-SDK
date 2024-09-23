# PCR Number Detection

### We are using the cv2 and pytesseract libraries from Python to extract the pipette measurement from a photo/webcam.

### The cv2 library helps us transform our image frames into a cropped black and white image with reduced noise. The pytesseract library then helps us read specific characters from the image, with our goal being that it accurately reads the pipette measurement.

### Things to Remember
1. The pytesseract.pytesseract.tesseract_cmd field must be set to different paths depending on OS.
2. The 2nd argument for cv2.VideoCapture also depends on OS.
