# import cv2
# import numpy as np

# cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# while True:
#     ret, frame = cap.read()
    
#     if not ret:
#         break
    
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
#     cv2.imshow("test", frame)
    
#     cv2.waitKey(0)
    

import cv2
import pytesseract
from pytesseract import image_to_string

img = cv2.imread("C:\\Users\\adity\\OneDrive\\Documents\\PCR\\PCR_Automation\\computer_vision\\number_detection\\IMG_5916.jpg")

# crop_img = img[:,:]
# cv2.imshow("cropped", crop_img)
# cv2.waitKey(0)

gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("test", gry)
cv2.waitKey(0)

thr = cv2.threshold(gry, 45, 225,
                    cv2.THRESH_BINARY_INV)[1]
cv2.imshow("test", thr)
cv2.waitKey(0)

pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'

txt = image_to_string(thr, config="--psm 7")
digits = "".join([t for t in txt if t != '|']).strip()

chars = []
for digit in digits:
    if digit.isnumeric():
        chars.append(digit)
        
print("Result: " + chars[0] + chars[1] + "." + chars[2])