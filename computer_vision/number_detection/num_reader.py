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
    
#imports
import cv2
import pytesseract
from pytesseract import image_to_string

#adding a comment to show git stuff

#importing image to extract numbers from
img = cv2.imread("/Users/snaehashriram/Coding/PCR_Automation/computer_vision/number_detection/14-5.jpg")
#converting to grayscale
gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("test", gry)
cv2.waitKey(0)

#setting threshold to make all pixels white or black
thr = cv2.threshold(gry, 45, 225,
                    cv2.THRESH_BINARY_INV)[1]
cv2.imshow("test", thr)
cv2.waitKey(0)

#to use pytesseract
pytesseract.pytesseract.tesseract_cmd = '/opt/homebrew/bin/tesseract'

#extracting text from the image as a single line
txt = image_to_string(thr, config='--psm 10  --oem 3 -c tessedit_char_whitelist=0123456789')
digits = "".join([t for t in txt if t != '|']).strip()

#only keeping numeric digits
chars = []
for digit in digits:
    if digit.isnumeric():
        chars.append(digit)
        
print(chars)
print("Result: " + chars[0] + chars[1] + "." + chars[2])