import cv2
import pytesseract
from pytesseract import image_to_string
import os
import numpy as np
import time
import platform

def num_reader(img):    
    
    # to use pytesseract
    # pytesseract.pytesseract.tesseract_cmd = '/opt/homebrew/bin/tesseract'
    # pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'
    
    # converting to grayscale
    gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("test", gry)
    # cv2.waitKey(0)
    
    # # blur
    # blur = cv2.GaussianBlur(gry, (0,0), sigmaX=33, sigmaY=33)

    # # divide
    # divide = cv2.divide(gry, blur, scale=255)
    
    # setting threshold to make all pixels white or black
    for i in range(45, 225, 5):

        thr = cv2.threshold(gry, i, 225, cv2.THRESH_BINARY_INV)[1]
        
        # # apply morphology
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        # morph = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, kernel)

        # extracting text from the image as a single line
        txt = image_to_string(thr, config='--psm 10  --oem 3 -c tessedit_char_whitelist=0123456789')
        digits = "".join([t for t in txt if t != '|']).strip()
        print(f"Extracted digits: {digits}")

        # only keeping numeric digits
        chars = []
        for digit in digits:
            if digit.isnumeric():
                chars.append(digit)
                
        # print("In",filename,"detecting",chars)
        if len(chars) == 3:
            print("Result: " + chars[0] + chars[1] + "." + chars[2])
            # cv2.imshow("test", thr)
            # cv2.waitKey(0)
            
            # cv2.imshow("test", morph)
            # cv2.waitKey(0)
            break
        else:
            continue

def read_camera(): 
    current_os = platform.system()

    if current_os == "Windows":
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  
    elif current_os == "Darwin": 
        cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
    elif current_os == "Linux":
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(0)

    # set lowest point of webcam 5 inches above the surface
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # crop frame first
        num_reader(frame)
        cv2.imshow("test", frame)
        cv2.waitKey(1)

def read_img(img_path: str, show_images= False):
    img = cv2.imread("test/IMG_2554(1).JPG")
    
    #converting to grayscale
    gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if show_images:
        cv2.imshow("test", gry)
        cv2.waitKey(0)

    #setting threshold to make all pixels white or black
    thr = cv2.threshold(gry, 45, 225,
                        cv2.THRESH_BINARY_INV)[1]
    if show_images:
        cv2.imshow("test", thr)
        cv2.waitKey(0)

    #extracting text from the image as a single line

    #101 [0,0,1]
    txt = image_to_string(thr, config='--psm 10  --oem 3 -c tessedit_char_whitelist=0123456789')
    digits = "".join([t for t in txt if t != '|']).strip()

    #only keeping numeric digits
    chars = []
    for digit in digits:
        print(digits)
        if digit.isnumeric():
            chars.append(digit)
            
    print(chars)
    return digits

read_img("sdfs")