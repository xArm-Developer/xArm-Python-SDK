import cv2
import pytesseract
from pytesseract import image_to_string
import os
import numpy as np
import time
import platform


# is this function redundant? 
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
        gry = thr
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
            # print("Result: " + chars[0] + chars[1] + "." + chars[2])
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


#TODO find a way to get confidence output from a read

def read_img(img_path: str, show_images= True):
    img = cv2.imread(img_path)
    
    #converting to grayscale
    gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # if show_images:
    #     cv2.imshow("test", gry)
    #     cv2.waitKey(0)


    #setting threshold to make all pixels white or black
  
    thr = cv2.threshold(gry, 45, 225,
                    cv2.THRESH_BINARY_INV)[1]

    # if show_images:
    #     cv2.imshow("test", thr)
    #     cv2.waitKey(0)


    # NOTE: experiement with the size of the structuring element


    # Experiment with different structuring elements
    # kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    # kernel_rect2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))

    # Apply morphological operations with different kernels
    morph_rect = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, kernel_rect)

    if show_images:
        cv2.imshow("Morphology Rect", morph_rect)
        cv2.waitKey(0)
    
    # Choose the best result based on visual inspection or further processing
    morph = morph_rect  # or morph_ellipse, morph_cross
    #extracting text from the image as a single line

    #101 [0,0,1]
    # have it return a dictionary from the results
    data = pytesseract.image_to_data(morph, config='--psm 8  --oem 3 -c tessedit_char_whitelist=.0123456789', 
                        output_type=pytesseract.Output.DICT)
    
    txt = data['text']
    confidences = data['conf']

    print(f'CONFIDENCE: {confidences}')

    digits = "".join([t for t in txt if t != '|']).strip()

    #only keeping numeric digits
    chars = []
    for digit in digits:
        print(digits)
        if digit.isnumeric() or digit == '.':
            chars.append(digit)
            
    print(chars)
    return ''.join(chars) 

# read_img("sdfs")