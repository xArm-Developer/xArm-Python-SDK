import cv2
import pytesseract
from pytesseract import image_to_string
import os
import numpy as np
import time
import platform
import re


'''
Noise Removal:
    - Clustering algorithm based on camera positions

    - Gaussian Blur
    
    - hybrid between neural and pretrained
        - build a layer ontop of pretrained model with a neural network
        - needs MORE DATA LOTS OF DATA
    
    - Find out what EXACTLY our camera will be reading the values

    - Make cropping algo
'''

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

def preprocessing(img):
    #grayscale
    img = cv2.bitwise_not(img)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.threshold(img,200,235, cv2.THRESH_BINARY)[1]
    cv2.imshow("binary",img)
    #NOTE: was 4x4
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(4,4))
    img = cv2.dilate(img,kernel=kernel,iterations=1)
    #NOTE: was 3x3
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    img = cv2.erode(img,kernel=kernel,iterations=3)
    img = cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel, iterations=3)

    img = cv2.medianBlur(img,5)
    
    # NOTE: comment these two lines out in order to display processed results

    cv2.imshow("final product",img)
    cv2.waitKey(0)

    return img

def read_img(img_path: str, show_images= True):
    img = cv2.imread(img_path)
    img = preprocessing(img)
    data = pytesseract.image_to_string(img,config='--psm 8  --oem 3 -c tessedit_char_whitelist=.0123456789')
    
    filtered_data = re.sub(r'[^0-9.]', '', data)

    # heuristic corrections

    # if size of just 3, add a . , if size of just 4, replace second to last with .
    # print(filtered_data)
    if (len(filtered_data) == 3):
            filtered_data = filtered_data[:2] + '.' + filtered_data[2:]
            
    elif (len(filtered_data) == 4):
                filtered_data = f"{filtered_data[:2]}.{filtered_data[3:]}"
    
    # NOTE: to see char results from OCR uncomment HERE
    # print(filtered_data)

    return filtered_data
