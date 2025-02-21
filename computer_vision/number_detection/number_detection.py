import cv2
import pytesseract
from pytesseract import image_to_string
import numpy as np
import re

from crop_tool import crop_image

def preprocessing(img,debug=False,still=False):
    #grayscale
    if img is None or img.size == 0:
         return None

    img = cv2.bitwise_not(img)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # img = cv2.equalizeHist(img)
    # img = cv2.GaussianBlur(img,(3,3),0)
    img = cv2.medianBlur(img,5)

    #NOTE: was 200, 215 clears 06.5.jpg issue, top 240
    img = cv2.threshold(img,200,240, cv2.THRESH_BINARY)[1]

    #NOTE: was 4x4 and 1 it
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(4,4))
    img = cv2.dilate(img,kernel=kernel,iterations=1)

    #NOTE: was 3x3 and 3 it
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    img = cv2.erode(img,kernel=kernel,iterations=3)

    #NOTE: 3 it
    img = cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel, iterations=3)
    
    # NOTE: comment these two lines out in order to display processed results
    if debug:
        cv2.imshow("final product",img)
    if still:
        cv2.waitKey(0)

    return img

def read_img(img):

    img = preprocessing(img)
    if img is None:
         return ""
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

# run video feed and 

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera not found or not accessible")
        exit(1)
    
    # calibrate camera

    #config 
    try:
        i = 0
        while True:
                ret, frame = cap.read()

                #for now frame is our image
                

                crop_frame, frame = crop_image(frame)

                if not ret: 
                    print("Failed to grab frame")
                    break
                
                processed = preprocessing(frame)
                filtered_data = read_img(crop_frame)
                print(f'\rFiltered data: {filtered_data}', end='', flush=True)

                cv2.imshow('Video feed',frame)
                cv2.imshow('processed',processed)
            # cv2.imshow('crop feed', crop_frame)

            # exit check
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()