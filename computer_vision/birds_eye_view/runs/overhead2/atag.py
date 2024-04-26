import cv2
import numpy as np

def detect_apriltag(frame, tag_size_mm):
    # Convert frame to grayscale
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Threshold the image to create a binary mask
    _, thresholded = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Iterate through contours to find AprilTag-like shapes
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Check if the polygon has 4 vertices (approximate rectangle)
        if len(approx) == 4:
            # Calculate the area of the polygon
            area = cv2.contourArea(approx)
            
            # Calculate the perimeter of the polygon
            perimeter = cv2.arcLength(approx, True)
            
            # Calculate the circularity of the polygon
            circularity = 4 * np.pi * area / (perimeter ** 2)
            
            # Check if the shape is approximately square and has high circularity
            if 0.9 < circularity < 1.1:
                # Get the bounding rectangle of the contour
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate the center of the rectangle
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Calculate scale factor
                tag_size_pixels = max(w, h)
                scale_factor = tag_size_mm / tag_size_pixels
                
                # Highlight the tag in the frame
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                print(f'Arm Location: ({x}, {y})')
                return center_x, center_y, scale_factor, frame
                
    # Return None if no AprilTag-like shape is found
    print("Looking for arm....")
    return None, None, None, None

cap = cap = cv2.VideoCapture(0)
while True:
    
