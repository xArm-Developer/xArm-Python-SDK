import cv2
import numpy as np
from getMatrix import getMatrix
import time
import apriltag

def detect_paper(frame):
        
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection using the Canny algorithm
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out contours based on area to find the contour of the paper
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour
    
    # Draw contour on original frame and perform perspective transformation
    if max_contour is not None:
        # Draw contour on original frame
        #cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 2)
        
        # Calculate the perspective transformation matrix
        epsilon = 0.02 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
        
        # Check if the contour has 4 points
        if len(approx) == 4:
            paper_contour = approx.reshape(4, 2)
            paper_width = 8.27  # A4 paper width in inches
            paper_height = 11.69  # A4 paper height in inches
            
            # Define dimensions of the output image with margin
            margin = 50  # Adjust this value as needed
            output_width = int(paper_width * 100) + 2 * margin
            output_height = int(paper_height * 100) + 2 * margin
            
            # Define desired dimensions of the paper in the output image
            desired_dimensions = np.array([[margin, margin], [output_width - margin, margin],
                                            [output_width - margin, output_height - margin], [margin, output_height - margin]], dtype=np.float32)
            matrix = cv2.getPerspectiveTransform(paper_contour.astype(np.float32), desired_dimensions)
            
            # Apply the perspective transformation to the frame
            frame = cv2.warpPerspective(frame, matrix, (output_width, output_height))
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            
    return frame

    


def detect_circles(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection.
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Detect edges using Canny edge detector.
    edges = cv2.Canny(blurred, 50, 150)

    # adjusted_frame = cv2.convertScaleAbs(edges, alpha=3.5, beta=0)

    # Apply Hough Circle Transform to find circles in the frame.
    circles = cv2.HoughCircles(
        adjusted_frame,
        cv2.HOUGH_GRADIENT,
        1,
        30,
        param1 = 50,
        param2 = 30,
        minRadius= 20,
        maxRadius=100
    )

    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers.
        circles = np.round(circles[0, :]).astype("int")

        # Draw circles on the frame.
        x_pixels = None
        y_pixels = None
        for (x, y, r) in circles:
            x_pixels = x
            y_pixels = y
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
    return image

def detect_ellipse(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection.
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Detect edges using Canny edge detector.
    edges = cv2.Canny(blurred, 50, 150)

    # adjusted_frame = cv2.convertScaleAbs(edges, alpha=3.5, beta=0)

    # Apply Hough Circle Transform to find circles in the frame.
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)

            (center, axes, angle) = ellipse

            # Center of the ellipse
            cx, cy = int(center[0]), int(center[1])

            # Side length of the square
            s = 50

            # Calculate top-left and bottom-right coordinates of the square
            top_left = (cx - s // 2, cy - s // 2)
            bottom_right = (cx + s // 2, cy + s // 2)
            cv2.ellipse(image, ellipse, (0, 255, 0), 2)
            cv2.rectangle(image, top_left, bottom_right, (255, 0, 0), 2)  # Green square with a thickness of 2
            
    return cx, cy, image

            
def detect_apriltag(frame):
    # Convert frame to grayscale
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Create an AprilTag detector
    detector = apriltag.Detector()
    
    # Detect AprilTags in the frame
    detections = detector.detect(img_gray)
    
    if len(detections) > 0:
        # Get the first detected tag
        tag = detections[0]
        
        # Get the x and y position of the tag
        x = int(tag.center[0])
        y = int(tag.center[1])
        
        # Highlight the tag in the frame
        for pt in tag.corners:
            pt = (int(pt[0]), int(pt[1]))
            cv2.circle(frame, pt, 5, (0, 255, 0), -1)
        print("AprilTag detected at position (x={}, y={})".format(x, y))
        return x, y, frame
    else:
        print("Looking for arm...")
        return None, None, None





    # # Convert the image to grayscale
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # # Apply Gaussian blur to reduce noise
    # gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # # Detect circles using HoughCircles
    # circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=200, param2=30, minRadius=10, maxRadius=100)
    
    # circle_x = None
    # circle_y = None
    # circle_r = None
    # if circles is not None:
    #     circles = circles[0]  # Convert circles to numpy array
    #     for circle in circles:
    #         x, y, r = circle
    #         center_coordinates = (int(x), int(y))
    #         # Draw the circle outline
    #         cv2.circle(image, center_coordinates, int(r), (0, 255, 0), 2)
    #         # Draw the center of the circle
    #         cv2.circle(image, center_coordinates, 2, (0, 0, 255), 3)
    #         # Print the coordinates of the center
    #         circle_x = x
    #         circle_y = y
    #         circle_r = r
    #         print("Center coordinates of the detected circle:", center_coordinates)
    # else:
    #     circle_x = None
    #     circle_y = None
    #     circle_r = None
    # return {
    #     'image': image,
    #     'x' : circle_x,
    #     'y' : circle_y,
    #     'r' : circle_r
    # }

def main():
    # Initialize the webcam
    warp_matrix = getMatrix()
    cap = cv2.VideoCapture(1)
    while True:
        
        ret, frame = cap.read()
        if not ret:
            print('Frame Failed')
        

        # Detect circles
        #result_frame = detect_paper(frame)
        
        margin = 50
        mat_width = 8.27  # A4 paper width in inches
        mat_height = 11.69  # A4 paper height in inches
        output_width = int(mat_width * 100) + 2 * margin
        output_height = int(mat_height * 100) + 2 * margin
        result_frame = cv2.warpPerspective(frame, warp_matrix, (output_width, output_height))
        beaker_x, beaker_y, result_frame = detect_ellipse(result_frame)
        arm_x, arm_y, tag_frame = detect_apriltag(result_frame)
        

        # Display the result frame
        if result_frame is not None:
            cv2.imshow("Circle Detection", tag_frame)

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
