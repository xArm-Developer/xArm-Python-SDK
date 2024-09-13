import cv2
import numpy as np
from getMatrix import getMatrix
import time
from pupil_apriltags import Detector

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
    cx = None
    cy = None
    for contour in contours:
        
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)

            (center, axes, angle) = ellipse

            # Center of the ellipse
            cx, cy = int(center[0]), int(center[1])

            # Side length of the square24
            s = 50

            # Calculate top-left and bottom-right coordinates of the square
            top_left = (cx - s // 2, cy - s // 2)
            bottom_right = (cx + s // 2, cy + s // 2)
            cv2.ellipse(image, ellipse, (0, 255, 0), 2)
            cv2.rectangle(image, top_left, bottom_right, (255, 0, 0), 2)  # Green square with a thickness of 2
            
    return cx, cy, image

            
def detect_apriltags(frame):
    # Convert the frame to grayscale as AprilTag detection requires a grayscale image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Create an AprilTag detector object
    # You can modify the parameters based on your specific requirements and camera setup
    at_detector = Detector(families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
    
    # Detect AprilTags in the image
    tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)
    
    # Initialize a list to store centers
    centers = []
    
    # Loop over the detected AprilTags
    for tag in tags:
        # Extract the corners of the AprilTag
        corners = tag.corners
        (ptA, ptB, ptC, ptD) = corners
        
        # Draw the bounding box of the AprilTag detection
        cv2.line(frame, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
        cv2.line(frame, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
        cv2.line(frame, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
        cv2.line(frame, tuple(ptD), tuple(ptA), (0, 255, 0), 2)
        
        # Compute and draw the center (x, y)-coordinates of the AprilTag
        cX = int((ptA[0] + ptC[0]) / 2.0)
        cY = int((ptA[1] + ptC[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        
        # Append the center position
        centers.append((cX, cY))
        
        # Optionally, you might want to annotate the tag ID
        cv2.putText(frame, str(tag.tag_id), (int(ptA[0]), int(ptA[1] - 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Return the frame and the list of centers
    if centers:
        print(f'Arm Loc: x: {centers[0][0]} y: {centers[0][1]}')
    else:
        print('Looking for arm....')
    return centers, frame




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
    # warp_matrix = getMatrix()
    # cap = cv2.VideoCapture(1)
    # while True:
        
    #     ret, frame = cap.read()
    #     if not ret:
    #         print('Frame Failed')
        

    #     # Detect circles
    #     #result_frame = detect_paper(frame)
        
    #     margin = 50
    #     mat_width = 8.27  # A4 paper width in inches
    #     mat_height = 11.69  # A4 paper height in inches
    #     output_width = int(mat_width * 100) + 2 * margin
    #     output_height = int(mat_height * 100) + 2 * margin
    #     result_frame = cv2.warpPerspective(frame, warp_matrix, (output_width, output_height))
    #     center, result_frame = detect_apriltags(result_frame)
    #     #beaker_x, beaker_y, result_frame = detect_ellipse(frame)
        
        

    #     # Display the result frame
    #     if result_frame is not None:
    #         cv2.imshow("Circle Detection", result_frame)

    #     # Exit on 'q' press
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # # Release the webcam and close all windows
    # cap.release()
    # cv2.destroyAllWindows()

    ########################################################

    cap = cv2.VideoCapture(1)  # Use 0 for webcam

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Process the frame
        processed_frame, centers = detect_apriltags(frame)
        
        # Display the frame
        cv2.imshow('AprilTag Detection', processed_frame)
        
        # Press 'q' to close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()
