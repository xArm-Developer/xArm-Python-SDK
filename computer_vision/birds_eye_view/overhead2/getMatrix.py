import cv2
import numpy as np
import robotpy_apriltag

def detect_apriltag_and_warp(frame, width_in_units, height_in_units):
    width_px = width_in_units
    height_px = height_in_units

    # Convert frame to grayscale for AprilTag detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Initialize the AprilTag detector
    tag_detector = robotpy_apriltag.AprilTagDetector()
    tag_detector.addFamily("tag25h9", 3)  # Adjust the family to match your tag type

    # Detect AprilTags in the image
    tags = tag_detector.detect(gray)

    if len(tags) > 0:
        print(f'{len(tags)} tags detected')

        # Create a buffer to hold corner points
        corners_buf = [0.0] * 8  # 8 floats for 4 corners (x, y) for each corner

        # Get the corners using the buffer
        corners_buf = tags[0].getCorners(corners_buf)  # Fill the buffer with corner values
        
        # Convert the buffer to a numpy array
        pts = np.array(corners_buf).reshape(4, 2).astype("float32")  # Reshape to (4, 2) for corners

        # Define the destination points (the desired size of the output warp)
        dst = np.array([
            [0, 0],
            [width_px - 1, 0],
            [width_px - 1, height_px - 1],
            [0, height_px - 1]], dtype="float32")

        # Compute the perspective transform matrix and warp the image
        M = cv2.getPerspectiveTransform(pts, dst)

        # return M
        return (cv2.warpPerspective(frame, M, (500, 500)), M)
    else:
        print('No tags detected')
        return (None, None)

def getMatrix():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Define the dimensions in pixels
    width_in_units = 100  # Adjust as needed (e.g., for a 5-inch tag at 20 pixels per inch)
    height_in_units = 100  # Adjust as needed

    while True:
        # Capture frame-by-frame from the webcam
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        # Detect and warp the AprilTag
        warped_frame, matrix = detect_apriltag_and_warp(frame, width_in_units, height_in_units)

        # If an AprilTag is detected and warped
        if warped_frame is not None:
            cv2.imshow('Warped AprilTag', warped_frame)
            print('--------------')
        else:
            cv2.imshow('Webcam Feed', frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()
    return matrix

# def getMatrix():
#     cap = cv2.VideoCapture(1)
#     while True:
#         ret, image = cap.read()
#         # Convert the image to grayscale
#         if not ret:
#             print('Frame failed')
#             continue
#         
#         cv2.imshow('Original Frame', image)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         
#         # Apply Gaussian blur to reduce noise
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         
#         # Perform edge detection using the Canny algorithm
#         edges = cv2.Canny(blurred, 50, 150)
#         
#         # Find contours in the edge-detected image
#         contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         
#         # Filter out contours based on area to find the contour of the paper
#         max_area = 0
#         max_contour = None
#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if area > max_area:
#                 max_area = area
#                 max_contour = contour
#         
#         # Draw contour on original frame and perform perspective transformation
#         matrix = []
#         if max_contour is not None:
#             # Calculate the perspective transformation matrix
#             epsilon = 0.02 * cv2.arcLength(max_contour, True)
#             approx = cv2.approxPolyDP(max_contour, epsilon, True)
#             
#             # Check if the contour has 4 points
#             if len(approx) == 4:
#                 paper_contour = approx.reshape(4, 2)
#                 
#                 # Define dimensions of the output image with margin
#                 margin = 200  # Adjust this value as needed
#                 paper_width_px = 210
#                 paper_height_px = 297
# 
#                 new_width = paper_width_px + 2 * margin
#                 new_height = paper_height_px + 2 * margin
# 
#                 dst_points = np.array([
#                     [margin, margin],  # Top-left point shifted by margin
#                     [margin + paper_width_px, margin],  # Top-right point
#                     [margin + paper_width_px, margin + paper_height_px],  # Bottom-right point
#                     [margin, margin + paper_height_px]  # Bottom-left point
#                 ], dtype="float32")
#                 
#                 # Define desired dimensions of the paper in the output image
#                 # desired_dimensions = np.array([[margin, margin], [output_width - margin, margin],
#                 #                                 [output_width - margin, output_height - margin], [margin, output_height - margin]], dtype=np.float32)
#                 matrix = cv2.getPerspectiveTransform(paper_contour.astype(np.float32), dst_points)
#                 
#                 # Apply the perspective transformation to the image
#                 warped = cv2.warpPerspective(image, matrix, (new_width, new_height))
# 
#                 dst_points = np.array([
#                     [0, 0],
#                     [paper_width_px - 1, 0],
#                     [paper_width_px - 1, paper_height_px - 1],
#                     [0, paper_height_px - 1]
#                 ], dtype="float32")
# 
#                 m2 = cv2.getPerspectiveTransform(paper_contour.astype(np.float32), dst_points)
#                 warped2 = cv2.warpPerspective(image, m2, (paper_width_px, paper_height_px))
# 
#                 cv2.imshow('Warped Image', warped)
#                 cv2.imshow('W2', warped2)
# 
#                 pressed = cv2.waitKey(0)
#                 if pressed == ord('a'):
#                     break
#                 else:
#                     cv2.destroyWindow('Warped Image')
#                     continue
# 
#     cap.release()
#     cv2.destroyAllWindows()
#     return matrix
