import cv2
import numpy as np
from getMatrix import getMatrix
import robotpy_apriltag


def find_beaker_center(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
    return None

def detect_apriltag_and_beaker(frame, warped, matrix):
    # Initialize the AprilTag detector

    # Convert frame to grayscale (required for detection)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the image
    tag_center = None
    beaker_center = None  # Assuming there's another function or method to find this
    tag = robotpy_apriltag.AprilTagDetector()
    tag.addFamily("tag25h9", 3)

    l: robotpy_apriltag.AprilTagDetection = tag.detect(gray)
    if l is not None:
        for i in l:
            # print(i.getCenter())
            point = i.getCenter()
            x: int = int(point.x)
            y: int = int(point.y)
            tag_center = (x, y)

            point = np.array([x, y, 1])
            warped_point_homogeneous = np.dot(matrix, point)
            warped_point = warped_point_homogeneous[:2] / warped_point_homogeneous[2]

            x = int(warped_point[0])
            y = int(warped_point[1])
            
            half_side = 10 // 2
            top_left = (x - half_side, y - half_side)
            bottom_right = (x + half_side, y + half_side)

            cv2.drawMarker(warped, (x,y), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

    # Process each detected AprilTag
    # for result in results:
    #     tag_center = (int(result.center[0]), int(result.center[1]))
    #     corners = np.array(result.corners).astype(int)
    #     # Draw the detected tag center and outline
    #     cv2.circle(frame, tag_center, 5, (0, 0, 255), -1)
    #     cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

    return tag_center, beaker_center, warped

def main():
    # cap = cv2.VideoCapture(1)  # Use 0 for webcam

    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         print("Failed to get frames")
    #         break
        
        
    #     frame = detect_paper(frame)
 
    #     cv2.imshow('Frame', frame)

    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    warp_matrix = getMatrix()

    cap = cv2.VideoCapture(1)
    paper_width_mm = 210
    paper_height_mm = 297
    margin = 200
    new_width = paper_width_mm + 2 * margin
    new_height = paper_height_mm + 2 * margin
    while True:
        
        ret, frame = cap.read()
        if not ret:
            print('Frame Failed')
        
    
        warped = cv2.warpPerspective(frame, warp_matrix, (new_width, new_height))
        #margin = add_margin(warped)

        tag_center, beaker_center, final_frame = detect_apriltag_and_beaker(frame, warped, warp_matrix)

        cv2.imshow('Final', final_frame)
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    main()


        