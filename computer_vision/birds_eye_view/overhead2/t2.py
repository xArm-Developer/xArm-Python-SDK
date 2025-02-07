import cv2
import numpy as np
from getMatrix import getMatrix
import robotpy_apriltag

# Return the coordinates of the center of the image
def find_beaker_center(image):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Find the 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.drawMarker(image, (cx,cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            return (cx, cy)
            

    return None

# Draw a circle around the beaker and return the coordinates of the beaker and the
# image frame
def find_beaker(frame):

    # Transform frame to find clear edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    adjusted_frame = cv2.convertScaleAbs(edges, alpha=3.5, beta=0)

    # Locate circles on screen
    circles = cv2.HoughCircles(
            adjusted_frame,
            cv2.HOUGH_GRADIENT,
            1,
            30,
            param1=50,
            param2=30,
            minRadius=20,
            maxRadius=100
        )
    x = None
    y = None
    if circles is not None:
            # Convert the (x, y) coordinates and radius of the circles to integers.
            circles = np.round(circles[0, :]).astype("int")

            # Draw circles on the frame.
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
            return (x,y), frame
    
    return None, frame
    
# Draw a circle around the beaker and return the warped frame and coordinates
def find_beaker_warp(frame, warped, matrix):

    # Transform frame to find clear edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    adjusted_frame = cv2.convertScaleAbs(edges, alpha=3.5, beta=0)

    # Locate circles on screen
    circles = cv2.HoughCircles(
            adjusted_frame,
            cv2.HOUGH_GRADIENT,
            1,
            30,
            param1=50,
            param2=30,
            minRadius=20,
            maxRadius=100
        )
    
    x = None
    y = None

    if circles is not None:
            # Convert the (x, y) coordinates and radius of the circles to integers.
            circles = np.round(circles[0, :]).astype("int")

            # Draw circles on the frame.
            for (x, y, r) in circles:
                point = np.array([x, y, 1])
                warped_point_homogeneous = np.dot(matrix, point)
                warped_point = warped_point_homogeneous[:2] / warped_point_homogeneous[2]

                x = int(warped_point[0])
                y = int(warped_point[1])

                cv2.circle(warped, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(warped, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
            return (x,y), warped
    
    return None, warped
     
# Detect and return the coordinates of the center of the AprilTag
def detect_apriltag(frame, warped, matrix):
    # Initialize the AprilTag detector

    # Convert frame to grayscale (required for detection)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the image
    tag_center = None
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
            
            tag_center = (x, y)
            cv2.drawMarker(warped, (x,y), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

    # Process each detected AprilTag
    # for result in results:
    #     tag_center = (int(result.center[0]), int(result.center[1]))
    #     corners = np.array(result.corners).astype(int)
    #     # Draw the detected tag center and outline
    #     cv2.circle(frame, tag_center, 5, (0, 0, 255), -1)
    #     cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

    return tag_center, warped

# Return the distance between two given points
def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    print(point1, point2)
    if point1 and point2:
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    
    return None

# Draw the line from the beaker to the 
def draw_line_and_label(image, point1, point2, distance):
    """Draw a line between two points and label them on the image."""
    # Draw the line
    cv2.line(image, point1, point2, (255, 0, 0), thickness=2)

    # Label each point with its coordinates and distance
    cv2.putText(image, f"{point1}", point1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(image, f"{point2}", point2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # Optionally, put the distance near the midpoint of the line
    midpoint = ((point1[0] + point2[0]) // 2, (point1[1] + point2[1]) // 2)
    cv2.putText(image, f"Dist: {distance:.2f}", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return image

# Main
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

        tag_center, tag_frame = detect_apriltag(frame, warped, warp_matrix)
        beaker_center, beaker_frame = find_beaker(tag_frame)

        distance = calculate_distance(tag_center, beaker_center)

        if distance:
            beaker_frame = draw_line_and_label(beaker_frame, tag_center, beaker_center, distance)

        cv2.imshow('Final', beaker_frame)
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    main()


        