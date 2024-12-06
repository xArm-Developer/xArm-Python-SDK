from calibration import calibrate
import cv2
import numpy as np
import robotpy_apriltag
from scipy.spatial.transform import Rotation as R

def channel_processing(channel):
    # Adaptive thresholding
    channel = cv2.adaptiveThreshold(channel, 255, 
                                     adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C, 
                                     thresholdType=cv2.THRESH_BINARY, 
                                     blockSize=55, 
                                     C=7)
    # Morphological operations to clean up noise
    channel = cv2.dilate(channel, None, iterations=1)
    channel = cv2.erode(channel, None, iterations=1)
    return channel

def inter_centre_distance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def colliding_circles(circles):
    for index1, circle1 in enumerate(circles):
        for circle2 in circles[index1 + 1:]:
            x1, y1, radius1 = circle1[:3]
            x2, y2, radius2 = circle2[:3]
            # Check for collision or containment
            if inter_centre_distance(x1, y1, x2, y2) < radius1 + radius2:
                return True
    return False

def find_circles(processed, LOW):
    while True:
        circles = cv2.HoughCircles(processed, cv2.HOUGH_GRADIENT, dp=2, minDist=32.0, 
                                   param1=30, param2=LOW)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            if not colliding_circles(circles):
                break
        LOW += 1
        print('Trying with LOW:', LOW)
    print('Number of circles:', len(circles))
    return circles

def draw_circles(circles, output):
    print(len(circles), 'circles found')
    for circle in circles:
        x, y, radius = circle
        # Draw the center of the circle
        cv2.circle(output, (x, y), 1, (0, 255, 0), -1, lineType=cv2.LINE_AA)
        # Draw the outline of the circle
        cv2.circle(output, (x, y), radius, (255, 0, 0), 3, lineType=cv2.LINE_AA)

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    if point1 and point2:
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def draw_line_and_label(image, point1, point2, distance):
    """Draw a line between two points and label them on the image."""
    # Draw the line
    cv2.line(image, point1, point2, (255, 0, 0), thickness=2)

    # Label each point with its coordinates and distance
    # cv2.putText(image, f"{point1}", point1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    # cv2.putText(image, f"{point2}", point2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # Optionally, put the distance near the midpoint of the line
    # midpoint = ((point1[0] + point2[0]) // 2, (point1[1] + point2[1]) // 2)
    # cv2.putText(image, f"Dist: {distance:.2f}", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    # return image

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera not found or not accessible")
        exit()

    ret, mtx, dist, rvecs, tvecs = calibrate(cap)

    config = robotpy_apriltag.AprilTagPoseEstimator.Config(
        tagSize=0.1,   # Replace with your tag size in meters
        fx=mtx[0, 0],        # Replace with your camera's horizontal focal length in pixels
        fy=mtx[1, 1],        # Replace with your camera's vertical focal length in pixels
        cx=mtx[0, 2],        # Replace with your camera's principal point X in pixels
        cy=mtx[1, 2]         # Replace with your camera's principal point Y in pixels
    )
    detector = robotpy_apriltag.AprilTagDetector()
    detector.addFamily("tag25h9")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(config)

    x_positions = []  # Store last N positions
    y_positions = []
    max_history = 100

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame")
                break
            
            # processing for april tag detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray)
            
            # processing for beaker detection
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            # cv2.imshow('blurred', blurred)
            edges = cv2.Canny(blurred, 50, 150)
            # cv2.imshow('edges', edges)
            # adjusted_frame = cv2.convertScaleAbs(edges, alpha=3.5, beta=0)
            # cv2.imshow('adjusted fraem', adjusted_frame)
            circles = cv2.HoughCircles(
                edges,
                cv2.HOUGH_GRADIENT,
                1,
                30,
                param1=50,
                param2=30,
                minRadius=20,
                maxRadius=100
            )

            if circles is not None:
                # Convert the (x, y) coordinates and radius of the circles to integers.
                circles = np.round(circles[0, :]).astype("int")

                # Draw circles on the frame.
                x, y, r = circles[0]
                # for (x, y, r) in circles:
                x_positions.append(x)
                y_positions.append(y)
                if len(x_positions) > max_history:
                    x_positions.pop(0)
                    y_positions.pop(0)
                avg_x = sum(x_positions) / len(x_positions)
                avg_y = sum(y_positions) / len(y_positions)
                cv2.circle(frame, (int(avg_x), int(avg_y)), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (int(avg_x - 5), int(avg_y) - 5), (int(avg_x) + 5, int(avg_y) + 5), (0, 128, 255), -1)

            if len(results) > 0:
                pose = estimator.estimate(results[0])
                print(pose)

                corners = results[0].getCorners((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

                corners_points = [
                    (int(corners[0]), int(corners[1])),  # (x1, y1)
                    (int(corners[2]), int(corners[3])),  # (x2, y2)
                    (int(corners[4]), int(corners[5])),  # (x3, y3)
                    (int(corners[6]), int(corners[7]))   # (x4, y4)
                ]

                for i in range(4):
                    # print(corners_points[i])
                    start_point = tuple(corners_points[i])
                    end_point = tuple(corners_points[(i + 1) % 4])
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

                # Draw the center of the tag
                center = tuple(np.mean(corners_points, axis=0, dtype=int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                if circles is not None:
                    x = int(sum(x_positions) / len(x_positions))
                    y = int(sum(y_positions) / len(y_positions))
                    # x, y, _ = circles[0]
                    # for (x, y, r) in circles:
                    distance = calculate_distance(center, (x, y))

                    x_tag, y_tag, z_tag = pose.translation()
                    roll = pose.rotation().x
                    pitch = pose.rotation().y
                    yaw = pose.rotation().z

                    r = R.from_euler('xyz', [roll, pitch, yaw])  # Assuming 'xyz' order; confirm with documentation
                    R_tag = r.as_matrix()

                    n = R_tag[:, 2]

                    p0 = np.array([x_tag, y_tag, z_tag])
                    
                    r = np.array([
                        (x - mtx[0, 2]) / mtx[0, 0],
                        (y - mtx[1, 2]) / mtx[1, 1],
                        1
                    ])

                    p_camera = np.array([0, 0, 0])

                    numerator = np.dot(n, (p0 - p_camera))
                    denominator = np.dot(n, r)
                    t = numerator / denominator

                    p_item = p_camera + t * r

                    distance = np.linalg.norm(p_item - np.array([x_tag, y_tag, z_tag]))
                    midpoint = ((center[0] + x) // 2, (center[1] + y) // 2)
                    cv2.putText(frame, f"Dist: {100*distance:.3f} cm", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                    cv2.line(frame, center, (x, y), (255, 0, 0), thickness=2)

            cv2.imshow('Video Feed', frame)

            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
