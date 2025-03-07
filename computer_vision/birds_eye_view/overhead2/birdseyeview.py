from calibration import calibrate
import cv2
import numpy as np
import robotpy_apriltag
from scipy.spatial.transform import Rotation as R
from itertools import combinations
import math
from collections import deque

def channel_processing(channel):
    # Adaptive thresholding, ot used right now
    # TODO: Add this to frame processing to boost accuracy
    channel = cv2.adaptiveThreshold(channel, 255, 
                                     adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C, 
                                     thresholdType=cv2.THRESH_BINARY, 
                                     blockSize=55, 
                                     C=7)
    # Morphological operations to clean up noise
    channel = cv2.dilate(channel, None, iterations=1)
    channel = cv2.erode(channel, None, iterations=1)
    return channel

def calculate_angle(P1, P2, P3, P4):
    # Step 1: Calculate the vectors
    A = np.array([P2[0] - P1[0], P2[1] - P1[1]])  # Vector from P1 to P2
    B = np.array([P4[0] - P3[0], P4[1] - P3[1]])  # Vector from P3 to P4

    # Step 2: Calculate the dot product and the magnitudes of the vectors
    dot_product = np.dot(A, B)
    mag_A = np.linalg.norm(A)
    mag_B = np.linalg.norm(B)

    # Step 5: Calculate the angle in radians, then convert to degrees
    angle_rad = np.arccos(dot_product / (mag_A * mag_B))
    angle_deg = np.degrees(angle_rad)

    return angle_deg

def inter_centre_distance(x1, y1, x2, y2):
    """Calculate the Euclidean distance between two circle centers."""
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def colliding_circles(circles):
    for index1, circle1 in enumerate(circles):
        for circle2 in circles[index1 + 1:]:
            x1, y1, radius1 = circle1[:3]
            x2, y2, radius2 = circle2[:3]
            # Check for collision or containment
            if inter_centre_distance(x1, y1, x2, y2) < radius1 + radius2:
                return True
    return False

def filter_overlapping_circles(circles):
    """
    Remove overlapping circles, keeping the earlier one in the array.
    Args:
        circles: np.ndarray of circles, each represented as [x, y, radius].
    Returns:
        np.ndarray: Filtered array of non-overlapping circles.
    """
    filtered_circles = []
    
    for index, circle1 in enumerate(circles):
        x1, y1, radius1 = circle1[:3]
        is_overlapping = False
        
        # Check against already-added circles
        for circle2 in filtered_circles:
            x2, y2, radius2 = circle2[:3]
            if inter_centre_distance(x1, y1, x2, y2) < radius1 + radius2:
                is_overlapping = True
                break
        
        # Add the circle if it's not overlapping
        if not is_overlapping:
            filtered_circles.append(circle1)
    
    return np.array(filtered_circles, dtype=np.int32)

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    if point1 and point2:
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def initialize_camera() -> cv2.VideoCapture:
    """Initialize the default webcam and return the capture object."""
    cap: cv2.VideoCapture = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera not found or not accessible")
        exit()
    return cap

def initialize_detector_and_estimator(mtx, tagFamily: str = "tag25h9"):
    """
    Initializes the AprilTag detector and pose estimator using the provided camera.

    This function calibrates the camera, sets up an AprilTag pose estimator with the 
    appropriate intrinsic parameters, and initializes an AprilTag detector with the 
    specified tag family.

    Parameters:
        cap (cv2.VideoCapture): OpenCV video capture object for the camera.
        tagFamily (str, optional): The tag family to detect (default: "tag25h9").

    Returns:
        Tuple[robotpy_apriltag.AprilTagDetector, robotpy_apriltag.AprilTagPoseEstimator]: 
        The initialized AprilTag detector and pose estimator.
    """
    
    # Configure the AprilTag pose estimator with camera intrinsic parameters
    config = robotpy_apriltag.AprilTagPoseEstimator.Config(
        tagSize=0.1,  # Size of the AprilTag in meters
        fx=mtx[0, 0],  # Camera's focal length in the x-direction (pixels)
        fy=mtx[1, 1],  # Camera's focal length in the y-direction (pixels)
        cx=mtx[0, 2],  # Camera's principal point X-coordinate (pixels)
        cy=mtx[1, 2]   # Camera's principal point Y-coordinate (pixels) [Fixed typo]
    )

    # Initialize the AprilTag detector
    detector = robotpy_apriltag.AprilTagDetector()
    
    # Add the specified tag family to the detector
    if not detector.addFamily(tagFamily):
        raise RuntimeError(f"Failed to add AprilTag family: {tagFamily}")

    # Create an AprilTag pose estimator with the configured parameters
    estimator = robotpy_apriltag.AprilTagPoseEstimator(config)

    return detector, estimator, mtx

def process_image(frame, detector):
    """Convert frame to grayscale and apply Gaussian blur for edge detection."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 150)
    results = detector.detect(gray)
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
    return gray, blurred, edges, results, circles

def process_circles(circles, num_circles, x_positions, y_positions, r_positions, frame):
    if circles is not None:
        # Convert detected circles to integer values
        circles = np.round(circles[0, :]).astype("int")

        # Remove overlapping circles
        circles = filter_overlapping_circles(circles)

        # Keep only the specified number of circles and sort them by x-coordinate
        circles = circles[:num_circles]
        circles = sorted(circles, key=lambda x: x[0])

        # Store detected circle positions
        for index, (x, y, r) in enumerate(circles):
            if index >= num_circles:
                break
            x_positions[index].append(x)
            y_positions[index].append(y)
            r_positions[index].append(r)

        # Define colors for visualizing detected circles
        colors = [(255, 0, 0), (0, 255, 0)]  # Adjust if tracking more circles
        curr_averages = []

        for i in range(num_circles):
            if len(x_positions[i]) > 0:
                # Compute average position and radius over the stored history
                x_avg = int(sum(x_positions[i]) / len(x_positions[i]))
                y_avg = int(sum(y_positions[i]) / len(y_positions[i]))
                r_avg = int(sum(r_positions[i]) / len(r_positions[i]))

                # Draw the detected circle and its center
                cv2.circle(frame, (x_avg, y_avg), r_avg, colors[i], 4)
                cv2.rectangle(frame, (x_avg - 5, y_avg - 5), (x_avg + 5, y_avg + 5), (0, 128, 255), -1)

                curr_averages.append((x_avg, y_avg, r_avg))

        return curr_averages
    return []

def process_apriltag(apriltags, estimator, frame):
    """
    Processes AprilTags detected in the frame, estimates pose, and visualizes the tag.

    Args:
        apriltags (list): List of detected AprilTags.
        estimator (object): AprilTag pose estimator.
        frame (numpy array): Frame on which to draw the tag visualization.

    Returns:
        tuple: Center of the detected AprilTag (or None if no tags detected).
    """
    center = None
    pose = None
    if len(apriltags) > 0:
        pose = estimator.estimate(apriltags[0])
        print(pose)

        # Get the AprilTag corner coordinates
        corners = apriltags[0].getCorners((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        corners_points = [
            (int(corners[i]), int(corners[i + 1])) for i in range(0, 8, 2)
        ]

        # Draw the detected corners
        for point in corners_points:
            cv2.circle(frame, point, 5, (0, 255, 0), -1)

        # Draw lines between corners
        for i in range(4):
            cv2.line(frame, corners_points[i], corners_points[(i + 1) % 4], (0, 255, 0), 2)

        # Compute and draw the center of the AprilTag
        center = tuple(np.mean(corners_points, axis=0, dtype=int))
        cv2.circle(frame, center, 5, (0, 0, 255), -1)

    return center, pose

def compute_real_world_coordinates(curr_averages, center, pose, mtx, frame):
    """
    Computes real-world coordinates of detected circles based on AprilTag pose estimation.

    Args:
        curr_averages (list): List of detected circle centers and radii [(x, y, r), ...].
        center (tuple): The detected center of the AprilTag (x, y).
        pose (object): AprilTag pose estimation object.
        mtx (numpy array): Camera matrix for intrinsic parameters.
        frame (numpy array): Frame on which to overlay distance annotations.

    Returns:
        list: Real-world coordinates of detected circles.
    """
    irl_coords = []
    
    for (x, y, _) in curr_averages:
        # Get the translation and rotation of the AprilTag
        x_tag, y_tag, z_tag = pose.translation()
        roll, pitch, yaw = pose.rotation().x, pose.rotation().y, pose.rotation().z

        # Convert rotation angles into a rotation matrix
        r = R.from_euler('xyz', [roll, pitch, yaw])
        R_tag = r.as_matrix()
        n = R_tag[:, 2]

        # Compute real-world coordinates of the detected circles
        r_dir = np.array([(x - mtx[0, 2]) / mtx[0, 0], (y - mtx[1, 2]) / mtx[1, 1], 1])
        r_dir /= np.linalg.norm(r_dir)

        p_camera = np.array([0, 0, 0])
        t = np.dot(n, (np.array([x_tag, y_tag, z_tag]) - p_camera)) / np.dot(n, r_dir)
        p_item = p_camera + t * r_dir
        irl_coords.append(p_item)

        distance = np.linalg.norm(p_item - np.array([x_tag, y_tag, z_tag]))

        # Display the calculated distance
        midpoint = ((center[0] + x) // 2, (center[1] + y) // 2)
        cv2.putText(frame, f"Dist: {100*distance:.3f} cm", midpoint, 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.line(frame, center, (x, y), (255, 0, 0), thickness=2)

    # Compute distances between detected circles
    for (i1, val1), (i2, val2) in combinations(enumerate(irl_coords), 2):
        distance = np.linalg.norm(np.array(val1) - np.array(val2))
        cv2.putText(frame, f"Dist: {100*distance:.3f} cm", 
                    (curr_averages[i1][0], curr_averages[i1][1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    return irl_coords

def main():
    num_circles = 1   # Number of circles being tracked
    max_history = 10  # Maximum number of past detections to store
    cap = initialize_camera()
    _, mtx, _, _, _ = calibrate(cap)
    detector, estimator, mtx = initialize_detector_and_estimator(mtx)

    # Lists to store detected positions of multiple circles (e.g., beakers)
    x_positions = [deque(maxlen=max_history) for _ in range(num_circles)]
    y_positions = [deque(maxlen=max_history) for _ in range(num_circles)]
    r_positions = [deque(maxlen=max_history) for _ in range(num_circles)]

    try:
        while True:
            # Read a frame from the webcam
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            _, _, _, apriltags, circles = process_image(frame, detector)

            curr_averages = process_circles(circles, num_circles, x_positions, y_positions, r_positions, frame)

            # AprilTag processing
            center, pose = process_apriltag(apriltags, estimator, frame)

            # Calculate distances from the tag to detected circles
            if center and curr_averages:
                irl_coords = compute_real_world_coordinates(curr_averages, center, pose, mtx, frame)

            cv2.imshow('Video Feed', frame)

            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
