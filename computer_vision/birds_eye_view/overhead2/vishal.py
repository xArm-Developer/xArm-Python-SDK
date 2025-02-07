from calibration import calibrate
import cv2
import numpy as np
import robotpy_apriltag
from scipy.spatial.transform import Rotation as R
from itertools import combinations
import math

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

    x_positions = []  
    y_positions = []
    r_positions = []
    max_history = 10
    num_circles = 2

    for i in range(0, num_circles):
        x_positions.append([])
        y_positions.append([])
        r_positions.append([])

    # num_circles = input("Number of circles to detect: ")
    
    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame")
                break
            
            # processing for april tag detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            cv2.imshow('Grey Feed', gray)

            results = detector.detect(gray)
            
            # processing for beaker detection
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            cv2.imshow('Blurred Feed', blurred)
            edges = cv2.Canny(blurred, 50, 150)
            cv2.imshow('Edges Feed', edges)
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

            curr_averages = []

            if circles is not None:
                # Convert the (x, y) coordinates and radius of the circles to integers.
                circles = np.round(circles[0, :]).astype("int")

                # print(circles)
                # get rid of overlapping circles
                circles = filter_overlapping_circles(circles)
                # print(circles)

                # for (x, y, r) in circles:
                #     cv2.circle(frame, (x, y), r, (255, 0, 0), 10)

                # Draw circles on the frame.
                # x, y, r = circles[0]
                circles = circles[:num_circles]
                circles = sorted(circles, key=lambda x: x[0])

                for index, (x, y, r) in enumerate(circles):
                    if index >= num_circles:
                        break
                    x_positions[index].append(x)
                    y_positions[index].append(y)
                    r_positions[index].append(r)
                
                # x_positions.append(x_coords)
                # y_positions.append(y_coords)
                # r_coords.append(r_coords)

                # x_positions.append(x)
                # y_positions.append(y)
                # r_positions.append(r)
                for index, arr in enumerate(x_positions):
                    if len(arr) > max_history:
                        x_positions[index].pop(0)
                        y_positions[index].pop(0)
                        r_positions[index].pop(0)

                # if len(x_positions) > max_history:
                #     x_positions.pop(0)
                #     y_positions.pop(0)
                #     r_positions.pop(0)
                
                colors = [255, 0]
                for i in range(0, num_circles):
                    if len(x_positions[i]) > 0:
                        x_avg = int(sum(x_positions[i]) / len(x_positions[i]))
                        y_avg = int(sum(y_positions[i]) / len(y_positions[i]))
                        r_avg = int(sum(r_positions[i]) / len(r_positions[i]))
                        cv2.circle(frame, (x_avg, y_avg), r_avg, (0, colors[i], 0), 4)
                        cv2.rectangle(frame, (int(x_avg- 5), int(y_avg) - 5), (int(x_avg) + 5, int(y_avg) + 5), (0, 128, 255), -1)
                        curr_averages.append((x_avg, y_avg, r_avg))
                # avg_x = sum(x_positions) / len(x_positions)
                # avg_y = sum(y_positions) / len(y_positions)
                # cv2.circle(frame, (int(avg_x), int(avg_y)), r, (0, 255, 0), 4)
                # cv2.rectangle(frame, (int(avg_x - 5), int(avg_y) - 5), (int(avg_x) + 5, int(avg_y) + 5), (0, 128, 255), -1)

            center = None
            c1 = None
            c2 = None
            corners_points = None
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

                c1 = corners_points[3]
                c2 = corners_points[2]

                corner_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]  # Red, Green, Blue, Yellow

                for i, point in enumerate(corners_points):
                    cv2.circle(frame, point, 5, corner_colors[i], -1)  # D

                for i in range(4):
                    start_point = tuple(corners_points[i])
                    end_point = tuple(corners_points[(i + 1) % 4])
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

                # Draw the center of the tag
                center = tuple(np.mean(corners_points, axis=0, dtype=int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # if circles is not None:
                #     x = int(sum(x_positions) / len(x_positions))
                #     y = int(sum(y_positions) / len(y_positions))
                #     # x, y, _ = circles[0]
                #     # for (x, y, r) in circles:
                #     distance = calculate_distance(center, (x, y))

                #     x_tag, y_tag, z_tag = pose.translation()
                #     roll = pose.rotation().x
                #     pitch = pose.rotation().y
                #     yaw = pose.rotation().z

                #     r = R.from_euler('xyz', [roll, pitch, yaw]) 
                #     R_tag = r.as_matrix()

                #     n = R_tag[:, 2]

                #     p0 = np.array([x_tag, y_tag, z_tag])
                    
                #     r = np.array([
                #         (x - mtx[0, 2]) / mtx[0, 0],
                #         (y - mtx[1, 2]) / mtx[1, 1],
                #         1
                #     ])

                #     p_camera = np.array([0, 0, 0])

                #     numerator = np.dot(n, (p0 - p_camera))
                #     denominator = np.dot(n, r)
                #     t = numerator / denominator

                #     p_item = p_camera + t * r

                #     distance = np.linalg.norm(p_item - np.array([x_tag, y_tag, z_tag]))
                #     midpoint = ((center[0] + x) // 2, (center[1] + y) // 2)
                #     cv2.putText(frame, f"Dist: {100*distance:.3f} cm", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                #     cv2.line(frame, center, (x, y), (255, 0, 0), thickness=2)

            if center:
                irl_coords = []
                for (x, y, _) in curr_averages:
                    distance = calculate_distance(center, (x, y))

                    x_tag, y_tag, z_tag = pose.translation()
                    roll = pose.rotation().x
                    pitch = pose.rotation().y
                    yaw = pose.rotation().z

                    r = R.from_euler('xyz', [roll, pitch, yaw]) 
                    R_tag = r.as_matrix()

                    n = R_tag[:, 2]

                    p0 = np.array([x_tag, y_tag, z_tag])

                    r_dir = np.array([
                        (x - mtx[0, 2]) / mtx[0, 0],
                        (y - mtx[1, 2]) / mtx[1, 1],
                        1
                    ])
                    r_dir /= np.linalg.norm(r_dir)
                    
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

                    # midpoint1 = ((corners_points[1][0] + corners_points[2][0]) // 2, (corners_points[1][1] + corners_points[2][1]) // 2)
                    # midpoint2 = ((corners_points[0][0] + corners_points[3][0]) // 2, (corners_points[0][1] + corners_points[3][1]) // 2)

                    # dx = midpoint2[0] - midpoint1[0]
                    # dy = midpoint2[1] - midpoint1[1]

                    # if dx == 0:  # Vertical line
                    #     start_point = (midpoint1[0], 0)  # Top of the frame
                    #     end_point = (midpoint1[0], frame.shape[0])  # Bottom of the frame
                    # else:
                    #     # Calculate line endpoints based on frame boundaries
                    #     slope = dy / dx
                    #     y1 = int(midpoint1[1] - slope * midpoint1[0])  # y at x = 0
                    #     y2 = int(midpoint1[1] + slope * (frame.shape[1] - midpoint1[0]))  # y at x = frame width

                    #     start_point = (0, y1)
                    #     end_point = (frame.shape[1], y2)

                    # # Draw the line
                    # cv2.line(frame, start_point, end_point, (0, 0, 255), 2)

                    # # Find the perpendicular point
                    # t = ((x - midpoint1[0]) * dx + (y - midpoint1[1]) * dy) / (dx ** 2 + dy ** 2)
                    # perp = (int(midpoint1[0] + t * dx), int(midpoint1[1] + t * dy))
                    # # xp = int(midpoint1[0] + t * dx)
                    # # yp = int(midpoint1[1] + t * dy)

                    # # Draw the perpendicular point
                    # cv2.circle(frame, perp, 5, (0, 255, 0), -1)  # Mark the perpendicular point
                    # cv2.line(frame, (x, y), perp, (255, 255, 0), 1)  # Draw the perpendicular line

                    # angle = calculate_angle(c1, c2, center, (x, y)) 
                    # angle_radians = math.radians(angle)

                    # # Calculate pixel distance of hypotenuse
                    # hd_pixels = math.sqrt((center[0] - x) ** 2 + (center[1] - y) ** 2)

                    # # Calculate the scaling factor
                    # scaling_factor = distance / hd_pixels

                    # # Calculate the horizontal and vertical legs in pixels
                    # xd_pixels = math.sqrt((center[0] - perp[0]) ** 2 + (center[1] - perp[1]) ** 2)
                    # yd_pixels = math.sqrt((x - perp[0]) ** 2 + (y - perp[1]) ** 2)

                    # # Convert the horizontal and vertical leg distances to real-world units
                    # d_horizontal = xd_pixels * scaling_factor
                    # d_vertical = yd_pixels * scaling_factor

                    # # Output the real-world distances
                    # print(f"Horizontal leg distance: {d_horizontal * 100} units")
                    # print(f"Vertical leg distance: {d_vertical* 100} units")

                    irl_coords.append(p_item)
                
                indexed_irl_coords = list(enumerate(irl_coords))

                for (i1, val1), (i2, val2) in combinations(indexed_irl_coords, 2):
                    distance = np.linalg.norm(np.array(val1) - np.array(val2))
                    midpoint = ((curr_averages[i1][0] + curr_averages[i2][0]) // 2, (curr_averages[i1][1] + curr_averages[i2][1]) // 2)
                    cv2.putText(frame, f"Dist: {100*distance:.3f} cm", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    print(curr_averages[i1])
                    cv2.line(frame, (curr_averages[i1][0], curr_averages[i1][1]), (curr_averages[i2][0], curr_averages[i2][1]), (255, 0, 0), thickness=2)

            cv2.imshow('Video Feed', frame)

            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

def get_locations(cap, num_circles, max_history=10):
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

    x_positions = []  
    y_positions = []
    r_positions = []

    for i in range(0, num_circles):
        x_positions.append([])
        y_positions.append([])
        r_positions.append([])
    
    return_value = []

    try:
        for blah in range(0, max_history):
            # return_value = []
            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame")
                break
            
            # processing for april tag detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            cv2.imshow('Grey Feed', gray)

            results = detector.detect(gray)
            
            # processing for beaker detection
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            cv2.imshow('Blurred Feed', blurred)
            edges = cv2.Canny(blurred, 50, 150)
            cv2.imshow('Edges Feed', edges)
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

            curr_averages = []

            if circles is not None:
                # Convert the (x, y) coordinates and radius of the circles to integers.
                circles = np.round(circles[0, :]).astype("int")

                # print(circles)
                # get rid of overlapping circles
                circles = filter_overlapping_circles(circles)
                # print(circles)

                for (x, y, r) in circles:
                    cv2.circle(frame, (x, y), r, (255, 0, 0), 10)

                # Draw circles on the frame.
                # x, y, r = circles[0]
                circles = circles[:num_circles]
                circles = sorted(circles, key=lambda x: x[0])

                for index, (x, y, r) in enumerate(circles):
                    if index >= num_circles:
                        break
                    x_positions[index].append(x)
                    y_positions[index].append(y)
                    r_positions[index].append(r)
                
                # x_positions.append(x_coords)
                # y_positions.append(y_coords)
                # r_coords.append(r_coords)

                # x_positions.append(x)
                # y_positions.append(y)
                # r_positions.append(r)
                for index, arr in enumerate(x_positions):
                    if len(arr) > max_history:
                        x_positions[index].pop(0)
                        y_positions[index].pop(0)
                        r_positions[index].pop(0)

                # if len(x_positions) > max_history:
                #     x_positions.pop(0)
                #     y_positions.pop(0)
                #     r_positions.pop(0)
                
                colors = [255, 0]
                for i in range(0, num_circles):
                    if len(x_positions[i]) > 0:
                        x_avg = int(sum(x_positions[i]) / len(x_positions[i]))
                        y_avg = int(sum(y_positions[i]) / len(y_positions[i]))
                        r_avg = int(sum(r_positions[i]) / len(r_positions[i]))
                        cv2.circle(frame, (x_avg, y_avg), r_avg, (0, colors[i], 0), 4)
                        cv2.rectangle(frame, (int(x_avg- 5), int(y_avg) - 5), (int(x_avg) + 5, int(y_avg) + 5), (0, 128, 255), -1)
                        curr_averages.append((x_avg, y_avg, r_avg))

            center = None
            c1 = None
            c2 = None
            corners_points = None
            if len(results) > 0:
                pose = estimator.estimate(results[0])
                # print(pose)

                corners = results[0].getCorners((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

                corners_points = [
                    (int(corners[0]), int(corners[1])),  # (x1, y1)
                    (int(corners[2]), int(corners[3])),  # (x2, y2)
                    (int(corners[4]), int(corners[5])),  # (x3, y3)
                    (int(corners[6]), int(corners[7]))   # (x4, y4)
                ]

                c1 = corners_points[3]
                c2 = corners_points[2]

                corner_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]  # Red, Green, Blue, Yellow

                for i, point in enumerate(corners_points):
                    cv2.circle(frame, point, 5, corner_colors[i], -1)  # D

                for i in range(4):
                    start_point = tuple(corners_points[i])
                    end_point = tuple(corners_points[(i + 1) % 4])
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

                # Draw the center of the tag
                center = tuple(np.mean(corners_points, axis=0, dtype=int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            if center:
                irl_coords = []
                for (x, y, _) in curr_averages:
                    return_value = []
                    distance = calculate_distance(center, (x, y))

                    x_tag, y_tag, z_tag = pose.translation()
                    roll = pose.rotation().x
                    pitch = pose.rotation().y
                    yaw = pose.rotation().z

                    r = R.from_euler('xyz', [roll, pitch, yaw]) 
                    R_tag = r.as_matrix()

                    n = R_tag[:, 2]

                    p0 = np.array([x_tag, y_tag, z_tag])

                    r_dir = np.array([
                        (x - mtx[0, 2]) / mtx[0, 0],
                        (y - mtx[1, 2]) / mtx[1, 1],
                        1
                    ])
                    r_dir /= np.linalg.norm(r_dir)
                    
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

                    midpoint1 = ((corners_points[1][0] + corners_points[2][0]) // 2, (corners_points[1][1] + corners_points[2][1]) // 2)
                    midpoint2 = ((corners_points[0][0] + corners_points[3][0]) // 2, (corners_points[0][1] + corners_points[3][1]) // 2)

                    dx = midpoint2[0] - midpoint1[0]
                    dy = midpoint2[1] - midpoint1[1]

                    if dx == 0:  # Vertical line
                        start_point = (midpoint1[0], 0)  # Top of the frame
                        end_point = (midpoint1[0], frame.shape[0])  # Bottom of the frame
                    else:
                        # Calculate line endpoints based on frame boundaries
                        slope = dy / dx
                        y1 = int(midpoint1[1] - slope * midpoint1[0])  # y at x = 0
                        y2 = int(midpoint1[1] + slope * (frame.shape[1] - midpoint1[0]))  # y at x = frame width

                        start_point = (0, y1)
                        end_point = (frame.shape[1], y2)

                    # Draw the line
                    cv2.line(frame, start_point, end_point, (0, 0, 255), 2)

                    # Find the perpendicular point
                    t = ((x - midpoint1[0]) * dx + (y - midpoint1[1]) * dy) / (dx ** 2 + dy ** 2)
                    perp = (int(midpoint1[0] + t * dx), int(midpoint1[1] + t * dy))

                    # Draw the perpendicular point
                    cv2.circle(frame, perp, 5, (0, 255, 0), -1)  # Mark the perpendicular point
                    cv2.line(frame, (x, y), perp, (255, 255, 0), 1)  # Draw the perpendicular line

                    angle = calculate_angle(c1, c2, center, (x, y)) 
                    angle_radians = math.radians(angle)
                    
                    # Calculate pixel distance of hypotenuse
                    hd_pixels = math.sqrt((center[0] - x) ** 2 + (center[1] - y) ** 2)

                    # Calculate the scaling factor
                    scaling_factor = distance / hd_pixels

                    # Calculate the horizontal and vertical legs in pixels
                    xd_pixels = math.sqrt((center[0] - perp[0]) ** 2 + (center[1] - perp[1]) ** 2)
                    yd_pixels = math.sqrt((x - perp[0]) ** 2 + (y - perp[1]) ** 2)

                    # Convert the horizontal and vertical leg distances to real-world units
                    d_horizontal = xd_pixels * scaling_factor
                    d_vertical = yd_pixels * scaling_factor

                    # Output the real-world distances
                    # print(f"Horizontal leg distance: {d_horizontal * 100} units")
                    # print(f"Vertical leg distance: {d_vertical* 100} units")
                    return_value.append((d_horizontal, d_vertical))

                    irl_coords.append(p_item)
                
                indexed_irl_coords = list(enumerate(irl_coords))

                for (i1, val1), (i2, val2) in combinations(indexed_irl_coords, 2):
                    distance = np.linalg.norm(np.array(val1) - np.array(val2))
                    midpoint = ((curr_averages[i1][0] + curr_averages[i2][0]) // 2, (curr_averages[i1][1] + curr_averages[i2][1]) // 2)
                    cv2.putText(frame, f"Dist: {100*distance:.3f} cm", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    # print(curr_averages[i1])
                    cv2.line(frame, (curr_averages[i1][0], curr_averages[i1][1]), (curr_averages[i2][0], curr_averages[i2][1]), (255, 0, 0), thickness=2)

            cv2.imshow('Video Feed', frame)

            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # cap.release()
        cv2.destroyAllWindows()
    
    return return_value

if __name__ == "__main__":
    main()
