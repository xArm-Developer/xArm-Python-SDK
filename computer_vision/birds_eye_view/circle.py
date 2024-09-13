import cv2
import numpy as np

def findBeaker():
    cap = cv2.VideoCapture(1)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame. Exiting ...")
            break

        # Convert the frame to grayscale for edge detection.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection.
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect edges using Canny edge detector.
        edges = cv2.Canny(blurred, 50, 150)

        adjusted_frame = cv2.convertScaleAbs(edges, alpha=3.5, beta=0)

        # Apply Hough Circle Transform to find circles in the frame.
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

        if circles is not None:
            # Convert the (x, y) coordinates and radius of the circles to integers.
            circles = np.round(circles[0, :]).astype("int")

            # Draw circles on the frame.
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                
                # Display the coordinates of the detected circle.
                print("Circle detected at (x={}, y={})".format(x, y))

        # Display the resulting frame with detected circles.
        cv2.imshow('Circles Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture and close all OpenCV windows.
    cap.release()
    cv2.destroyAllWindows()

findBeaker()