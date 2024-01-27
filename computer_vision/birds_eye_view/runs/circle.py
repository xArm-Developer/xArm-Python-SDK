import cv2
import numpy as np

# Create a VideoCapture object to capture video from a file or camera.
cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)  # Replace 'your_video.mp4' with the video file name or 0 for the default camera.

while True:
    ret, frame = cap.read()
    
    if not ret:
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
        param1 = 50,
        param2 = 30,
        minRadius= 20,
        maxRadius=100
    )

    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers.
        circles = np.round(circles[0, :]).astype("int")

        # Draw circles on the frame.
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

    # Display the resulting frame with detected circles.
    cv2.imshow('Circles Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all OpenCV windows.
cap.release()
cv2.destroyAllWindows()

# cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)

# while True:
#     _,img = cap.read()
#     img2 = img.copy()
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     gray = cv2.GaussianBlur(gray, (9, 9), 2)
#     #parameters---(img,circle_method,dp,mindist,parm1,parm2[p1<p2],)
#     circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 10,
#                               param1=50, param2=30, minRadius=0,
#                               maxRadius=0)
#     if circles is not None:
#         data = np.uint16(np.around(circles))
#         for (x, y ,r) in data[0, :]:
#             cv2.circle(img2, (x, y), r, (50, 10, 50), 3) #outer circle
#             cv2.circle(img2, (x, y), 2, (0, 255, 100), -1) #center
#     cv2.imshow("res",img2)
#     if cv2.waitKey(25) & 0xFF == ord("q"):
#         break
    

# cap.release()
# cv2.destroyAllWindows()