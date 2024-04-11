import robotpy_apriltag
import cv2
import numpy as np
# from runs.arm_test import moveToSquare

class AprilTags:
    """_summary_
    Class that encompasses all functionality for working with AprilTags
    """
    def __init__(self) -> None:
        self.WIDTH = 640
        self.HEIGH = 480

WIDTH = 640 #Width of image from camera
HEIGHT = 480 #Height of image from camera

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
tag = robotpy_apriltag.AprilTagDetector()
tag.addFamily("tag36h11", 3)

def getScalar():
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.resize(frame, (640, 480))
        warped_img = frame
        l: robotpy_apriltag.AprilTagDetection = tag.detect(frame)
        if l is not None:
            for i in l:
                # print(i.getCenter())
                point = i.getCenter()
                x: int = int(point.x)
                y: int = int(point.y)   

                points = [(0, 0), (0, 0), (0, 0), (0, 0)]
                colors_list = [(0, 0, 255), (0, 255, 255), (255, 0, 0), (255, 255, 255)]
                for j in range(4):
                    points[j] = (int(i.getCorner(j).x), int(i.getCorner(j).y))   
                
                # for j in range(4):
                #     cv2.circle(frame, points[j], 2, (50, 0, 0), thickness=j)
                """"
                A3 ----------- B2
                |             |
                |             |
                C0 ----------- D1
                
                """
                # Calculate the width and height of the april tag to calculate homography
                width_AB = np.sqrt(((points[2][0] - points[3][0])**2) + ((points[2][1] - points[3][1])**2))
                width_CD = np.sqrt(((points[1][0] - points[0][0])**2) + ((points[1][1] - points[0][1])**2))
                width = max([width_AB, width_CD]) #Defines width of April Tag
                
                height_BD = np.sqrt(((points[2][0] - points[1][0])**2) + ((points[2][1] - points[1][1])**2))
                height_AC = np.sqrt(((points[3][0] - points[0][0])**2) + ((points[3][1] - points[0][1])**2))
                height = max([height_BD, height_AC]) #Defines height of april tag
                input_pts = np.float32([points[3], points[0], points[1], points[2]])
                output_pts = np.float32([[0, 0],
                            [0, height - 1],
                            [width - 1, height - 1],
                            [width - 1, 0]])
                M = cv2.getPerspectiveTransform(input_pts, output_pts)
                #print("M: ", M)
                #print(height, width)
                warped_img = cv2.warpPerspective(frame, M, (int(width), int(height)), flags=cv2.INTER_LINEAR)
                
                cv2.line(frame, (x, 0), (x, HEIGHT), (0, 255, 0), thickness=2)
                cv2.line(frame, (0, y), (WIDTH, y), (0, 255, 0), thickness=2)
                
                tag_side_mm = 100
                scalar =  tag_side_mm / width
                #print(scalar)
                cv2.imshow("Camera Feed", frame)
                cv2.imshow("Flat image", warped_img)
                return (scalar, x, y)

        # Display the frame
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Flat image", warped_img)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    


    # Release the capture object and close the window
    cap.release()
    cv2.destroyAllWindows()

getScalar()
