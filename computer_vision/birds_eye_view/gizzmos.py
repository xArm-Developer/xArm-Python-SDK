from xarm.wrapper import XArmAPI

'''
    Gizzmos are external hardware elements that assist the robotic arm
    complete the PCR challenge! This list includes ECLAIR's specially
    curated gripper, a pressure sensor, and the Atalanta Module which
    uses OCR to adjust the volume on the pipettes to the desired amount
    for the reactant
'''

class CustomGripper:

    def __init__(self, arm: XArmAPI) -> None:
        self.arm = arm
        pass

    def close_gripper(self) -> str:
        return None, None

    def open_gripper(self) -> str:
        return None, None

    def fill_pipette(self) -> str:
        return None, None

    def empty_pipette(self) -> str:
        return None, None

    def remove_pipette_tip(self) -> str:
        return None, None

class PressureSensor:

    def __init__(self):
        pass

class AtalantaModule:

    def __init__(self):
        pass

    def adjust_volume(self, volume: float) -> str:

        # Communicate with arduino/RPI externally somehow
        # TCP Network socket maybe?
        return None, None

    def check_connection(self) -> bool:
        return True
    

import cv2 as cv
import numpy as np
class TestTubeCV:
    def __init__(self):
        self.videoCapture = cv.VideoCapture(0, cv.CAP_DSHOW)
        self.prevCircle = None
        self.dist = lambda x1, y1, x2, y2: (x1 - x2) ** 2 + (y1 - y2) ** 2
        self.debug = False

    def predict(self, frame):
        """
        Interprets a single frame given to it by the video feed.
        Preprocesses the image by first converting it to greyscale, performing a median blur, and then utilizing a Hough Transform to generate circles.
        Of the given Hough circles, will filter out any circles that overlap with already predicted circles.
        Will get x- and y- corrdinate of each predicted circle's center.

        DISCLAIMER: As of right now, Hough Transform, RANSAC, or some mixture of the two doesn't work the best.

        Args: frame (cv.image): the input image to interpret from and predict on.
        Returns: pred (list(int, int)): a list of tuples consisting of the x- and y- coordinates of the predicted circles.
        """
        pred = []

        # blur image for canny detection (Hough Transform)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 13)
        
        # docstring of HoughCircles: HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1.0, minDist=10, 
                                param1 = 60,
                                param2 = 75,
                                minRadius = 1, maxRadius = 50)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            chosen = None
            for i in circles[0, :]:
                if chosen is None:
                    chosen = i
                if self.prevCircle is not None:
                    if self.dist(chosen[0], chosen[1], self.prevCircle[0], self.prevCircle[1]) <= self.dist(i[0], i[1], self.prevCircle[0], self.prevCircle[1]):
                        chosen = i
                cv.circle(frame, (i[0], i[1]), 1, (0, 100, 100), 3)
                cv.circle(frame, (i[0], i[1]), i[2], (255, 0 , 255), 3)
            
                self.prevCircle = chosen
                pred.append((chosen[0], chosen[1]))
        
        
        cv.imshow("circles", frame)
        cv.imshow("grayscale", gray)
        return pred

    def getCircles(self):
        """
        Reads in a frame from a birds' eye view video feed, predicts circles, and prints the x- and y- coordinates of the center.
        Args: None
        Returns: None
        """
        while True:
            ret, frame = self.videoCapture.read()
            if not ret:
                break

            predicted = self.predict(frame)
            if predicted != []:
                print(predicted)

            if cv.waitKey(1) == ord('q'):
                break
        self.videoCapture.release()
        cv.destroyAllWindows()

