import cv2 as cv
import time
import os
import glob
import numpy as np
import shutil

def calibrate(cap):
    # Number of photos to take
    num_photos = 15
    chess_points_vertical = 9
    chess_points_horizontal = 6
    photo_directory = 'calibration_photos'  # Change this to your desired folder path

    photos_found = True
    # photos_found = False
    # if os.path.exists(photo_directory):
    #     existing_photos = [f for f in os.listdir(photo_directory) if f.endswith('.jpg')]
    #     if existing_photos:
    #         print(f"Warning: The folder '{photo_directory}' already contains {len(existing_photos)} photos.")
    #         response = input("Do you want to overwrite the existing photos? (y/n): ").lower()
    #         if response != 'y':
    #             print("Continuing without overwriting.")
    #             photos_found = True
    #             # exit()  # Quit the script if the user doesn't want to overwrite
    # else:
    #     # Create the directory if it doesn't exist
    #     os.makedirs(photo_directory)

    if not photos_found:
        # Check if the camera is opened correctly
        if not cap.isOpened():
            print("Error: Could not open camera.")
            exit()

        # Loop to take 15 photos
        for i in range(num_photos):
            print(f"Press the spacebar to take photo {i+1}...")

            # Show the live camera feed
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Error: Could not read frame.")
                    break

                # Display the camera feed
                cv.imshow("Camera Feed", frame)

                # Wait for key press
                key = cv.waitKey(1)  # 1 ms delay to update the feed

                # Check if the spacebar is pressed
                if key == ord(' '):  # Spacebar key
                    # Save the photo to the specified directory
                    photo_filename = os.path.join(photo_directory, f"photo_{i+1}.jpg")
                    cv.imwrite(photo_filename, frame)
                    print(f"Photo {i+1} saved as {photo_filename}")
                    break  # Exit the loop after capturing the photo

                # If the user presses 'q', break the loop and exit
                if key == ord('q'):
                    print("Exiting...")
                    break

            # Wait a little before taking the next photo
            time.sleep(0.5)  # Optional: Wait for half a second before taking the next photo

            # If 'q' was pressed to quit, stop the loop entirely
            if key == ord('q'):
                break

        # Release the camera and close all OpenCV windows
        # cap.release()
        cv.destroyAllWindows()

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chess_points_horizontal*chess_points_vertical,3), np.float32)
    objp[:,:2] = np.mgrid[0:chess_points_vertical,0:chess_points_horizontal].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(f'{photo_directory}/*.jpg')
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (chess_points_vertical,chess_points_horizontal), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            # cv.drawChessboardCorners(img, (chess_points_vertical,chess_points_horizontal), corners2, ret)
            # cv.imshow('img', img)
            # cv.waitKey(100)
    # cv.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # print(f'Error: {ret}')
    # print(ret, mtx, dist, rvecs, tvecs)

    # response = input(f"Do you want to keep the calibration photos in '{photo_directory}'? (y/n): ").lower()
    # if response != 'y':
    #     # Delete the directory and its contents
    #     print(f"Deleting '{photo_directory}' and its contents...")
    #     shutil.rmtree(photo_directory)
    #     print(f"'{photo_directory}' and its contents have been deleted.")
    # else:
    #     print(f"'{photo_directory}' has been kept.")

    return ret, mtx, dist, rvecs, tvecs