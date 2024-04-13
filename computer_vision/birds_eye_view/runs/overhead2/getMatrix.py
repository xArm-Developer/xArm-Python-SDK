import cv2
import numpy as np

def getMatrix():
    cap = cv2.VideoCapture(1)
    while True:
        
        ret, image = cap.read()
        # Convert the image to grayscale
        if not ret:
            print('Frame failed')
            continue
        
        
        cv2.imshow('Original Frame', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Perform edge detection using the Canny algorithm
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter out contours based on area to find the contour of the paper
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
        
        #Draw contour on original frame and perform perspective transformation
        matrix = []
        if max_contour is not None:
            # Calculate the perspective transformation matrix
            epsilon = 0.02 * cv2.arcLength(max_contour, True)
            approx = cv2.approxPolyDP(max_contour, epsilon, True)
            
            # Check if the contour has 4 points
            if len(approx) == 4:
                paper_contour = approx.reshape(4, 2)
                paper_width = 8.27  # A4 paper width in inches
                paper_height = 11.69  # A4 paper height in inches
                
                # Define dimensions of the output image with margin
                margin = 200  # Adjust this value as needed
                output_width = int(paper_width * 100) + 2 * margin
                output_height = int(paper_height * 100) + 2 * margin
                
                # Define desired dimensions of the paper in the output image
                desired_dimensions = np.array([[margin, margin], [output_width - margin, margin],
                                                [output_width - margin, output_height - margin], [margin, output_height - margin]], dtype=np.float32)
                matrix = cv2.getPerspectiveTransform(paper_contour.astype(np.float32), desired_dimensions)
                
                # Apply the perspective transformation to the image
                warped = cv2.warpPerspective(image, matrix, (output_width, output_height))
                cv2.imshow('Warped Image', warped)

                pressed = cv2.waitKey(0)
                if pressed == ord('a'):
                    break
                else:
                    cv2.destroyWindow('Warped Image')
                    continue

    cap.release()
    cv2.destroyAllWindows()
    return matrix
