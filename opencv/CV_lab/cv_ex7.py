import cv2
import numpy as np

cap = cv2.VideoCapture("test.mp4")

if cap.isOpened():
    print("Video opened")

    while cv2.waitKey(3) != ord('q'):
        ret, frame = cap.read()
        if not ret:
            print("End of video or error reading frame")
            break

        # Convert color space from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for blue color
        lowBound = np.array([110, 50, 50])
        upBound = np.array([130, 255, 255])

        # Filtering by color
        mask = cv2.inRange(hsv, lowBound, upBound)

        # Image dilation to improve detection
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process up to 2 largest contours
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01 * peri, True)

            if len(approx) == 4:  # If it is a quadrilateral
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), -1)
            elif len(approx) > 10:  # If it is circular
                cv2.drawContours(frame, [approx], 0, (0, 0, 255), -1)

        # Create a combined image for visualization
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # Convert grayscale mask to BGR
        imgCom = np.hstack((mask_colored, frame))  # Stack horizontally

        # Resize the combined image while maintaining aspect ratio
        h, w, _ = imgCom.shape
        scale_factor = 600 / w  # Resize to width 600 pixels
        imgCom = cv2.resize(imgCom, (600, int(h * scale_factor)))

        # Show the processed frame
        cv2.imshow("Object Tracking", imgCom)

else:
    print("Opening video failed")

cap.release()
cv2.destroyAllWindows()
