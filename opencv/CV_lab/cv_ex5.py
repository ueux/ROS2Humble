import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if cap.isOpened():
    print("Webcam opened")
    
    while cv2.waitKey(3) != ord('q'):
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 100, 200)

        # Apply Probabilistic Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)

        # Draw detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]  # Extract line coordinates
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Show the processed frame
        cv2.imshow("webCam", frame)
        
else:
    print("Opening webcam failed")

cap.release()
cv2.destroyAllWindows()
