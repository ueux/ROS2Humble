import numpy as np
import cv2
import math

drawMatchingPoints = True  # Show matching points
threshold = 0.08  # Threshold for detection
numPoints = 100  # Number of best matching points

# Load target image
img1 = cv2.imread('target.bmp')
if img1 is None:
    print("Error: Target image not found!")
    exit()

h1, w1, _ = img1.shape
corners1 = np.float32([[0, 0], [0, h1 - 1], [w1 - 1, h1 - 1], [w1 - 1, 0]]).reshape(-1, 1, 2)

# ORB Detector
orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1, None)

cap = cv2.VideoCapture(0)  # Open webcam

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from webcam.")
        break

    h2, w2, _ = frame.shape

    # Find keypoints and descriptors
    kp2, des2 = orb.detectAndCompute(frame, None)

    if des2 is None or len(kp2) == 0:
        print("Warning: No keypoints detected in webcam frame.")
        continue  # Skip this frame

    # BFMatcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    if len(matches) < 4:
        print("Warning: Not enough matches found!")
        continue  # Skip if not enough matches

    matches = sorted(matches, key=lambda x: x.distance)
    good = matches[:numPoints]  # Select top matches

    # Stack images side by side
    # Resize the frame to match img1's height (h1)
    frame_resized = cv2.resize(frame, (w2, h1))

    # Stack the images horizontally
    imgCom = np.hstack((img1, frame_resized))

    # Match keypoints
    pts1 = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    if drawMatchingPoints:
        for i in range(len(good)):
            (x1, y1) = pts1[i][0]
            (x2, y2) = pts2[i][0]
            cv2.circle(imgCom, (int(x1), int(y1)), 4, (255, 0, 0), 1)
            cv2.circle(imgCom, (int(x2) + w1, int(y2)), 4, (255, 0, 0), 1)
            color = (np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255))
            cv2.line(imgCom, (int(x1), int(y1)), (int(x2) + w1, int(y2)), color, 1)

    # Find homography
    M, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC)
    if M is not None:
        corners2 = cv2.perspectiveTransform(corners1, M)

        # Compute angle to check transformation correctness
        vect1 = (corners2[3][0][0] - corners2[0][0][0], corners2[3][0][1] - corners2[0][0][1])
        vect2 = (corners2[1][0][0] - corners2[0][0][0], corners2[1][0][1] - corners2[0][0][1])
        cosTheta = np.abs(vect1[0] * vect2[0] + vect1[1] * vect2[1]) / (
                math.sqrt(vect1[0] ** 2 + vect1[1] ** 2) * math.sqrt(vect2[0] ** 2 + vect2[1] ** 2))

        if cosTheta < threshold:
            for i in range(4):
                corners2[i][0][0] += w1  # Shift to match stacked image
            cv2.polylines(imgCom, [np.int32(corners2)], True, (255, 255, 255), 3)

    # Show results
    cv2.imshow('Matched Features', imgCom)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
