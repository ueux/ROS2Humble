import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Force Video4Linux2 (Linux users)


if not cap.isOpened():
    print("Error: Unable to access webcam.")
    exit()

# Load Haarcascade models
face_cascade = cv2.CascadeClassifier('/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml')

eyes_cascade = cv2.CascadeClassifier( '/usr/share/opencv4/haarcascades/haarcascade_eye.xml')

if face_cascade.empty() or eyes_cascade.empty():
    print("Error: Could not load Haarcascade XML files.")
    cap.release()
    exit()

print("Webcam opened")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from webcam.")
        break

    # Convert to grayscale for better performance
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(50, 50))

    for (x, y, w, h) in faces:
        # Draw a rectangle around the face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Region of Interest (ROI) for eyes detection
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = frame[y:y + h, x:x + w]

        # Detect eyes in the face region
        eyes = eyes_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=10, minSize=(20, 20))

        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Face & Eye Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
