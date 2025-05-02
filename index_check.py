import cv2

for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Webcam found at index {i}")
        cap.release()
    else:
        print(f"No webcam at index {i}")
