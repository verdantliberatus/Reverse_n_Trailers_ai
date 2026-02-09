import cv2
import os
import math
import numpy as np
import threading


# Open the default webcam ((1)usually at index 0)
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv2.CAP_PROP_FPS, 30)
fps = int(cap.get(5))
print("fps:", fps)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Resolution set to: {width}x{height}")

scale_factor = 1

# Check if the webcam opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
else:
    print("Webcam opened successfully.")

# Loop to continuously get frames from the webcam
while True:
    # Capture each frame
    ret, frame = cap.read()

    # If the frame was captured successfully
    if ret:
        # Display the frame in a window named 'Webcam'
        cv2.imshow('Original', frame)
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Error: Could not retrieve frame.")
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()