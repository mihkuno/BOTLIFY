import os
import cv2
import time
from datetime import datetime, timedelta
import numpy as np
import pandas as pd
import supervision as sv
from ultralytics import YOLO

camera_index = 2  # for built-in camera, 1 or 2 for external camera
weights_path = 'yolov8n.pt'

# Load the model
model = YOLO(weights_path)

# Create a VideoCapture object
cam = cv2.VideoCapture(0)

# Check if the camera is opened successfully
if not cam.isOpened():
    print("Error: Could not open camera.")
    exit()

# Get the default screen resolution
screen_width = 1920  # Update with your screen's width
screen_height = 1080  # Update with your screen's height

# Create a fullscreen window
cv2.namedWindow("detections", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("detections", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# Capture loop
while True:
    print('getting frame')
    # Capture frame-by-frame
    ret, frame = cam.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Failed to grab frame")
        break

    results = model.predict(frame, conf=0.7, verbose=False)

    # put 'results' to annotate all detections instead
    detections = sv.Detections.from_ultralytics(results[0])

    labels = [
        model.model.names[class_id]
        for class_id
        in detections.class_id
    ]

    frame = sv.BoundingBoxAnnotator().annotate(scene=frame, detections=detections)
    frame = sv.LabelAnnotator().annotate(scene=frame, detections=detections, labels=labels)

    cv2.imshow('detections', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close all windows
cam.release()
cv2.destroyAllWindows()
