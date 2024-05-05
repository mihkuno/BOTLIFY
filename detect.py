import os
import cv2
import time
from datetime import datetime, timedelta
import numpy as np
import pandas as pd
import supervision as sv
from ultralytics import YOLO


# config
weights_path = 'yolov8n.pt'
camera_index = 0

# Load the model
model = YOLO(weights_path) 

# Create a VideoCapture object
cam = cv2.VideoCapture(camera_index)

# Check if the camera is opened successfully
if not cam.isOpened():
    print("Error: Could not open camera.")
    exit()

# Capture loop
while True:
  
    # Capture frame-by-frame
    ret, frame = cam.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Failed to grab frame")
        break
    
    results = model.predict(frame, conf=0.7, verbose=False)
    
    if len(results[0]) > 0:
        detections = results[0]

        # put 'results' to annotate all detections instead
        detections = sv.Detections.from_ultralytics(detections) 
        
        
        # Assuming detections.data is {'class_name': array(['person'], dtype='<U6')}
        class_names = detections.data['class_name']

        print(class_names)
        
        
        labels = [
            model.model.names[class_id]
            for class_id
            in detections.class_id
        ]
        

        frame = sv.BoundingBoxAnnotator().annotate(scene=frame, detections=detections)
        frame = sv.LabelAnnotator().annotate(scene=frame, detections=detections, labels=labels)
        

        # Extract the first roi detection
        x_min, y_min, x_max, y_max = detections.xyxy[0]
        detect_region = frame[int(y_min):int(y_max), int(x_min):int(x_max)];


    cv2.imshow('annotations', frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close all windows
cam.release()
cv2.destroyAllWindows()
