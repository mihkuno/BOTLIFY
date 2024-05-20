# object detection
import os
import cv2
from datetime import datetime, timedelta
import numpy as np
import pandas as pd
import supervision as sv
from ultralytics import YOLO


# distance detection
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

SONIC_VCC_PIN  = 21
SONIC_TRIG_PIN = 20
SONIC_ECHO_PIN = 16

GPIO.setup(SONIC_VCC_PIN, GPIO.OUT)
GPIO.setup(SONIC_TRIG_PIN, GPIO.OUT)
GPIO.setup(SONIC_ECHO_PIN, GPIO.IN) 
GPIO.output(SONIC_VCC_PIN, True)
GPIO.output(SONIC_TRIG_PIN, True) # bug fix

time.sleep(0.5)

# servo motor controls
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep


SERVO_PIN = 12
factory = PiGPIOFactory()
servo = Servo(SERVO_PIN, pin_factory=factory)



def setServoPosition(position):

    global SERVO_PIN    
    global servo
    global factory
    
    if position == "min":
        servo.min()
    elif position == "mid":
        servo.mid()
    elif position == "max":
        servo.max()
    else:
        print("Invalid position")
        return



def getDistance():
    global SONIC_VCC_PIN
    global SONIC_TRIG_PIN
    global SONIC_ECHO_PIN
    
    GPIO.output(SONIC_TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(SONIC_TRIG_PIN, False)

    while GPIO.input(SONIC_ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(SONIC_ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance




def getDetection():
    
    # config
    weights_path = 'yolov8m.pt'
    camera_index = 0

    # Load the model
    model = YOLO(weights_path) 

    # Create a VideoCapture object
    cam = cv2.VideoCapture(camera_index)

    
    # Check if the camera is opened successfully
    if not cam.isOpened():
        print("Error: Could not open camera.")
        exit()
    
    # Capture frame-by-frame
    ret, frame = cam.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Failed to grab frame")
        return
    
    results = model.predict(frame, conf=0.35, verbose=False)

    class_names = []

    if len(results[0]) > 0:
        detections = results[0]

        # put 'results' to annotate all detections instead
        detections = sv.Detections.from_ultralytics(detections) 
                            
        class_names = detections.data['class_name']
        
    return class_names




try:    
    
    print('Start Listening...')
    
    while True:    
    
        print('Set servo position to mid...')    
        setServoPosition('mid');        
        
        print('Getting distance...')
        detection_threshold = 9 # centimeters
        distance = getDistance()
        print('Distance: ', distance)
        
        sleep(0.1)
        
        if distance < detection_threshold:            
            print('Set servo position to mid...')    
            setServoPosition('mid');        
        
            print('Obstruction detected... wait for 1.5 second before checking for detections...')
            sleep(1.5)
            print('Getting detection...')
            
            detections = getDetection()
            
            print('Detections: ', detections)
            
            if 'bottle' in detections or 'wine glass' in detections or 'vase' in detections or 'toilet' in detections:
                print('Bottle detected...')
                setServoPosition('min');
            else: 
                print('No bottle detected...')
                setServoPosition('max');
            
            print('Wait 0.5 seconds...')
            sleep(0.5)    
            print('Looping...')
        
            
except Exception as e:
    print("Stopped by user, cleaning up...", e)
    GPIO.cleanup()
