#!/home/botlify/BOTLIFY/venv/bin/python

import os
import time
import subprocess
import sys

# data handler
import json
import display

# object detection
import cv2
from datetime import datetime, timedelta
import numpy as np
import pandas as pd
from ultralytics import YOLO

# Initialize GPIO pins
import RPi.GPIO as GPIO
import time

# Voucher generation
import random
import string

# wait for 5*5 seconds for hostapd.service to finsh setting up
for i in range(5):
    display.initializing()
    time.sleep(5)

os.system('sudo pigpiod')
os.system('sudo nodogsplash')

GPIO.setmode(GPIO.BCM)

LED_RELAY_PIN  = 5
SONIC_VCC_PIN  = 21
SONIC_TRIG_PIN = 20
SONIC_ECHO_PIN = 16

GPIO.setup(LED_RELAY_PIN, GPIO.OUT)
GPIO.setup(SONIC_VCC_PIN, GPIO.OUT)
GPIO.setup(SONIC_TRIG_PIN, GPIO.OUT)
GPIO.setup(SONIC_ECHO_PIN, GPIO.IN)

GPIO.output(SONIC_VCC_PIN, True)
GPIO.output(SONIC_TRIG_PIN, True) # bug fix
GPIO.output(LED_RELAY_PIN, False) 


# Initialize output directory
output_directory = '/home/botlify/BOTLIFY/output'
data_file_path = os.path.join(output_directory, 'data.json')

if not os.path.exists(output_directory):
    os.makedirs(output_directory)
    
with open(data_file_path, 'w+') as file:
    json.dump({"box": [], "label": "", "score": 0, "area": 0, "minutes": 0, "voucher": ""}, file)

# Initialize servo motor controls
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

SERVO_PIN = 12
factory = PiGPIOFactory()
servo = Servo(SERVO_PIN, pin_factory=factory)

# config
weights_path = '/home/botlify/BOTLIFY/bottle-nano-large-320/weights/best.pt'

# Load the model
model = YOLO(weights_path) 


from multiprocessing import Queue
import cv2, threading, time

# custom bufferless VideoCapture
# a bugfix to get the latest frame in cam.read()
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except Exception as e:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

# Create a VideoCapture object
camera_index = -1
cam = VideoCapture(camera_index)

display.redirecting()

def generate_random_combination():
    characters = string.ascii_letters + string.digits
    result = ''

    for _ in range(5):
        choice = random.choice(characters)
        result += choice

    return result


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
    
    pulse_start = 0
    pulse_end = 0
    
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
    global cam
    global model
    global output_directory
    global data_file_path
    global LED_RELAY_PIN
    
    # Capture frame-by-frame
    frame = cam.read()

    # Crop the frame    
    frame          = frame[50:400, 220:440]
    captured_frame = frame

    results = model.predict(frame, conf=0.77, verbose=False, imgsz=320)
    output = {}
    
    if len(results) > 0:        
        result = results[0] # select the first result only
        boxes = result.boxes.xyxy  # Bounding boxes
        scores = result.boxes.conf  # Confidence scores
        labels = result.boxes.cls  # Class labels (indices)
        
        print('in the loop', boxes, scores, labels)

        for box, score, label in zip(boxes, scores, labels):
            label_name = result.names[int(label)]
            
            # calculate the area of the bounding box
            area = (box[2] - box[0]) * (box[3] - box[1])

            # 1000ml  = 28000 cm^2 = 1.5 min
            # 500ml   = 21000 cm^2 = 1.0 min
            # 290ml   = 17000 cm^2 = 0.5 min
            
            # map the area to the corresponding time
            minutes = 0
            if area > 27000:   # <1000ml
                minutes = 45
            elif area > 21000: # 500ml
                minutes = 25
            else:              # >500ml 
                minutes = 15
            print(area, minutes)
            
            # Create a dictionary with the same information
            output = {
                "box": box.tolist(),
                "label": label_name,
                "score": score.item(),
                "area": area.item(),
                "minutes": minutes,
                "voucher": generate_random_combination()
            }
            
            # Read then increment time
            with open(data_file_path, 'r') as file:
                data = json.load(file)
                output['minutes'] += data['minutes']            
                    
            # Write the json file
            with open(data_file_path, 'w') as file:
                json.dump(output, file)

            # Draw the bounding box on the captured frame
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(captured_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            break
    
        # Save the frame with detections to the output directory
        captured_frame_path = os.path.join(output_directory, 'frame.jpg')
        cv2.imwrite(captured_frame_path, captured_frame)
        
    return output
              
            
time.sleep(1)
display.insert_bottle_a()

def main():
    try:        
        print('Start Listening...')
        
        while True:    
            
            print('Set servo position to mid...')    
            setServoPosition('mid');        
            
            print('Getting distance...')
            detection_threshold = 28 # centimeters
            distance = getDistance()
            print('Distance: ', distance)
            
            sleep(0.1)
            
            if distance < detection_threshold:            
                print('Set servo position to mid...')    
                setServoPosition('mid');        
            
                display.please_wait()
            
                print('Wait 1s..')
                GPIO.output(LED_RELAY_PIN, True) 
                time.sleep(1)
                print('Getting detection..')
                detection = getDetection()
                time.sleep(0.1)
                print('Detections: ', detection)
                GPIO.output(LED_RELAY_PIN, False) 
                time.sleep(0.1)
                
                
                isBottle = False
                if len(detection) > 0:
                    isBottle = True
                    print('Bottle detected...')
                    setServoPosition('max');
                else: 
                    isBottle = False
                    display.invalid()
                    print('No bottle detected...')
                    setServoPosition('min');
                     
            
                # obstruction detection
                # second check distance to see if bottle is dropped
                distance = getDistance()
                while distance < detection_threshold:      
                    print('Set servo position to mid...')    
                    setServoPosition('mid');   
                    
                    print('Obstruction:', distance)
                    display.print_to_tty1("Obstruction! Please contact the administrator.")
                
                    if isBottle:
                        for i in range(3):
                            setServoPosition('mid');   
                            time.sleep(0.2)
                            setServoPosition('max');
                            time.sleep(0.2)
                    else:
                        for i in range(3):
                            setServoPosition('mid');   
                            time.sleep(0.2)
                            setServoPosition('min');
                            time.sleep(0.2)
                            
                    time.sleep(1)
                    distance = getDistance()
                
                # Read then increment time
                with open(data_file_path, 'r') as file:
                    data = json.load(file)
                
                    if data['voucher'] == "":
                        display.insert_bottle_a()
                    else:
                        display.insert_bottle_b(data['voucher'], data['minutes'])

                print('Wait 1 second...')
                sleep(1)    
            
    except Exception as e:
        # restart the program in case of error
        print("Stopped by user, cleaning up...", e)
        display.print_to_tty1(e)
        GPIO.cleanup()
        time.sleep(5)
        display.print_to_tty1("Restarting...")
        time.sleep(2)
        main()
        
          
main()
