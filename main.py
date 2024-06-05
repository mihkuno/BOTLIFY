#!/home/botlify/BOTLIFY/venv/bin/python

import os
import time
import subprocess
import sys

# data handler
import json
import display

# wait for 5*5 seconds for hostapd.service to finsh setting up
for i in range(5):
    display.initializing()
    time.sleep(5)

# image classification
import cv2
import numpy as np
import pandas as pd
import mediapipe as mp
from datetime import datetime, timedelta

# Bufferless VideoCapture
from multiprocessing import Queue
import threading, time

# Initialize GPIO pins
import RPi.GPIO as GPIO
import time

# Voucher generation
import random
import string

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
    json.dump({"box": [], "label": "", "score": 0, "diagonal": 0, "minutes": 0, "voucher": ""}, file)

# Initialize servo motor controls
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

SERVO_PIN = 12
factory = PiGPIOFactory()
servo = Servo(SERVO_PIN, pin_factory=factory)



model_path = "/home/botlify/BOTLIFY/model.tflite"

BaseOptions = mp.tasks.BaseOptions
ImageClassifier = mp.tasks.vision.ImageClassifier
ImageClassifierOptions = mp.tasks.vision.ImageClassifierOptions
VisionRunningMode = mp.tasks.vision.RunningMode
ImageFormat = mp.ImageFormat

options = ImageClassifierOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    max_results=5,
    running_mode=VisionRunningMode.IMAGE
)


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


def getDetection(classifier):
    global cam
    global output_directory
    global data_file_path
    global LED_RELAY_PIN
    
    # Capture frame-by-frame
    frame = cam.read()

    # Crop the frame    
    cropped_image = frame[60:355, 260:440]
    
    # Convert the BGR image to RGB (MediaPipe expects RGB format)
    rgb_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)

    # Convert the image to a MediaPipe Image object
    mp_image = mp.Image(image_format=ImageFormat.SRGB, data=rgb_image)

    # Perform image classification on the provided single image.
    classification_result = classifier.classify(mp_image)

    # Print classification results
    result = classification_result.classifications[0].categories[0]
    category_name = result.category_name
    score = round(result.score, 2)
    
    output = None
    
    if category_name == 'plastic':
        
        blurred_image = cv2.GaussianBlur(cropped_image, (13, 13), 0)
        grayed_image  = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
        canny_image = cv2.Canny(grayed_image, 0, 30)
        
        kernel = np.ones((5, 5), np.uint8)
        dilate_image = cv2.dilate(canny_image, kernel, iterations=1)

        contours, hierarchy = cv2.findContours(dilate_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        max_area = 0
        max_cnt = []
        for index, cnt in enumerate(contours):
            
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_cnt = cnt
        
        contour_image = cv2.drawContours(cropped_image, max_cnt, -1, (0, 255, 0), 3)
        
        peri = cv2.arcLength(max_cnt, True)
        approx = cv2.approxPolyDP(max_cnt, 0.02 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)
        diagonal = int(np.sqrt(w**2 + h**2))

    
        # map the area to the corresponding time
        minutes = 0
        if diagonal > 280:   # <1000ml
            minutes = 45
        elif diagonal > 240: # 500ml
            minutes = 25
        else:                # >500ml
            minutes = 15
        print(diagonal, minutes)
            
        # Create a dictionary with the same information
        output = {
            "box": [x,y,w,h],
            "label": category_name,
            "score": score,
            "diagonal": diagonal,
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

        # Save the frame with detections to the output directory        
        cv2.line(contour_image, (x, y), (x+w, y+h), (0, 0, 255), 2)        
        cv2.rectangle(contour_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(contour_image, f'Area: {int(max_area)}', (x, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(contour_image, f'Diagonal: {diagonal}', (x, y+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        captured_frame_path = os.path.join(output_directory, 'frame.jpg')
        cv2.imwrite(captured_frame_path, contour_image)
        
    return output
    
              
def main():
    with ImageClassifier.create_from_options(options) as classifier:
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
                
                    print('Wait 1.2s..')
                    GPIO.output(LED_RELAY_PIN, True) 
                    time.sleep(1.2)
                    print('Getting detection..')
                    detection = getDetection(classifier)
                    time.sleep(0.1)
                    print('Detections: ', detection)
                    GPIO.output(LED_RELAY_PIN, False) 
                    time.sleep(0.1)
                    
                    isBottle = False
                    if detection == None:
                        isBottle = False
                        display.invalid()
                        print('No bottle detected...')
                        setServoPosition('min');
                        time.sleep(0.5)
                    else: 
                        isBottle = True
                        print('Bottle detected...')
                        setServoPosition('max');
                        
                
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
            

time.sleep(1)
display.insert_bottle_a()          
main()
