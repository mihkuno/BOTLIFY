import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

VCC_PIN  = 21
TRIG_PIN = 20
ECHO_PIN = 16

GPIO.setup(VCC_PIN, GPIO.OUT)
GPIO.output(VCC_PIN, True)

print("Distance Measurement In Progress")

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

try:
    while True:
        dist = distance()
        print("Distance:", dist, "cm")
        time.sleep(1)

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()
