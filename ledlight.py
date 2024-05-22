import RPi.GPIO as GPIO
import time

# Use BCM GPIO references instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin to use (e.g., GPIO 17)
relay_pin = 5

# Set up the GPIO pin as an output
GPIO.setup(relay_pin, GPIO.OUT)
GPIO.output(relay_pin, GPIO.HIGH)


time.sleep(1)


try:
    while True:
        # Turn the relay on
        print("Turning relay ON")
        GPIO.output(relay_pin, True)
        time.sleep(0.1)  # Keep the relay on for 5 seconds

        # Turn the relay off
        print("Turning relay OFF")
        GPIO.output(relay_pin, False)
        time.sleep(0.1)  # Keep the relay off for 5 seconds

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    # Clean up the GPIO settings
    GPIO.cleanup()
