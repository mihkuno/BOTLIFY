from gpiozero import Servo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()


servo = Servo(12, pin_factory=factory)

servo.mid()

try:
  while True:
    servo.min()
    sleep(1.5)
    servo.mid()
    sleep(1.5)
    servo.max()
    sleep(1.5)
    servo.mid()
    sleep(1.5)

except KeyboardInterrupt:
  print("Program stopped")


print('program done')

