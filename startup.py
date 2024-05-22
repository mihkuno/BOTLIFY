#!/home/botlify/BOTLIFY/venv/bin/python

import os
import time
import subprocess
import sys

import display

def clear_tty1():
    with open('/dev/tty1', 'w') as tty1:
        tty1.write('\033c')  # ANSI escape code to reset the terminal


def print_to_tty1(message):
    with open('/dev/tty1', 'w') as tty1:
        tty1.write(message + '\n')



for i in range(5):
    display.initializing()
    time.sleep(5)


os.system('sudo pigpiod')
os.system('sudo nodogsplash')

display.redirecting()

time.sleep(3)
display.insert_bottle_a()

print_to_tty1('starting main.py')

# Run main.py
subprocess.run([sys.executable, '/home/botlify/BOTLIFY/main.py'])

# Exit the current script
sys.exit()