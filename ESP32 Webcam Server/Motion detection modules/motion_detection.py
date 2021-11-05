#Import JSON to be able to host the web server session on local machine
import json
from time import time, sleep

#Libraries for machine vision and communication between the ESP and the server
import numpy as np
from PIL import Image
from serial import SerialException, Serial

#Read the files generated from ESP32 from the serial port required for visualization frontend
def serial_to_html_file(port=None, baudrate=115200):
    SHAPE = (48,64)
    with Serial(port, baudrate, timeout=3) as ser:
        changed = []
        while True:
            try:
                line = ser.readline().decode('utd-8').strip()
                #Raw Images processing && save image to disk to update frontend
                if line == 'Current frame:':
                    pixels = []
                    #Read the rows of the image
                    for i in range(SHAPE[0]):
                #Process the difference between current and previous frame
                if line.startswith('diff'):
                    #A pair of the x, y for detected changed tile
                    y, x = line.split('\t')[1:3]
                    changed.append(int(y),int(x))
                #After ESP32 update motion frame -> write new motion image to frontend
                if line.startswith('================='):
            except(SerialException, KeyboardInterrupt):
                break

#connect serial port address to the file
if __name__ == '__main__':
    serial_to_html_file(port='/dev/') #check USB port