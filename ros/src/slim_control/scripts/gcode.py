#!/usr/bin/env python
"""
Simple g-code script
"""
 
import serial
import time
 
# Open serial port
s = serial.Serial('/dev/ttyACM0',115200)
 
# Wake up board
s.write("\r\n\r\n")
time.sleep(2)   # Wait for board to initialize
s.flushInput()  # Flush startup text in serial input
 
print 'about to write'
s.write("G0 X20 Z20\n")
# s.write("M114\n")
print 'done writing'
output = s.readline()
print output.strip()

s.close()
