import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'

motionInfo = ""

ser = serial.Serial(SERIAL_PORT,9600,timeout=0)  # open serial port for non-blocking reads
while True:
    x = ser.read(1).decode('utf-8')
    while len(x) > 0 :
        if x == "\n":
            print(motionInfo)
            motionInfo = ""
        else:
            motionInfo += x
        x = ser.read(1).decode('utf-8')
    time.sleep(0.5)



