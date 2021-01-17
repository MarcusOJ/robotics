import serial
import math
import cv2
import numpy as np
from time import sleep

dist = 0.115
wheelone = 0
wheeltwo = 120
wheelthree = 240


ser = serial.Serial(port="/dev/ttyACM0",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE
            )

ser.isOpen()

# print(ser.isOpen())

# wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + wheelDistanceFromCenter * robotAngularVelocity

def wheelLogic(robotspeed, wheel, dist, suund):
    angVel = robotspeed / 0.035
    speed = robotspeed * math.cos(math.radians(suund - wheel))  # +dist*angVel
    return speed


def shutdown():
    ser.write('sd:0:0:0\r\n'.encode('utf-8'))

def stop():
    ser.write('sd:0:0:0\r\n'.encode('utf-8'))


def spinright():
    ser.write('sd:-10:-10:-10\r\n'.encode('utf-8'))


def spinleft():
    ser.write('sd:10:10:10\r\n'.encode('utf-8'))

def searchright():
    ser.write('sd:16:16:16\r\n'.encode('utf-8'))

def spin(speed):
    ser.write(('sd:' + str(speed) + ':0:0\r\n').encode('utf-8'))
    sleep(0.05)
    ser.read()

def forward_adjust(speed, another):
    #print('sd:' + str(speed) + ':' + str(another) + ':-' + str(another) + '\r\n')
    ser.write(('sd:' + str(speed) + ':' + str(another) + ':-' + str(another) + '\r\n').encode('utf-8'))
    sleep(0.05)
    ser.read()   

def circleBall(speed):
    ser.write(('sd:-30:' + str(speed) + ':0\r\n').encode('utf-8')) 

def skip_90():
    ser.write('sd:16:16:16\r\n'.encode('utf-8'))
    sleep(0.5)

def setspeed(suund):
    speed = 40
    spd1 = int(wheelLogic(speed, wheelone, dist, suund))
    spd2 = int(wheelLogic(speed, wheeltwo, dist, suund))
    spd3 = int(wheelLogic(speed, wheelthree, dist, suund))
    text = ("sd:" + str(spd1) + ":" + str(spd2) + ":" + str(spd3) + "\r\n")
    #ser.write('f0\r\n'.encode('utf-8'))
    ser.write(text.encode('utf-8'))
    sleep(.1)
    ser.read()
    
    ### KUI HAKKAB KOKKU JOOKSMA ILMA PÕHHJUSETA SIIS PROOVI SEDA 

    """
    while(ser.inWaiting() > 0):
        ser.read()
    """

def throw(speed):
    for i in range(5):
        ser.write(('d:' + str(speed) + '\r\n').encode('utf-8'))
        sleep(0.1)
    forwards()
    sleep(0.5)
    stop()

def close_connection():
    ser.close()
    print("Serial connection closed!")

def forwards():
    setspeed(90)

def backwards():
    setspeed(270)

def right():
    setspeed(0)

def left():
    setspeed(180)
