import serial
import math
import cv2
import numpy as np
import time

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
    ser.write('sd:-29:-29:-29\r\n'.encode('utf-8'))


def spinleft():
    ser.write('sd:29:29:29\r\n'.encode('utf-8'))


def circleBall():
    ser.write('sd:-9:0:0\r\n'.encode('utf-8'))


def setspeed(suund):
    speed = 50
    spd1 = int(wheelLogic(speed, wheelone, dist, suund))
    spd2 = int(wheelLogic(speed, wheeltwo, dist, suund))
    spd3 = int(wheelLogic(speed, wheelthree, dist, suund))
    text = ("sd:" + str(spd1) + ":" + str(spd2) + ":" + str(spd3) + "\r\n")
    ser.write('f0\r\n'.encode('utf-8'))
    ser.write(text.encode('utf-8'))
    """"""

def forwards():
    setspeed(90)

def backwards():
    setspeed(270)

def right():
    setspeed(0)

def left():
    setspeed(180)
#    print("mootor1")
#   print(spd1)
# print("mootor2")
#  print(spd2)
# print("mootor3")
# print(spd3)


frame = np.zeros((200,200))

while(True):
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    #print(key)
    # if the 'q' key is pressed, stop the loop
    if key == ord("w"):
        setspeed(90)
    if key == ord("d"):
        setspeed(180)
    if key == ord("s"):
        setspeed(270)
    if key == ord("a"):
        setspeed(0)
    if key == ord("r"):
        spinright()
    if key == ord("l"):
        spinleft()
    if key == ord("f"):
        stop()
    if key == ord("q"):
        shutdown()
        ##cv2.imwrite("test.png", frame)
        break

"""

for i in range(4):
    ser.flushInput()
    setspeed(90)
    time.sleep(1)

# print(wheelLogic(1,wheelone,dist,1,180))
"""