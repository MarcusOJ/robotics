import serial
from time import sleep
ser = serial.Serial(port="/dev/ttyACM0",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			stopbits=serial.STOPBITS_ONE,
			timeout=3
			)

ser.isOpen()
ser.write("sd:-9:0:0".encode("utf-8"))
sleep(.1)
ser.flushInput()
ser.flushOutput()
sleep(.1)
ser.close()