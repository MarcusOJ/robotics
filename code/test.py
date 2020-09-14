import serial 

ser = serial.Serial(port="/dev/ttyACM0",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			stopbits=serial.STOPBITS_ONE
			)


ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

input=1
while (True):
	
   	 # get keyboard input

	stuf = input(">> ")
	print(stuf)
        # Python 3 users
        # input = input(">> ")
	if (stuf == 'exit'):
        	ser.close()
        	exit()
	else:
        # send the character to the device
        # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
        	ser.write(stuf + '\r\n')
	        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        	time.sleep(1)
	        while ser.inWaiting() > 0:
        	    out += ser.read(1)

        	if out != '':
	            print(">>" + out)
	stuf = 0
	      
