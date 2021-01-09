import numpy as np
import cv2
import time
import pyrealsense2 as rs
import main_drive as md
from time import sleep
import json

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
#pipeline.start(config)

profile = pipeline.start(config)
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure,False)
color_sensor.set_option(rs.option.enable_auto_white_balance, False)
color_sensor.set_option(rs.option.auto_exposure_priority, 0)
color_sensor.set_option(rs.option.exposure, 78)

with open("values.json", "r") as jfile:
	data = json.load(jfile)

ball_lower_limits = np.array([data["ball"]["h_min"], data["ball"]["s_min"], data["ball"]["v_min"]])
ball_upper_limits = np.array([data["ball"]["h_max"], data["ball"]["s_max"], data["ball"]["v_max"]])

blue_lower_limits = np.array([data["bluegoal"]["h_min"], data["bluegoal"]["s_min"], data["bluegoal"]["v_min"]])
blue_upper_limits = np.array([data["bluegoal"]["h_max"], data["bluegoal"]["s_max"], data["bluegoal"]["v_max"]])

pink_lower_limits = np.array([data["pinkgoal"]["h_min"], data["pinkgoal"]["s_min"], data["pinkgoal"]["v_min"]])
pink_upper_limits = np.array([data["pinkgoal"]["h_max"], data["pinkgoal"]["s_max"], data["pinkgoal"]["v_max"]])

line_lower_limits = np.array([data["line"]["h_min"], data["line"]["s_min"], data["line"]["v_min"]])
line_upper_limits = np.array([data["line"]["h_max"], data["line"]["s_max"], data["line"]["v_max"]])

#todo variables for thresholded values

frameCounter = 0
start_time = time.time()

frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
frame = np.asanyarray(color_frame.get_data())
time.sleep(1)

global time_passed, static_time, movement_action
time_passed = False
static_time = time.time()
send_dif = 0.2
directed = False
at_ball = False
throw_ready = False
blue = False


def send_command(can_send, command, value=1):
	global time_passed, static_time
	if(can_send):
		if(command == "stop"):
			md.stop()
		elif(command =="spinleft"):
			md.spinleft()
		elif(command == "spinright"):
			md.spinright()
		elif(command == "searchright"):
			md.searchright()
		elif(command == "f"):
			md.forwards()
		elif(command == "spin"):
			md.spin(value)
		elif(command == "forward_adjust"):
			md.forward_adjust(value)
		elif(command == "circle"):
			md.circleBall()

		time_passed = False
		static_time = time.time()
		print("saadetud")

def find_ball(time_passed):
	send_command(time_passed, "searchright")

def direct(time_passed, mapped_speed):
	global movement_action
	global directed
	send_command(time_passed, "spin", mapped_speed * 5)
	movement_action = "direct"
	if(abs(mapped_speed) <= 1):
		directed = True

def drive_to(time_passed, mapped_speed, rad):
	global movement_action
	global directed, at_ball
	send_command(time_passed, "forward_adjust", (mapped_speed-0.8) * 6)
	movement_action = "drive_to"
	print(rad)
	if(rad > 28):
		at_ball = True

def rotate(time_passed, rad, x, frame):
	global movement_action
	movement_action = "rotate"
	if(blue):
		keypoints = vision(frame, blue_lower_limits, blue_upper_limits)
	else:
		keypoints = vision(frame, pink_lower_limits, pink_upper_limits)

	if keypoints != []:
		c = max(keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		
		if(x[0] <= 340 and x[0] >= 300 ):
			throw_ready = True
			md.stop()
		else:
			send_command(time_passed, "circle")
	
movement_action = ""

def vision(frame, lowerLimits, upperLimits):
	# Capture frame-by-frame

	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Our operations on the frame come here
	thresholded = cv2.inRange(hsv_frame, lowerLimits, upperLimits)

    #Morphological operation
	kernel = np.ones((5,5),np.uint8)
    #erosion = cv2.erode(thresholded,kernel,iterations = 1)
    #dilation = cv2.dilate(thresholded,kernel,iterations = 1)
    #thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)
	thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)

	#outimage = cv2.bitwise_and(frame, frame, mask = thresholded)

    #Keypoint detection
	#keypoints = detector.detect(thresholded)
	
	keypoints, erra = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	return keypoints



while(True):
	now_time = time.time()

	#print(time_passed, now_time - static_time, now_time, static_time)
	if(now_time - static_time >= send_dif):
		time_passed = True

	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())
	
	ball_keypoints = vision(frame, ball_lower_limits, ball_upper_limits)

	if ball_keypoints != []:

		c = max(ball_keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		#print(x, rad)	
		cv2.putText(frame, "Ball here", (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)

		dif = 320 - x[0]
		
		# MOVEMENT SECTION
		if(dif<0):
			mapped_speed = ((dif - (-320))/(0 - (-320)))*(0 - 10) + 10
		else:
			mapped_speed = ((dif - 320)/(0 - 320))*(0 - (-10)) - 10

		#print(mapped_speed)

		if(directed):
			if(at_ball):
				if(throw_ready):
					print("throw")
				else:
					rotate(time_passed, rad, x, frame)
			else:
				drive_to(time_passed, mapped_speed, rad)
		else:
			direct(time_passed, mapped_speed)
	else:
		find_ball(time_passed)
		movement_action = "searching"

	cv2.imshow("Object", frame)

	frameCounter += 1
	
	if now_time-start_time >= 1:
		print("FPS:", frameCounter," | " ,movement_action)
		start_time = time.time()
		frameCounter = 0
    # Display the resulting frame
    
	if cv2.waitKey(1) & 0xFF == ord('q'):
		pipeline.stop()
		break

