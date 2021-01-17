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
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
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

#frames = pipeline.wait_for_frames()
#color_frame = frames.get_color_frame()
#frame = np.asanyarray(color_frame.get_data())
#depth_frame = frames.get_depth_frame()
time.sleep(1)

global time_passed, static_time, movement_action, at_ball, throw_ready, time_pas, directed, speed, timeout_init, timeout_rotate, static_time_rotate
time_passed = False
static_time = time.time()
static_time_line = time.time()
send_dif = 0.2
directed = False
at_ball = False
throw_ready = False
blue = True
ran_list = []
time_pas = False
precentage = 0
speed = 20
static_time_rotate = time.time()
timeout_rotate = False
timeout_init = False

def reset():
	global time_passed, static_time, movement_action, at_ball, throw_ready, time_pas, directed
	time_passed = False
	directed = False
	at_ball = False
	throw_ready = False
	time_pas = False
	time_passed = False

def send_command(can_send, command, value=1, value_2=1):
	global time_passed, static_time

	if(command == "skip"):
		md.skip_90()

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
			md.forward_adjust(value, value_2)
		elif(command == "circle"):
			md.circleBall(value)

		time_passed = False
		static_time = time.time()
		#print("saadetud")

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
	global movement_action, precentage, directed
	global directed, at_ball
	send_command(time_passed, "forward_adjust", (mapped_speed-0.8) * (rad/2), int(speed))
	movement_action = "drive_to | " + "precentage: " + str(round(precentage)) + "% | rad: " + str(round(rad))

	if(rad > 10):
		if(precentage < 50):
			directed = False
			movement_action += "| skip"
			send_command(time_passed, "skip")
	if(rad > 35):
		md.stop()
		movement_action += " | at ball"
		at_ball = True

def rotate(time_passed, rad, x, frame, mapped_speed):
	global movement_action, throw_ready, at_ball, directed, timeout_rotate, timeout_init, static_time_rotate
	movement_action = "rotate"
	if(blue):
		keypoints = vision(frame, blue_lower_limits, blue_upper_limits)
	else:
		keypoints = vision(frame, pink_lower_limits, pink_upper_limits)

	if keypoints != []:
		c = max(keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		
		movement_action +=  " | goal: " + str(round(x[0])) + " | rad: " + str(rad)
		if(x[0] <= 340 and x[0] >= 300 ):
			if(x[0] <= 340 and x[0] >= 300 ):
				throw_ready = True
				print("Throw ready = " + str(throw_ready))
				md.stop()
				exit()

	else:
		movement_action += " | no goal"
		send_command(time_passed, "circle", mapped_speed * 2)
	"""
	if(timeout_rotate):
		at_ball = False
		directed = False
		timeout_rotate = False
		timeout_init = False
		print("reset")
	if(not timeout_init):
		static_time_rotate = time.time()
		timeout_init = True
		print("timeout init")
	"""

def throw(time_passed, vision, depth_frame, frame):
	global movement_action
	if(blue):
		keypoints = vision(frame, blue_lower_limits, blue_upper_limits)
	else:
		keypoints = vision(frame, pink_lower_limits, pink_upper_limits)

	if keypoints != []:
		c = max(keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		
		if depth_frame:
			distance = depth_frame.get_distance(int(x[0]), int(x[1]))
			movement_action = "distance from goal: "+ str(distance)
			reset()
	return
	
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

def checkline(frame, x):
	global static_time_line, time_pas, ran_list, precentage, movement_action
	line_key = vision(frame, line_lower_limits, line_upper_limits)
	if(len(line_key) != 0):
		if(line_key != None):
			behind = False
			suurus = len(line_key)
			muutuja = 0
			if(suurus < 30):
				precentage = 100
				return
			for i in line_key:
				line = i[0][0].flat[1]
				if(line < x[1]):
					muutuja += 1
			ran_list.append((muutuja/suurus)*100)
		if(time_pas):
			precentage = sum(ran_list)/len(ran_list)
			movement_action += " | " + str(precentage) + "%"
			ran_list = []
			time_pas = False
			static_time_line = time.time()
	else:
		precentage = 100

while(True):
	now_time = time.time()

	#print(time_passed, now_time - static_time, now_time, static_time)
	if(now_time - static_time >= send_dif):
		time_passed = True
	if(now_time - static_time_line >= 0.2):
		time_pas = True
	"""
	if(now_time - static_time_rotate >= 7):
		timeout_rotate = True
	"""
	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())
	depth_frame = frames.get_depth_frame()

	ball_keypoints = vision(frame, ball_lower_limits, ball_upper_limits)

	if ball_keypoints != []:

		c = max(ball_keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		speed = 80/(rad/8)
		if(speed > 90):
			speed = 90
		#print(x, rad)	
		cv2.putText(frame, "Ball here", (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)

		dif = 320 - x[0]
		
		# MOVEMENT SECTION
		if(dif<0):
			mapped_speed = ((dif - (-320))/(0 - (-320)))*(0 - 10) + 10
		else:
			mapped_speed = ((dif - 320)/(0 - 320))*(0 - (-10)) - 10

		#print(mapped_speed)
		checkline(frame, x)
		#print(precentage)

		if(directed):
			if(at_ball):
				if(throw_ready):
					throw(time_passed, vision, depth_frame, frame)
				else:
					rotate(time_passed, rad, x, frame, mapped_speed)
			else:
				drive_to(time_passed, mapped_speed, rad)
		else:
			direct(time_passed, mapped_speed)
	else:
		find_ball(time_passed)
		movement_action = "searching"
		directed = False
		at_ball = False
		throw_ready = False

	cv2.imshow("Object", frame)

	frameCounter += 1
	
	if now_time-start_time >= 0.25:
		print("FPS:", (frameCounter * 4)," | ", movement_action)
		start_time = time.time()
		frameCounter = 0
    # Display the resulting frame
    
	if cv2.waitKey(1) & 0xFF == ord('q'):
		pipeline.stop()
		break

