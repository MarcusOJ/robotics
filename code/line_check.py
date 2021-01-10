import numpy as np
import cv2
import time
import pyrealsense2 as rs
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


line_lower_limits = np.array([data["line"]["h_min"], data["line"]["s_min"], data["line"]["v_min"]])
line_upper_limits = np.array([data["line"]["h_max"], data["line"]["s_max"], data["line"]["v_max"]])

ball_lower_limits = np.array([data["ball"]["h_min"], data["ball"]["s_min"], data["ball"]["v_min"]])
ball_upper_limits = np.array([data["ball"]["h_max"], data["ball"]["s_max"], data["ball"]["v_max"]])

frameCounter = 0
start_time = time.time()
ran_list = []

def checkline(frame, x):
	global static_time, time_pas, ran_list
	line_key = vision(frame, line_lower_limits, line_upper_limits)
	if(line_key != None):
		behind = False
		suurus = len(line_key)
		print(suurus)
		muutuja = 0
		for i in line_key:
			line = i[0][0].flat[1]
			if(line< x[1] - 40):
				muutuja += 1
		ran_list.append((muutuja/suurus)*100)
		#if muutuja == suurus - 10:
			#behind = True
	if(time_pas):
		print(sum(ran_list)/len(ran_list))
		ran_list = []
		time_pas = False
		static_time = time.time()

		

	
def vision(frame, lower, upper):
	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
	# colour detection limits
	

	# Our operations on the frame come here
	thresholded = cv2.inRange(hsv_frame, lower, upper)

	#Morphological operation
	kernel = np.ones((5,5),np.uint8)
	#erosion = cv2.erode(thresholded,kernel,iterations = 1)
	#dilation = cv2.dilate(thresholded,kernel,iterations = 1)
	#thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)
	thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)

	#outimage = cv2.bitwise_and(frame, frame, mask = thresholded)
	
	#keypoints = detector.detect(thresholded)
	img_cp = frame.copy()
	
	keypoints, erra = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	return keypoints

time_pas = False
static_time = time.time()


while(True):
	now_time = time.time()

	if(now_time - static_time >= 1):
		#print(rad)
		time_pas= True
	
	# Capture frame-by-frame
	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())

	movement_action = ""

	ball_keypoints = vision(frame, ball_lower_limits, ball_upper_limits)

	if ball_keypoints != []:
		c = max(ball_keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		#print(x, rad)	
		#cv2.putText(img_cp, "Object here", (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)
		
		checkline(frame, x)

		dif = 320 - x[0]
		thyst = 20

		


	else:
		movement_action = "searching"
	frameCounter += 1
	
	if now_time-start_time >= 1:
		#print("FPS:", frameCounter," | " ,movement_action)
		start_time = time.time()
		frameCounter = 0
	# Display the resulting frame
	
	
	# Display the resulting frame
	#cv2.imshow('Processed', thresholded)
	#cv2.imshow("Object", img_cp)
	cv2.imshow("Original", frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		f = open("trackbar_value.txt", "w+")
		pipeline.stop()
		#f.write(str(output))
		#f.close()
		break

# When everything done, release the capture

cv2.destroyAllWindows()
