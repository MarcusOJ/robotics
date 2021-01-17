import numpy as np
import cv2
import time
import pyrealsense2 as rs
import json
import main_drive as md

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


#Blobdetector
"""
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.filterByArea = True
blobparams.minArea = 20
blobparams.maxArea = 2000000
blobparams.filterByCircularity = False
blobparams.minDistBetweenBlobs = 50
blobparams.filterByColor = True
blobparams.blobColor = 255
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
detector = cv2.SimpleBlobDetector_create(blobparams)
"""
def updateValue(new_value):
	# make sure to write the new value into the global variable
	global trackbar_value
	trackbar_value = new_value
	return  

with open("values.json", "r") as jfile:
	data = json.load(jfile)

obj = ""
#print(data)
# 0 = bluegoal, 1 = pinkgoal, 2 = ball, 3 = line
try:
	if(data["default"] == 0):
		obj = "bluegoal"
	elif(data["default"] == 1):
		obj = "pinkgoal"
	elif(data["default"] == 2):
		obj = "ball"
	elif(data["default"] == 3):
		obj = "line"
except KeyError:
	print("could not find key")


h_min = data[obj]["h_min"]
s_min = data[obj]["s_min"]
v_min = data[obj]["v_min"]
h_max = data[obj]["h_max"]
s_max = data[obj]["s_max"]
v_max = data[obj]["v_max"]

cv2.namedWindow("Processed")
cv2.createTrackbar("h_min", "Processed", h_min, 179, updateValue)
cv2.createTrackbar("s_min", "Processed", s_min, 255, updateValue)
cv2.createTrackbar("v_min", "Processed", v_min, 255, updateValue)
cv2.createTrackbar("h_max", "Processed", h_max, 179, updateValue)
cv2.createTrackbar("s_max", "Processed", s_max, 255, updateValue)
cv2.createTrackbar("v_max", "Processed", v_max, 255, updateValue)
cv2.createTrackbar("option", "Processed", data["default"], 3, updateValue)

frameCounter = 0
start_time = time.time()
movement_action = ""

while(True):
	now_time = time.time()
	
	# Capture frame-by-frame
	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())


	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
	# colour detection limits
	lH = cv2.getTrackbarPos("h_min", "Processed")
	lS = cv2.getTrackbarPos("s_min", "Processed")
	lV = cv2.getTrackbarPos("v_min", "Processed")
	hH = cv2.getTrackbarPos("h_max", "Processed")
	hS = cv2.getTrackbarPos("s_max", "Processed")
	hV = cv2.getTrackbarPos("v_max", "Processed")
	opt = cv2.getTrackbarPos("option", "Processed")


	lowerLimits = np.array([lH, lS, lV])
	upperLimits = np.array([hH, hS, hV])

	# Our operations on the frame come here
	thresholded = cv2.inRange(hsv_frame, lowerLimits, upperLimits)

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

	
	depth_frame = frames.get_depth_frame()

	if keypoints != []:
		c = max(keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		#print(x, rad)	
		movement_action += " | " + str(rad)
		cv2.putText(img_cp, "Object here", (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)
		
		if depth_frame:
			distance = depth_frame.get_distance(int(x[0]), int(x[1]))
			movement_action = str(distance)
			#print(distance)

	else:
		movement_action = "searching"
	frameCounter += 1
	
	if now_time-start_time >= 1:
		print("FPS:", frameCounter," | " ,movement_action)
		start_time = time.time()
		frameCounter = 0
	# Display the resulting frame
	
	
	# Display the resulting frame
	cv2.imshow('Processed', thresholded)
	cv2.imshow("Object", img_cp)
	# cv2.imshow("Original", frame)
	
	
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		f = open("trackbar_value.txt", "w+")
		pipeline.stop()
		output = {
			"h_min": lH,
			"s_min": lS,
			"v_min": lV,
			"h_max": hH,
			"s_max": hS,
			"v_max": hV
			}
		print(output)
		with open('values.json', 'r') as file:
			json_data = json.load(file)
			for item in json_data[obj]:
				json_data[obj][item] = output[item]
			json_data["default"] = opt
			#print(json_data)
		with open('values.json', 'w') as file:
			json.dump(json_data, file, indent=2)
		
		#f.write(str(output))
		#f.close()
		break
	elif key == ord('t'):
		md.throw(167)
		print("throw")
		#y = 210.364 + (166.8712 - 210.364)/(1 + (x/2.119431)^3.002456)

# When everything done, release the capture

cv2.destroyAllWindows()
