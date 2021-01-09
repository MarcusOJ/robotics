import numpy as np
import cv2
import time
import pyrealsense2 as rs

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

#Väärtuste lugemine failist
default = [100, 100, 100, 100, 100, 100]
try:
	c = open("trackbar_value.txt", "r")
except FileNotFoundError:
	c = open("trackbar_value.txt", "w+")
	c.write(str(default))
contents = c.read()
if contents == "":
	contents = str(default)

contents = contents.replace("[", "")
contents = contents.replace("]", "")
contents = contents.replace(",", " ")
ct = contents.split()
c.close()

cv2.namedWindow("Processed")
cv2.createTrackbar("h_min", "Processed", int(ct[0]), 179, updateValue)
cv2.createTrackbar("s_min", "Processed", int(ct[1]), 255, updateValue)
cv2.createTrackbar("v_min", "Processed", int(ct[2]), 255, updateValue)
cv2.createTrackbar("h_max", "Processed", int(ct[3]), 179, updateValue)
cv2.createTrackbar("s_max", "Processed", int(ct[4]), 255, updateValue)
cv2.createTrackbar("v_max", "Processed", int(ct[5]), 255, updateValue)



frameCounter = 0
start_time = time.time()

while(True):
	now_time = time.time()
	
	# Capture frame-by-frame
	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())


	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
	# colour detection limits
	lB = cv2.getTrackbarPos("h_min", "Processed")
	lG = cv2.getTrackbarPos("s_min", "Processed")
	lR = cv2.getTrackbarPos("v_min", "Processed")
	hB = cv2.getTrackbarPos("h_max", "Processed")
	hG = cv2.getTrackbarPos("s_max", "Processed")
	hR = cv2.getTrackbarPos("v_max", "Processed")

	lowerLimits = np.array([lB, lG, lR])
	upperLimits = np.array([hB, hG, hR])

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

	movement_action = ""

	if keypoints != []:
		c = max(keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		#print(x, rad)	
		cv2.putText(img_cp, "Ball here", (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)
		
		dif = 320 - x[0]
		thyst = 20

		if(abs(dif) <= thyst):
			movement_action = "stop"
		if(dif < 0):
			movement_action = "rightspin"
		if(dif > 0):
			movement_action = "leftspin"


	else:
		movement_action = "searching"
	frameCounter += 1
	
	if now_time-start_time >= 1:
		print("FPS:", frameCounter," | " ,movement_action, " | ", rad)
		start_time = time.time()
		frameCounter = 0
	# Display the resulting frame
	
	
	# Display the resulting frame
	cv2.imshow('Processed', thresholded)
	cv2.imshow("Object", img_cp)
	# cv2.imshow("Original", frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		f = open("trackbar_value.txt", "w+")
		pipeline.stop()
		output = [lB, lG, lR, hB, hG, hR]
		f.write(str(output))
		f.close()
		break

# When everything done, release the capture

cv2.destroyAllWindows()
