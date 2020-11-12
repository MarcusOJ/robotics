import numpy as np
import cv2
import time
import pyrealsense2 as rs
import main_drive as md

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

frameCounter = 0
start_time = time.time()

lowerLimits = np.array([int(ct[0]), int(ct[1]), int(ct[2])])
upperLimits = np.array([int(ct[3]), int(ct[4]), int(ct[5])])

frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
frame = np.asanyarray(color_frame.get_data())
time.sleep(1)

while(True):
	now_time = time.time()
	# Capture frame-by-frame
	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())

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

	if keypoints != []:
		c = max(keypoints, key = cv2.contourArea)
		x, rad = cv2.minEnclosingCircle(c)
		#print(x, rad)	
		cv2.putText(frame, "Ball here", (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)

		dif = 320 - x[0]
		thyst = 20

		if(abs(dif) <= thyst):
			md.stop()
			print("stop")
		if(dif < 0):
			md.spinright()
			print("rightspin")
		if(dif > 0):
			md.spinleft()
			print("leftspin")


	else:
		md.spinleft()
		print("searching")


	cv2.imshow("Object", frame)

	frameCounter += 1
	
	if now_time-start_time >= 1:
		#print(frameCounter)
		start_time = time.time()
		frameCounter = 0
    # Display the resulting frame
    
	if cv2.waitKey(1) & 0xFF == ord('q'):
		pipeline.stop()
		break

# When everything done, release the capture

