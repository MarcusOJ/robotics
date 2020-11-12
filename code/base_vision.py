import numpy as np
import cv2
import time
import RS_thread_example
import pyrealsense2 as rs

#cap = RS_thread_example.imageCapRS2()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
pipeline.start(config)
        
frameCounter = 0
start_time = time.time()
while(True):

	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	frame = np.asanyarray(color_frame.get_data())

	now_time = time.time()
    # Capture frame-by-frame
	#frame = cap.getFrame()

	##fps = (1 / (time.time() - start_time))
		

	frameCounter += 1
	
	if now_time-start_time >= 1:
		print(frameCounter)
		start_time = time.time()
		frameCounter = 0
		

	cv2.imshow("Object", frame)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		pipeline.stop()
		break

# When everything done, release the capture

cv2.destroyAllWindows()
