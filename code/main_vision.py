import numpy as np
import cv2
import time
import RS_thread_example

cap = RS_thread_example.imageCapRS2()

#Blobdetector
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

while(True):
    # Capture frame-by-frame
    frame = cap.getFrame()

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lowerLimits = np.array([int(ct[0]), int(ct[1]), int(ct[2])])
    upperLimits = np.array([int(ct[3]), int(ct[4]), int(ct[5])])   

    # Our operations on the frame come here
    thresholded = cv2.inRange(hsv_frame, lowerLimits, upperLimits)

    #Morphological operation
    kernel = np.ones((5,5),np.uint8)
    #erosion = cv2.erode(thresholded,kernel,iterations = 1)
    #dilation = cv2.dilate(thresholded,kernel,iterations = 1)
    #thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)
    thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)

    outimage = cv2.bitwise_and(frame, frame, mask = thresholded)

    #Keypoint detection
    #keypoints = detector.detect(thresholded)
    keypoints = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if keypoints != []:
        area = cv2.contourArea(keypoints[0][0])
        print(area)
    """
    img_cp = frame.copy()
    img_cp = cv2.drawKeypoints(img_cp, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    if keypoints != []:    
        n = 0
        for i in keypoints:
            x = keypoints[n].pt
            cv2.putText(img_cp, "Ball here" + " X: " + str(round(x[0])) + " Y: " + str(round(x[1])), (int(x[0]), int(x[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 2)
            n+=1
    """
    
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.setStopped(False)
        break

# When everything done, release the capture

