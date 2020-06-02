import numpy as np 
import cv2 

videoFeed = cv2.VideoCapture(1) 
while (True): 
    ret, frame = videoFeed.read()
    if ret == False:
        print("Failed to retrieve frame")
        break 
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    cv2.imshow('Feed', frame_gray) 
    if cv2.waitKey(10) & 0xFF == ord("q"): 
        break 
videoFeed.release() 
cv2.destroyAllWindows() 
