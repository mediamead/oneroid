"""
Grabs frames from the camera and shows them
"""

import cv2
import time

# Get resolution
cap = cv2.VideoCapture(1)
print("Default resolution: %sx%s" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
print("New resolution: %sx%s" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

def save(img):
    filename = "img-%d.jpg" % int(time.time() * 1e6)
    cv2.imwrite(filename, img)
    print("captured image saved in '%s'" % filename)

print("### Press 's' to save the frame, 'q' to quit")

while True:
    retval, img = cap.read()
    assert(retval)

    cv2.imshow('img', img)

    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        break
    elif k == ord('s'):
        save(img)
