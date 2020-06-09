"""
Grabs frames from the camera and displays it
"""
import sys
import cv2
from opencv_camera import Camera
from opencv_tools import save, resize

print("### Press 's' to save the frame, 'q' to quit")

cam_n = int(sys.argv[1])
#cam = Camera(cam_n, 1280, 720)
cam = Camera(cam_n, 1920, 1080)

while True:
    img = cam.read()

    img2 = resize(img, 0.5)
    cv2.imshow('img', img2)

    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        break
    elif k == ord('s'):
        save(img)

cam.close()
