"""
Grabs frames from the camera and displays it half size
"""
import sys
import cv2
from opencv_camera import Camera
from opencv_tools import resize, init_argparser, run_argparser

parser = init_argparser(cal_required=False)
args, params = run_argparser(parser)

print("### Press 's' to save the frame, 'q' to quit")

cam = Camera(args.cam_device)

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
