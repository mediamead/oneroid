"""
Camera class implements initialiation and frame grabbing from OpenCV cameras.
"""

import cv2

class Camera:
    def __init__(self, cam_n, W, H):
        self.W = W
        self.H = H

        self.cap = cv2.VideoCapture(cam_n) #, cv2.CAP_DSHOW)
        self.setResolution()

    def getResolution(self):
        return self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def setResolution(self):
        # Get resolution
        W1, H1 = self.getResolution()
        print("Current resolution: %sx%s" % (W1, H1))

        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)

        # Check resolution
        W1, H1 = self.getResolution()
        print("New resolution: %sx%s" % (W1, H1))
        assert((W1 == self.W) and (H1 == self.H))

    def read(self):
        retval, img = self.cap.read()
        assert(retval)
        return img

    def close(self):
        self.cap.release() 
