"""
Camera class implements initialiation and frame grabbing from OpenCV cameras.
"""

import cv2

class Camera:
    def __init__(self, cam_n, res=None):
        self.cap = cv2.VideoCapture(cam_n) #, cv2.CAP_DSHOW)
        #self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off

        if res is not None:
            self.setResolution(res)
        else:
            self.W, self.H = self.getResolution()

    def getResolution(self):
        return self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def setResolution(self, res):
        # Get resolution
        W0, H0 = self.getResolution()
        print("Current resolution: %sx%s" % (W0, H0))

        # Set resolution
        self.W, self.H = int(res[0]), int(res[1])
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
