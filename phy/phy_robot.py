import os
import numpy as np

from phy.manipulator import Manipulator
from opencv.opencv_camera import Camera
from opencv.opencv_pose import Pose

BASEDIR = os.path.dirname(__file__)

CAM_N = 0
CAL_F = os.path.join(BASEDIR, 'cal.npz')
CAL_D = 0.0423 # TV


class PhyRobot:
    W = 1280
    H = 720
    D = 0.0423 # TV

    def __init__(self):
        self.m = Manipulator(homing=False, dry_run=False)
        self.cam = Camera(CAM_N, 1920, 1080)
        self.pose = Pose(self.cam.W, self.cam.H, CAL_F, CAL_D)

    def step(self, phis):
        assert(phis.shape == (4,2))
        phis2 = -phis * 180. / np.pi
        print(phis2)
        self.m.move(phis2.ravel())

    def getHeadcam(self):
        img = self.cam.read()
        return img, None, None, None, None

if __name__ == "__main__":
    r = PhyRobot()
    phis = np.array([[0,0],[0,0],[0,0],[0,0]], dtype=np.float32)
    r.step(phis)
    img = r.getHeadcam()
