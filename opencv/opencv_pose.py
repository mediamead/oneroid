"""
Pose class implements 3D pose estimation from chessboard image
taked by a calibrated OpenCV camera.
"""

import cv2
import numpy as np

flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
zero = np.float32([[0, 0, 0]]).reshape(-1,3)

class Pose:
    N, M = 14, 9 # chessboard dimensions, in tiles
    X0, Y0 = 7, 4 # location of chessboard center, in tiles

    def __init__(self, W, H, cal_file, D):
        # D - size of chessboard tile, in meters
        # CALF - calibration file
        self.loadCalibration(W, H, cal_file)
        self.init_objp(D)

    def loadCalibration(self, W, H, cal_file):
        with np.load(cal_file) as CAL:
            self.mtx, self.dist, _, _ = [CAL[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

        self.newcameramtx, _roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist,
            (W, H), 1, (W, H))

    def init_objp(self, D):
        # prepare 3D object points
        # start with (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.N*self.M, 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.N, 0:self.M].T.reshape(-1, 2)
        # scale and shift to place zero at the center
        self.objp[:, 0] -= self.X0
        self.objp[:, 1] -= self.Y0
        self.objp *= D
    
        # coordinate of axes endpoints (five tiles long)
        DD = D * 5
        self.axes = np.float32([[DD, 0, 0], [0, DD, 0], [0, 0, DD]]).reshape(-1,3)

    def undistort(self, img):
        img = cv2.undistort(img, self.mtx, self.dist, None, self.newcameramtx)
        return img

    def findChessboardCorners(self, img):
        # Find the chess board corners
        return cv2.findChessboardCornersSB(img, (self.N, self.M), flags=flags)

    def _solvePnP(self, corners):
        _, rvecs, tvecs, _ = cv2.solvePnPRansac(self.objp, corners, self.mtx, self.dist)
        return rvecs, tvecs

    def findChessboardRTVecs(self, img):
        # Finds the chessboard, returns: (img, rvecs, tvecs)
        # img - optionally with with chessboard and 3D axes drawn on it
        retval, corners = self.findChessboardCorners(img)
        if not retval:
            return False, None, None, None

        rvecs, tvecs = self._solvePnP(corners)
        return retval, rvecs, tvecs, corners

    def drawAxes(self, img, rvecs, tvecs, corners):
        zeropts, _ = cv2.projectPoints(zero, rvecs, tvecs, self.mtx, self.dist)
        imgpts, _ = cv2.projectPoints(self.axes, rvecs, tvecs, self.mtx, self.dist)

        img = cv2.drawChessboardCorners(img, (self.N, self.M), corners, True)

        zeropt = tuple(zeropts[0].ravel())
        img = cv2.line(img, zeropt, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv2.line(img, zeropt, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv2.line(img, zeropt, tuple(imgpts[2].ravel()), (0,0,255), 5)
        return img
