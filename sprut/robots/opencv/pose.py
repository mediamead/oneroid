import cv2
import numpy as np
import glob

np.set_printoptions(linewidth=100, formatter={'float_kind': "{:6.3f}".format})

flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

class Camera:
    # chessboard dimensions
    N, M = 14, 9

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((N*M, 3), np.float32)
    objp[:,:2] = np.mgrid[0:N, 0:M].T.reshape(-1, 2)

    def __init__(self, cam, W, H):
        self.W = W
        self.H = H

        self.cap = cv2.VideoCapture(cam) 
        self.setResolution()
        self.loadCalibration()

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

    def loadCalibration(self):
        filename = "cal-%d-%d.npz" % (self.W, self.H)

        with np.load(filename) as CAL:
            self.mtx, self.dist, _, _ = [CAL[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

        self.newcameramtx, _roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.W, self.H), 1, (self.W, self.H))

    def read(self):
        retval, img = self.cap.read()
        assert(retval)

        img = cv2.undistort(img, self.mtx, self.dist, None, self.newcameramtx)
        return img

    def findChessboardCorners(self, img):
        # Find the chess board corners
        return cv2.findChessboardCornersSB(img, (self.N, self.M), flags=flags)

    def solvePnP(self, corners):
        _, rvecs, tvecs, _ = cv2.solvePnPRansac(self.objp, corners, self.mtx, self.dist)
        return rvecs, tvecs

    def _draw_axis(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        return img

    def findChessboardRTVecs(self, img, drawAxis=False):
        # Finds the chessboard, returns: (img, rvecs, tvecs)
        # img - optionally with with chessboard and 3D axis drawn on it
        retval, corners = self.findChessboardCorners(img)
        if not retval:
            return None, None, None

        rvecs, tvecs = self.solvePnP(corners)
        if drawAxis:
            imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, self.mtx, self.dist)

            img = cv2.drawChessboardCorners(img, (self.N, self.M), corners, retval)
            img = self._draw_axis(img, corners, imgpts)

        return img, rvecs, tvecs

    def close(self):
        self.cap.release() 

# ========================================================================

#cv2.namedWindow("img", cv2.WND_PROP_FULLSCREEN)
#cv2.setWindowProperty("img", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

cap = Camera(1, 1920, 1080)

m0 = None

while True:
    img = cap.read()

    img2, rvecs, tvecs = cap.findChessboardRTVecs(img, drawAxis=True)
    if img2 is None:
        img2 = img
    else:
        d = np.sqrt(np.sum(tvecs**2))
        m = np.concatenate((np.array([d]), tvecs.ravel(), rvecs.ravel()))
        if m0 is None:
            m0 = m
        dm = m - m0
        print("%10.3f | %30s | %30s" % (dm[0], dm[1:4], dm[4:7]))

    cv2.imshow('img', img2) 

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"): 
        break 
    elif k == ord(" "):
        m0 = m

cap.close()

cv2.destroyAllWindows() 
