import cv2
import numpy as np
import glob

N = 14
M = 9

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((N*M, 3), np.float32)
objp[:,:2] = np.mgrid[0:N, 0:M].T.reshape(-1, 2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

FNAME = "cal.npz"

flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY

def draw_axis(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def img2img(img):
    # Find the chess board corners
    retval, corners = cv2.findChessboardCornersSB(img, (N, M), flags=flags)
    if not retval:
        return None

    # Find the rotation and translation vectors.
    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)
    # project 3D points to image plane
    imgpts, _jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

    img2 = cv2.drawChessboardCorners(img, (N, M), corners, retval)
    img2 = draw_axis(img2, corners, imgpts)

    d = np.sqrt(np.sum(tvecs**2))

    print(d, tvecs.ravel(), rvecs.ravel())
    return img2

# ========================================================================

cv2.namedWindow("img", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("img", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

W, H = 1920, 1080

# Load previously saved data
with np.load(FNAME) as CAL:
    mtx, dist, _, _ = [CAL[i] for i in ('mtx','dist','rvecs','tvecs')]
newcameramtx, _roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (W, H), 1, (W, H))

if False:
    for fname in glob.glob('*.jpg'):
        img = cv2.imread(fname)

        img2 = img2img(img)
        if img2 is None:
            continue

        cv2.imshow('img', img2)
        cv2.waitKey(250)
else:    
    videoFeed = cv2.VideoCapture(1) 

    videoFeed.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    videoFeed.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

    while True:
        ret, img = videoFeed.read()
        if ret == False:
            print("Failed to retrieve frame")
            break 

        img2 = cv2.undistort(img, mtx, dist, None, newcameramtx)
        img2 = img2img(img2)
        if img2 is None:
            img2 = img

        cv2.imshow('img', img2) 
        if cv2.waitKey(10) & 0xFF == ord("q"): 
            break 

    videoFeed.release() 
    cv2.destroyAllWindows() 
