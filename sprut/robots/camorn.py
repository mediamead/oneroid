import cv2
import numpy as np

from pylsd.lsd import lsd

def get_green_line(brg_img):
    # https://stackoverflow.com/questions/47483951/how-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image
    img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2HLS)
    lower = np.uint8([40, 40,40])
    upper = np.uint8([70, 255, 255])
    green_mask = cv2.inRange(img, lower, upper)

    lines = lsd(green_mask)
    for i in range(lines.shape[0]):
        pt1 = (int(lines[i, 0]), int(lines[i, 1]))
        pt2 = (int(lines[i, 2]), int(lines[i, 3]))
        width = lines[i, 4]
        cv2.line(brg_img, pt1, pt2, (0, 0, 255), int(np.ceil(width / 2)))
        cv2.imshow("mask", brg_img)
        cv2.waitKey(1)

        return (lines[i])

def get_cam_orn(bgr_img):
    """
    Detects green line on the given BGR image
    Returns:
        alpha = inclination angle (from axis X is zero, CCW is positive) in radians
    """
    line = get_green_line(bgr_img)
    if line is None:
        return None
    
    H, W = bgr_img.shape[0:2]
    lx = (line[2] - line[0]) / W
    ly = (line[3] - line[1]) / W
    if lx < 0:
        (lx, ly) = (-lx, -ly)
    print("W=%d H=%d, lx=%.3f, ly=%.3f" % (W, H, lx, ly))
    l = np.sqrt(lx**2 + ly**2)
    angle = np.arctan2(ly, lx)

    
    print("lenght=%f, angle=%f" % (l, angle/np.pi*180))
