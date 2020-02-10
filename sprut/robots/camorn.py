import cv2
import numpy as np

from pylsd.lsd import lsd

def get_horizon_lines(bgr_img):
    """
    Given BGR image,
    Returns list of green lines (x1,y1,x2,y2,w).
    """
    # extract green color into green_mask
    # https://stackoverflow.com/questions/47483951/how-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image
    img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HLS)
    lower = np.uint8([40, 40,40])
    upper = np.uint8([70, 255, 255])
    green_mask = cv2.inRange(img, lower, upper)

    return lsd(green_mask)

def get_horizon_bank(bgr_img):
    """
    Given BGR image,
    Returns horizon rotation angle (in radians, growing from axis X towards Y)
        or None if horizon line not found
    """
    lines = get_horizon_lines(bgr_img)
    H, W = bgr_img.shape[0:2]
    for i in range(lines.shape[0]):
        lx = (lines[i, 2] - lines[i, 0]) / W
        ly = (lines[i, 3] - lines[i, 1]) / W
        l = np.sqrt(lx**2 + ly**2)
        #print("W=%d H=%d, lx=%.3f, ly=%.3f, l=%.3f" % (W, H, lx, ly, l))

        if l >= 0.5:
            # long enough, qualifies as a horizon
            if False:
                # show camera image, overlay red lines over detected segment 
                pt1 = (int(lines[i, 0]), int(lines[i, 1]))
                pt2 = (int(lines[i, 2]), int(lines[i, 3]))
                width = lines[i, 4]
                cv2.line(bgr_img, pt1, pt2, (0, 0, 255), int(np.ceil(width / 2)))
                cv2.imshow("brg_img", bgr_img)
                cv2.waitKey(1)

            if lx < 0:
                # compensate for LSD's random choice of start and end points
                (lx, ly) = (-lx, -ly)

            angle = np.arctan2(ly, lx)
            #print("angle=%f" % (angle/np.pi*180))

            return angle

    return None
