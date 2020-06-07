import cv2
import time

def save(img):
    filename = "img-%d.jpg" % int(time.time() * 1e6)
    cv2.imwrite(filename, img)
    print("image saved in '%s'" % filename)

def resize(img, scale):
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    img = cv2.resize(img, (width, height), interpolation = cv2.INTER_AREA)
    return img
