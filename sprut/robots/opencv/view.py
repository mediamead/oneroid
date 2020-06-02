import cv2

# Set resolution
cap = cv2.VideoCapture(1)
print("Default resolution: %sx%s" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print("New resolution: %sx%s" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

while True:
    ret, img = cap.read()
    assert(ret)
    cv2.imshow('img', img)
    cv2.waitKey(100)