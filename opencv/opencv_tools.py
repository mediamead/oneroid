import cv2
import time
import argparse

def init_argparser(cal_required):
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--cam-device", type=int,
        required=True, help="OpenCV camera device")

    parser.add_argument("-c", "--cal-file", type=str,
        required=cal_required, help="Camera calibration file (created by calibrate.py)")

    parser.add_argument("-p", "--cal-pattern", choices=['A4', 'A3', 'TV'],
        required=cal_required, help='Calibration pattern')
    return parser

def run_argparser(parser):
    args = parser.parse_args()
    params = dict()

    if args.cal_pattern == "A4":
        params['D'] = 0.0116 # A4 (home wall tests)
    elif args.cal_pattern == "A3":
        params['D'] = 0.0232 # A3 (door)
    elif args.cal_pattern == "TV":
        params['D'] = 0.0423 # TV
    elif args.cal_pattern is not None:
        raise argparse.ArgumentTypeError('bad calibration pattern value')

    return args, params

def save(img):
    filename = "img-%d.jpg" % int(time.time() * 1e6)
    cv2.imwrite(filename, img)
    print("image saved in '%s'" % filename)

def resize(img, scale):
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    img = cv2.resize(img, (width, height), interpolation = cv2.INTER_AREA)
    return img
