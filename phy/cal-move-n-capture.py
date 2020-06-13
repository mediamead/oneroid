"""
This script is used to capture calibration images. It:
 * moves the manipulator through scenario-cal.csv, and for each:
 * captures webcam image,
 * checks that a chessboard is present,
 * saves the image into current directory.
Normally you should run this script in a dedicated cal-XXX/directory first,
then run calibrate.py in the same directory.
"""
import time

from manipulator import Manipulator
from scenario import Scenario
from XXX import Camera
from YYY import Pose

BASEDIR = os.path.dirname(__file__)
scenario_file = os.path.join(BASEDIR, 'scenario-cal.csv')

def move_n_capture(m, poss, cam, pose):
    # go through all lines in the scenario, starting 1
    for i in range(1, len(poss)):
        print("*** Position %d of %d ***" % (i, len(poss)-1))
        m.move(poss[i].pos)
        time.sleep(1)

        while True:
            img = cam.read()
            retval, _, img2 = pose.find_chessboard(img, draw_axes=True)
            if retval:
                utils.imshow2(img2)
                utils.save(img)
                print("# chessboard found!")
                break
            else:
                utils.imshow2(img)
                print("# chessboard is not found, retrying ...")

# ------------------------------------------------------

cam = Camera()
pose = Pose()

cal_scenario = Scenario(scenario_file)
poss = cal_scenario.positions

m = Manipulator(homing=False, dry_run=True)

try:
    move_n_capture(m, poss, cam, pose):
finally:
    # return to zero position
    m.move(poss[0].pos)