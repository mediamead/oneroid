#!/usr/bin/env python

from KBHit import KBHit
from manipulator import Manipulator

#print("zc/xs/bm/nh - движение новой точки по X/Y/Z/A осям")
#print("Esc - сбросить текущую вернуться к последней точке в сценарии")
#print("Enter - добавить новую точку в сценарий")
# print("Нажии")

class AxesPos:
    """
    Encapsulates information about position of multiple axes and
    an optional comment describing that position
    """

    def __init__(self, pos, nline=None, comment=None):
        self.nline = nline
        self.pos = list(pos)
        self.comment = comment
    
    def clone(self):
        return AxesPos(self.pos)

    def apply_delta(self, axis, delta):
        new_pos = self.pos[axis] + delta
        if new_pos >= -45 and new_pos <= 45:
            self.pos[axis] = new_pos
        else:
            # limit exceeded
            print('\a')

    def __str__(self):
        """
        Returns a string specifying coordinates for all axes
        """
        return str(self.pos)

class Scenario:
    """
    Encapsulates information about sequence of positions we want the manipulator to move between
    """

    def __init__(self, home_pos):
        self.positions = [AxesPos(home_pos, nline=0, comment="HOME")]

    def get_next_nline(self):
        return len(self.positions)

    def add_pos(self, new_pos):
        new_pos.nline = self.get_next_nline()
        self.positions.append(new_pos)
        #print("#FIXME: SAVE SCENARIO LINE %d: %s" % (nline, new_pos))

    def get_pos(self, nline):
        return self.positions[nline]

    def get_first_pos(self):
        return self.positions[0]

    def get_last_pos(self):
        return self.positions[-1]

    def print(self):
        print("=== BEGIN ===")
        for i in range(len(self.positions)):
            print("%3d: %s" % (i, self.positions[i].pos))
        print("=== END ===")

kb = KBHit()

m = Manipulator(homing=False, dry_run=False)
scenario = Scenario(m.get_pos())

for pos in [
   [0, 0, 0, 0, 0, 0, 0, 0],
   [-5, -28, 21, 40, -20, 15, 12, -37],
   [-12, -26, 31, 40, -20, 13, 40, -40],
   [-5, -6, 9, 13, 3, 1, 37, 11],
   [-7, -32, 33, 33, -26, 19, 26, 19],
   [9, -21, 2, 30, -16, 4, -41, 41],
   [-7, -27, 29, 30, -26, 19, 21, 40],
   [-5, -19, 28, 30, -38, 27, 40, 34],
   [0, 0, 0, 0, 0, 0, 0, 0],
   [0, 0, 0, 0, 0, 0, 0, 0]
  ]:
    scenario.add_pos(AxesPos(pos))

def handle_keyboard(ch):
    """
    Returns axis movement tuple (axis, delta) in response to keypresses.
    Keys (zc/xs/bm/nh) correspond to X/Y/Z/A axes.
    Returns None if none of these keys were pressed.
    Lowercase - 1 step, uppercase - 10.
    """

    if ch.isupper():
        step = 10
        ch = ch.lower()
    else:
        step = 1
    # 1
    if ch == 'q':
        axis = 0
        delta = step
    elif ch == 'a':
        axis = 0
        delta = -step
    elif ch == 'w':
        axis = 1
        delta = step
    elif ch == 's':
        axis = 1
        delta = -step
    # 2
    elif ch == 'e':
        axis = 2
        delta = step
    elif ch == 'd':
        axis = 2
        delta = -step
    elif ch == 'r':
        axis = 3
        delta = step
    elif ch == 'f':
        axis = 3
        delta = -step
    # 3
    elif ch == 't':
        axis = 4
        delta = step
    elif ch == 'g':
        axis = 4
        delta = -step
    elif ch == 'y':
        axis = 5
        delta = step
    elif ch == 'h':
        axis = 5
        delta = -step
    # 4
    elif ch == 'u':
        axis = 6
        delta = step
    elif ch == 'j':
        axis = 6
        delta = -step
    elif ch == 'i':
        axis = 7
        delta = step
    elif ch == 'k':
        axis = 7
        delta = -step
    # gotopos
    elif ch >= '0' and ch <= '9':
        return ('goto', ord(ch) - ord('0'))
    # ?
    elif ch == '?':
        return ('dump', None)
    else:
        print("#KBHIT %d" % ord(ch))
        return (None, None)

    # fall-through from axis movement commands
    return ('move', [axis, delta])

# FIXME: load saved scenario

pos = scenario.get_first_pos()

while True:
    print("# %s" % pos)

    while not kb.kbhit():
        pass

    ch = kb.getch()
    cmd, cmd_params = handle_keyboard(ch)
    if cmd == 'move':
        if cmd_params:
            pos.apply_delta(cmd_params[0], cmd_params[1])
            m.move(pos.pos)
        #elif ord(ch) == 27:  # ESC
        #    next_pos = scenario.get_last_pos().clone()
        #    m.move(next_pos.pos)
        #elif ord(ch) == 10:  # ENTER
        #    scenario.add_pos(next_pos)
        #    next_pos = scenario.get_last_pos().clone()
    elif cmd == 'dump': # dump scenario
        scenario.print()
        print("#CURRENT POS: nline=%d pos=%s" % (pos.nline, pos.pos))
    elif cmd == 'goto':
        pos = scenario.get_pos(cmd_params)
        m.move(pos.pos)
        print("#GOTO POS: nline=%d pos=%s" % (pos.nline, pos.pos))
