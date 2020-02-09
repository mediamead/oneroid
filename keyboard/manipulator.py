#!/usr/bin/env python

N_AXIS = 8

import time
from grbl import Grbl

PORT1 = "/dev/cu.usbmodem145201"
PORT2 = "/dev/cu.usbmodem145101"

class Manipulator(object):
    class GrblAxisMap:
        """
        Maps between manipulator's axes and GRBLs
        """
        def __init__(self, n_axis):
            self.axis_map = [None] * n_axis

        def set_axis_map(self, axis, grbl_n, grbl_axis, grbl_axis_scale=1):
            """
            Map manipulator axis (a positive number) to specific axis on specific instance of GRBL
            Optionally invert the axis
            """
            m = dict()
            m['n'] = grbl_n
            m['axis'] = grbl_axis
            m['scale'] = grbl_axis_scale
            self.axis_map[axis] = m
    
        def get_gcode(self, axis, pos):
            m = self.axis_map[axis]
            cmd = "%s%d" % (m['axis'], pos * m['scale'])
            return m['n'], cmd

    def __init__(self, port1=PORT1, port2=PORT2, speed=115200, homing=False, dry_run=False):
        self.grbls = []
        self.grbls.append(Grbl(port1, speed, homing, dry_run))
        self.grbls.append(Grbl(port2, speed, homing, dry_run))

        # while not self.grbl1.homing_done() or not self.grbl2.homing_done(): pass - FIXME
        self.poss = [0] * N_AXIS # FIXME

        self.gam = self.GrblAxisMap(N_AXIS)
        self.gam.set_axis_map(0, 0, 'A')
        self.gam.set_axis_map(1, 0, 'Z', -1)
        self.gam.set_axis_map(2, 0, 'Y', -1)
        self.gam.set_axis_map(3, 0, 'X')
        self.gam.set_axis_map(4, 1, 'A')
        self.gam.set_axis_map(5, 1, 'Z', -1)
        self.gam.set_axis_map(6, 1, 'X', -1)
        self.gam.set_axis_map(7, 1, 'Y')

    def move(self, pos):
        # build commands for all GRBLs
        grbl_cmds = [[], []]
        for i in range (N_AXIS):
            grbl_n, gcode = self.gam.get_gcode(i, pos[i])
            grbl_cmds[grbl_n].append(gcode)
        
        # send built commands
        for grbl_n in range(len(grbl_cmds)):
            cmd = " ".join(grbl_cmds[grbl_n])
            self.grbls[grbl_n].send(cmd)
        
        self.poss = pos
    
    def get_pos(self):
        return self.poss

if __name__ == "__main__":
    m = Manipulator(homing=True, dry_run=False)
    for _ in range(1):
        A = 10
        for (a0, a1) in ((A, 0), (0, A), (-A, 0), (0, -A)):
            m.move([a0, a1, a0, a1, a0, a1, a0, a1])
            #m.move([0, 0, 0, 0, a0, a1, a0, a1])
        time.sleep(5)
    m.move([0, 0, 0, 0, 0, 0, 0, 0])
    time.sleep(1)