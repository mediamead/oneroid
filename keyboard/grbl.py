# AXES = ["X", "Y", "Z", "A"]
# HOME_POS = {"X": 0, "Y": 0, "Z": 0, "A": 0}

# def get_gcode_move(pos0, pos1):
#     """
#     Returns GCODE moving manipulator to pos1
#     If pos0 (current position) is defined, do not emit redundant codes 
#     """
#     moves = []
#     for axis in AXES:
#         if pos0 is not None and pos1[axis] == pos0[axis]:
#             continue
#         move = "%s%d" % (axis, pos1[axis])
#         moves.append(move)
#     return " ".join(moves)

import time
import serial

class Grbl(object):
    def __init__(self, port, speed=115200, homing=False, dry_run=False):
        """
        Connects to single GRBL instance over serial port
        Initiates homing routine
        Does not wait for homing to finish, assumes it will complete successfully - FIXME
        """
        #self.pos = HOME_POS
        self.port = port
        self.dry_run = dry_run
        self._open_stream(port, speed, homing)

    def _open_stream(self, port, speed, homing):
        if self.dry_run:
            print("#DRY _open_stream: port=%s" % port)
            return

        self.s = serial.Serial(port, speed)

        # Wake up grbl
        self.s.write("\r\n\r\n".encode())
        time.sleep(2)   # Wait for grbl to initialize
        self.s.flushInput()  # Flush startup text in serial input

        # Go through homing
        if homing:
            self.send("$H")
        else:
            self.send("$X")
            time.sleep(0.1)

    # def go(self, pos):
    #     gcode = get_gcode_move(self.pos, pos)        
    #     if len(gcode) > 0:
    #         #print("#FIXME: SEND GCODE: %s" % gcode)
    #         self.send(gcode)
    #     self.pos = dict(pos)
    
    def send(self, gcode):
        if self.dry_run:
            print("#DRY send port=%s, gcode=%s" % (self.port, gcode))
            return

        self.s.write((gcode + "\n").encode())
        #print("#Grbl.send(port=%s, gcode=%s" % (self.port, gcode))
