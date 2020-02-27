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

def find_ports(SER1="858303033393515190B0", SER2="5583834363335110D111"):
  while True:
    print("# ==== ")
    port1 = port2 = None
    for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
        #(1, ('COM3', 'Arduino Mega 2560 (COM3)', 'USB VID:PID=2341:0042 SER=5583834363335110D111 LOCATION=1-7'))
        print('# COMPORT %d port=%s desc=%s hwid=%s' % (n, port, desc, hwid))
        m = re.search(" SER=([^ ]+)", hwid) 
        if m is None:
            print("# - NO SERIALNO, ignoring")
        else:
            ser = m.group(1)
            if ser == SER1:
                port1 =  port
                print("# + IDENTIFIED AS PORT1: %s" % port1)
            elif ser == SER2:
                port2 = port
                print("# + IDENTIFIED AS PORT2: %s" % port2)
            else:
                print("# ? UNKNOWN SERIALNO, ignoring")

    if port1 is not None and port2 is not None:
        print("# + Found PORT1=%s PORT2=%s" % (port1, port2))
        return (port1, port2)
    if port1 is None:
        print("# ! PORT1 not found")
    if port2 is None:
        print("# ! PORT2 not found")

    time.sleep(2)

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

        self.s = aioserial.AioSerial(port=port, baudrate=speed)

        # Go through homing
        if homing:
            self.send("$H")
        else:
            self.send("$X")

    def send(self, cmd):
        asyncio.run(asyncio.wait([
            self.grbls[grbl_n].async_send(cmd),
            self.grbls[grbl_n].async_wok()
        ]))
    
    async def async_send(self, cmd):
        if self.dry_run:
            print("#DRY send port=%s, cmd=%s" % (self.port, cmd))
            return

        nwritten: int = await aioserial_instance.write_async(cmd)
        print("#Grbl.async_send(port=%s, cmd=%s, nwritten=%d)" % (self.port, cmd, nwritten))

    async def async_wok(self):
        line: bytes = await aioserial_instance.readline_async()
        ok = line.decode(errors='ignore').strip()
        print("#Grbl.async_wok(port=%s, ok=%s" % (self.port, ok))
        if ok != "ok":
            print("# ERROR: got %s instead of 'ok' on port %s" % (ok, self.port))

import re, time
from serial.tools.list_ports import comports

if __name__ == "__main__":
    port1, port2 = find_ports()
    Grbl(port=port1)
    Grbl(port=port2)