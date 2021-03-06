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
import aioserial, asyncio

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
        self.flush()
        # Go through homing or unlock
        if homing:
            self.send("$H")
        else:
            self.send("$X")

    def flush(self):
        print("# + Flushing %s ..." % self.port)
        self.s.timeout = 1
        for _ in range(5):
            line = asyncio.run(self.s.readline_async())
            line = line.decode(errors='ignore').strip()
            print("# Got: %s" % line)
        self.s.timeout = None
        print("# Flushing done")

    def send(self, cmd, debug=False):
        asyncio.run(asyncio.wait([
            self.async_send(cmd, debug),
            self.async_wok(debug)
        ]))
    
    async def async_send(self, cmd, debug=False):
        cmd = (cmd + "\n").encode()
        if self.dry_run:
            print("#DRY send port=%s, cmd=%s" % (self.port, cmd))
            return

        nwritten: int = await self.s.write_async(cmd)
        if debug:
            print("#Grbl.async_send(port=%s, cmd=%s, nwritten=%d)" % (self.port, cmd, nwritten))

    async def async_wok(self, debug=False):
        assert(not self.dry_run)

        while True:
            line = await self.s.readline_async()
            line = line.decode(errors='ignore').strip()
            if debug:
                print("#Grbl.async_wok(port=%s, line=%s" % (self.port, line))
            if line == "ok":
                return
            #if line.startswith("error:"):
            #    return

    async def async_wait_idle(self):
        assert(not self.dry_run)

        while True:
            cmd = ("?").encode()
            nwritten: int = await self.s.write_async(cmd)
            #print("#Grbl.async_wait_idle(port=%s, sent=%s, nwritten=%d)" % (self.port, cmd, nwritten))

            line = await self.s.readline_async()
            line = line.decode(errors='ignore').strip()
            #print("#Grbl.async_wait_idle(port=%s, read=%s" % (self.port, line))
            if line.startswith("<Idle|"):
                break
            time.sleep(0.1)

import re, time
from serial.tools.list_ports import comports

if __name__ == "__main__":
    port1, port2 = find_ports()
    Grbl(port=port1)
    Grbl(port=port2)