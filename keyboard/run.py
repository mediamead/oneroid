#!/usr/bin/env python

## --------------------------------------------------

class MoveCmd:
    def __init__(self, line):
        self.gcode1 = None
        self.gcode2 = None
        s1.write(("%s\n" % cmd1).encode())
    s2.write(("%s\n" % cmd2).encode())
    time.sleep(1.5)

    def run(self):
        pass

class WaitCmd:
    def __init__(self, line):  
        self.delay = float(line)

    def run(self):
        time.sleep(self.delay)    

## --------------------------------------------------

cmd = None
cmds = []
comments = []

def parse_lines(lines):
    for i in range(len(lines)):
        if lines[i].startswith("#"):
            comments.append(line[i])
        elif lines[i].startswith("G"):
            cmd = MoveCmd(lines[i][1:], comments)
        elif lines[i].startswith("W"):
            cmd = WaitCmd(lines[i][1:], comments)

        if cmd is not None:
            cmds.append(cmd)
            cmd = None

if __name__ == "__main__":
    TEST_LINES = [
        "# first step",
        "M  1,41, 9,-31, 100; -5,-16, -2,0, 75
        "W 1200"
    ]

    TEST_CMDS = []


