
import serial
import time

port1 = "/dev/cu.usbmodem145301"
port2 = "/dev/cu.usbmodem145401"
speed = 115200

def init(port):
    s = serial.Serial(port, speed)

    # Wake up grbl
    s.write("\r\n\r\n".encode())
    time.sleep(2)   # Wait for grbl to initialize
    s.flushInput()  # Flush startup text in serial input

    # Go through homing
    s.write("$H\n".encode())

    return s

s1 = init(port1)
time.sleep(30)   # Wait for grbl to do homing

s2 = init(port2)
time.sleep(45)   # Wait for grbl to do homing

def send(cmd1, cmd2):
    s1.write(("%s\n" % cmd1).encode())
    s2.write(("%s\n" % cmd2).encode())
    time.sleep(10)

send("X1 Y-41 Z9 A-31", "X-5 Y-16 Z-2 A0")
send("X-27 Y-19 Z9 A-31", "X-12 Y-7 Z-4 A-9")
send("X21 Y36 Z-27 A-47", "X2 Y14 Z-11 A8")

send("X0 Y0 Z0 A0", "X0 Y0 Z0 A0")
