#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import serial

# Adjust this if needed (e.g. "/dev/ttyUSB0" or "/dev/serial/by-id/...")
PORT = "/dev/my_ft_sensor"
BAUD = 460800

# Your measured baseline bias from the ROS script:
BIAS_VALUES = (-9.928801,5.918258,2.375053,0.008588,-0.183896,0.019959)
BIAS_CMD = "b,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % BIAS_VALUES

def read_some(ser, label):
    time.sleep(0.1)
    buf = ser.read(ser.in_waiting or 1)
    print("---- %s ----" % label)
    if buf:
        try:
            print(buf)
        except:
            print("(binary data)")
    else:
        print("(no data)")

if __name__ == "__main__":
    print("Opening %s @ %d" % (PORT, BAUD))
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    # We are probably in RUN mode, streaming.
    # 1) Enter CONFIG mode with single-character 'C'
    ser.write(b"C")
    ser.flush()
    read_some(ser, "After C (CONFIG)")

    # 2) Print current config (should include current bias)
    ser.write(b"w")
    ser.flush()
    read_some(ser, "Before bias (w)")

    # 3) Send bias command as ASCII, no newline required per manual
    print("Sending bias command: %s" % BIAS_CMD)
    ser.write(BIAS_CMD.encode("ascii"))
    ser.flush()
    read_some(ser, "After bias (b,...)")

    # 4) Save to NVM: 's'
    ser.write(b"s")
    ser.flush()
    read_some(ser, "After save (s)")

    # 5) Back to RUN mode: 'R'
    ser.write(b"R")
    ser.flush()
    read_some(ser, "After R (RUN)")

    ser.close()
    print("Done.")

