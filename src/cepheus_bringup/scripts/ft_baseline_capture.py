#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
DURATION = 60.0  # seconds to average

data = []

def cb(msg):
    w = msg.wrench
    data.append([w.force.x, w.force.y, w.force.z,
                 w.torque.x, w.torque.y, w.torque.z])

if __name__ == "__main__":
    rospy.init_node("ft_baseline_capture")
    rospy.Subscriber(TOPIC, WrenchStamped, cb)

    rospy.loginfo("Collecting FT baseline for %.1f seconds on %s", DURATION, TOPIC)
    start = rospy.Time.now()
    rate = rospy.Rate(200)

    while (rospy.Time.now() - start).to_sec() < DURATION and not rospy.is_shutdown():
        rate.sleep()

    if not data:
            rospy.logerr("No FT data collected.")
            exit(1)

    arr = np.array(data)
    mean = arr.mean(axis=0)
    std = arr.std(axis=0)

    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    print("")
    print("=== FT Baseline (mean +- std) over %.1f s ===" % DURATION)
    for i, label in enumerate(labels):
        print("%s: %.6f +- %.6f" % (label, mean[i], std[i]))

    print("")
    print("Bias command (for sensor):")
    print("b,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" %
          (mean[0], mean[1], mean[2], mean[3], mean[4], mean[5]))

