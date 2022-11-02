#!/usr/bin/python

import os
import re
import rospy

rospy.init_node('jetson_power_mode_warning', anonymous = True)

powerReport = os.popen("nvpmodel -q").read()

if "5W" in powerReport:
    rospy.logwarn("Jetson is in 5W power mode, performance may be reduced! Use 'sudo nvpmodel -m 0' to set it to 10W mode.")
elif "MAXN" not in powerReport:
    rospy.logwarn("Unexpected output from 'nvpmodel -q'!")
