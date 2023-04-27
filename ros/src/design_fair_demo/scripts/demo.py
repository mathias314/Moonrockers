#!/usr/bin/python3

import rospy
import math
import time

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

left_power = None
right_power = None
autoDriveLeft = None
autoDriveRight = None
targetDist = 1
deadzone = 0.5
rotationDeadzone = 0.5
lastTargetDistTime = 0
autodriveTimeout = 0.125  # seconds


def poseCallback(data):
    global targetDist
    global lastTargetDistTime

    if time.time() - lastTargetDistTime < autodriveTimeout:
        Tvec = (data.position.x, data.position.y, data.position.z)
        dist = math.sqrt(Tvec[0] ** 2 + Tvec[1] ** 2 + Tvec[2] ** 2)

        if Tvec[0] > rotationDeadzone:
            autoDriveLeft.publish(0.75)
            autoDriveRight.publish(-0.2)
        elif Tvec[0] < -rotationDeadzone:
            autoDriveLeft.publish(-0.75)
            autoDriveRight.publish(0.2)
        elif dist > targetDist + deadzone:
            autoDriveLeft.publish(-0.75)
            autoDriveRight.publish(-0.2)
        elif dist < targetDist - deadzone:
            autoDriveLeft.publish(0.75)
            autoDriveRight.publish(0.2)
        else:
            autoDriveLeft.publish(0)
            autoDriveRight.publish(0)


def targetDistanceCallback(data):
    global targetDist
    global lastTargetDistTime
    lastTargetDistTime = time.time()
    targetDist = data.data


def demo():
    # initialize ROS stuff
    global left_power
    global right_power
    global autoDriveLeft
    global autoDriveRight
    rospy.init_node("demo", anonymous=True)
    autoDriveLeft = rospy.Publisher("autoDriveLeft", Float32, queue_size=100)
    autoDriveRight = rospy.Publisher("autoDriveRight", Float32, queue_size=100)
    rospy.Subscriber("pose", Pose, poseCallback)
    rospy.Subscriber("targetDistance", Float32, targetDistanceCallback)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        demo()
    except rospy.ROSInterruptException:
        print("exception caught in vision demo file")
        pass
