#!/usr/bin/python3

import gamepad_lib as Gamepad
import rospy
import typing
import time

from copy import deepcopy
from gamepad_msgs.msg import Sticks
from std_msgs.msg import Float32

gamepadType = Gamepad.Gamepad


class DiffTracker:
    # numItems: number of items we're going to track the difference in
    # delta: mininum change in any one item for self.hasChanged() to return True
    def __init__(self, numItems: int, delta: float):
        self.numItems = numItems
        self.lastVals = [0] * numItems
        self.delta = delta
        self.lastTime = 0
        self.timeout = 0.1

    # returns True if one of the values has changed by more than self.delta
    def hasChanged(self, newVals: typing.Iterable):
        # trigger an exception if newVals isn't the right size
        assert len(newVals) == self.numItems

        changed = False

        # check all pairs of elements
        for i in range(self.numItems):
            if abs(self.lastVals[i] - newVals[i]) > self.delta:
                changed = True

        # return True and keep track of the newest vals if they've changed enough
        if changed:
            for i in range(self.numItems):
                self.lastVals[i] = deepcopy(newVals[i])
            return True

        if time.time() - self.lastTime > self.timeout:
            self.lastTime = time.time()
            return True

        return False


class JoystickScaler:
    def __init__(self, scaleFactor):
        self.scaleFactor = scaleFactor

    def scale(self, values: typing.Iterable):
        scaledValues = []
        for value in values:
            scaledValues.append(pow(value, 3) * self.scaleFactor)

        return scaledValues


def joysticks():
    # Wait for a connection
    if not Gamepad.available():
        rospy.logwarn("Please connect your gamepad...")
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print("Gamepad connected")

    # set up the DiffTracker class
    diff = DiffTracker(4, 0.01)

    # set up the JoystickScaler class
    scaler = JoystickScaler(1)

    # initialize ROS stuff
    sticks = rospy.Publisher("sticks", Sticks, queue_size=100)
    collectionChainDrive = rospy.Publisher("chain_drive_power", Float32, queue_size=100)
    collectionPlunge = rospy.Publisher(
        "collection_plunge_power", Float32, queue_size=100
    )
    bucket = rospy.Publisher("bucket_power", Float32, queue_size=100)
    rospy.init_node("gamepad", anonymous=True)
    rate = rospy.Rate(20)  # 20 Hz, joystick update rate

    gamepad.startBackgroundUpdates()

    chainDriveMult = 0.3
    bucketMult = 0.2
    collectionPlungeMult = 0.5
    r2LastState = False
    r1LastState = False
    l2LastState = False
    l1LastState = False
    hatyLastState = 0

    try:
        while not rospy.is_shutdown() and gamepad.isConnected():
            lx = gamepad.axis(0)
            ly = gamepad.axis(1)
            rx = gamepad.axis(3)
            ry = gamepad.axis(4)
            stickValues = (lx, -ly, rx, -ry)
            haty = gamepad.axis(7)

            # only publish a message if the sticks have moved a bit
            if diff.hasChanged(stickValues):
                scaled = scaler.scale(stickValues)
                sticksMsg = Sticks(scaled[0], scaled[1], scaled[2], scaled[3])
                sticks.publish(sticksMsg)

            r2State = gamepad.axis(5) > 0
            l2State = gamepad.axis(2) > 0
            r1State = gamepad.isPressed(5)
            l1State = gamepad.isPressed(4)

            if r2State != r2LastState:
                collectionChainDrive.publish(chainDriveMult * r2State)
                r2LastState = r2State
            if l2State != l2LastState:
                collectionChainDrive.publish(-chainDriveMult * l2State)
                l2LastState = l2State
            if r1State != r1LastState:
                collectionPlunge.publish(collectionPlungeMult * r1State)
                r1LastState = r1State
            if l1State != l1LastState:
                collectionPlunge.publish(-collectionPlungeMult * l1State)
                l1LastState = l1State
            if hatyLastState != haty:
                bucket.publish(-haty * bucketMult)
                hatyLastState = haty

            rate.sleep()
    finally:
        gamepad.disconnect()


if __name__ == "__main__":
    try:
        joysticks()
    except rospy.ROSInterruptException:
        pass
