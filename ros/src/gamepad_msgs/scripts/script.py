#!/usr/bin/python3

import rospy
from gamepad_msgs.msg import Sticks

def script():
  s = Sticks(1, 2, 3, 4)
  print(s)

if __name__ == '__main__':
  script()
