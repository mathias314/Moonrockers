#!/usr/bin/python3

import rospy
import PySimpleGUI as sg
from gamepad_msgs.msg import Sticks
from moon_msgs.msg import Pose

from baseStationGui import BaseStationGui

def sticksCallback(data: Sticks, bsg: BaseStationGui):
  bsg.updateSticks(data.lx, data.ly, data.rx, data.ry)

def poseCallback(data: Pose, bsg: BaseStationGui):
  bsg.updateRobotPosition(data.tx, data.ty, data.rz)

def gui():
  bsg = BaseStationGui()

  print('Waiting for ROS master...')
  rospy.init_node('gui', anonymous = True)
  print('Initializing subscribers...')
  rospy.Subscriber('sticks', Sticks, sticksCallback, bsg)
  rospy.Subscriber('pose', Pose, poseCallback, bsg)

  while True:
    event, values = bsg.read(1)

    if event in (sg.WIN_CLOSED, 'Exit'):
      break

# boilerplate
if __name__ == '__main__':
  try:
    gui()
  except rospy.ROSInterruptException:
    pass
