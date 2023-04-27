#!/usr/bin/python3

import rospy
import PySimpleGUI as sg
from gamepad_msgs.msg import Sticks
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

from baseStationGui import BaseStationGui

def poseCallback(data: Pose, bsg: BaseStationGui):
  bsg.updateRobotPosition(data.position.z * 100, data.position.x * 100, data.orientation.z)

def gui():
  bsg = BaseStationGui()

  print('Waiting for ROS master...')
  rospy.init_node('gui', anonymous = True)
  print('Initializing subscribers...')
  rospy.Subscriber('pose', Pose, poseCallback, bsg)
  distancePub = rospy.Publisher('targetDistance', Float32, queue_size=100)

  while True:
    event, values = bsg.read(10)

    if (values['checkEnableAutodrive']):
        distancePub.publish(values['sliderDistance'] / 100)

    if event in (sg.WIN_CLOSED, 'Exit'):
      break

# boilerplate
if __name__ == '__main__':
  try:
    gui()
  except rospy.ROSInterruptException:
    pass
