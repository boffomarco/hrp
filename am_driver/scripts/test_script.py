#! /usr/bin/env python
import os
import select
import sys
import termios
import threading
import tty
import numpy as np
import rospy
from am_driver.msg import BatteryStatus, Mode, SensorStatus
from geometry_msgs.msg import Twist
#from std_msgs.msg import UInt16


def goodbye():
    twist = Twist()
    print(0)
    twist.linear.x = 0
    twist.angular.z = 0
    rospy.Publisher('cmd_vel', Twist, queue_size=1).publish(twist)
    print('Shutting down')

class Mower(object):

  def init(self):
    # Initial variables 
    rospy.on_shutdown(goodbye)
    self.update_rate = 1  # Hz
    # Setup publishers
    self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

  def run(self):
    try:
      self.init()
      r = rospy.Rate(self.update_rate) # Hz
      speed = 0.6
      angle = 0
      i = 0
      j = 0
      while (j < 8 and not rospy.is_shutdown()):
        self.update(speed,angle)
        r.sleep()
        if i == 8:
          angle = 0.8
          speed = 0.6
        if i == 10:
          angle = 0
          speed = 0.0
          i = -1
          j += 1
        i+=1
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.update(0,0)


  def update(self,speed,angle):
    if rospy.is_shutdown():
      return 
    twist = Twist()
    #cmd  = self.speed*self.command
    print(speed)
    twist.linear.x = speed
    twist.angular.z = angle

    self.pub_twist.publish(twist)



if __name__ == '__main__':
  rospy.init_node('test_node')
  mower = Mower()
  mower.run()
