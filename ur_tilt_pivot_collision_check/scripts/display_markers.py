#!/usr/bin/env python

#Author: Olivia
#Date: 2018/5/15


# Included headers
import sys
import copy
import rospy
import geometry_msgs.msg
import tf
import numpy as np
from math import *
from std_msgs.msg import String

from nav_msgs.msg import Odometry

global display_flag

#if this node receive 'on' signal from robot manipulation node
#the flag will be set as 1
def trigger_callback(data):
  global display_flag
  if data.data == 'on':
    display_flag = 1
  else:
    display_flag = 0


def display_markers():
  global display_flag

  #listener to capture the end effector's position and orientation of
  listener = tf.TransformListener()
  #publisher to publish the end effector's position and orientation information 
  odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
  
  #subscriber to capture the trigger signal from robot manipulation node
  trigger_sub = rospy.Subscriber('/display_trigger', String, trigger_callback)
  
  #flag initialized as 0
  display_flag = 0
  rate = rospy.Rate(10.0)

  while not rospy.is_shutdown():
    #if the flag is set as 1, the position and orientation of current end effector will be displayed in RVIZ
    if (display_flag == 1):  
      try:
        (position, orientation) = listener.lookupTransform('world', 'ee_link', rospy.Time())
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]

        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]

        odom_pub.publish(odom)
        rospy.sleep(0.1)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      rate.sleep()

if __name__=='__main__':
  
  rospy.init_node('display_markers')
  display_markers()
  print 'end'
