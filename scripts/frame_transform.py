#!/usr/bin/env python  
import roslib
import numpy
import rospy
import tf
from math import pi

if __name__ == '__main__':
    rospy.init_node('frame_establish')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

    xaxis = (1, 0, 0)
    yaxis = (0, 1, 0)
    zaxis = (0, 0, 1)
    Ry = tf.transformations.rotation_matrix(pi/2, yaxis)
    Rx = tf.transformations.rotation_matrix(pi, xaxis)
    euler = tf.transformations.euler_from_matrix(Ry, 'sxyz')
    quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2], axes='sxyz')

    while not rospy.is_shutdown():
	br.sendTransform((0.090, -0.16375, -0.002),
			 (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                         rospy.Time.now(),
                         "camera_link",
                         "ee_link")
        br.sendTransform((0.275, -0.0225, 0),
			 (0, 0, 0, 1),
                         rospy.Time.now(),
                         "left_fgtip",
                         "ee_link")
        br.sendTransform((0.275, 0.0225, 0),
			 (0, 0, 0, 1),
                         rospy.Time.now(),
                         "right_fgtip",
                         "ee_link")
        rate.sleep()
