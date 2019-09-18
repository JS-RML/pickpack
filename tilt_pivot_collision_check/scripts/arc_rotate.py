import numpy as np
from math import *
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import copy
from tf import transformations as tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
import rospy

R_h_ee_tool = np.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')

def set_tool(tool_x, tool_y, tool_z):
  global R_h_ee_tool

  R_h_ee_tool[0, 3] = tool_x
  R_h_ee_tool[1, 3] = tool_y
  R_h_ee_tool[2, 3] = tool_z

def calc_waypoints_ARC(init_pose, center_point, axis, total_angle, odom_c_pub):
  
  if total_angle >= 0:
    direction = 1
  else:
    direction = -1

  way_points = []
  
  #first way point is the start point  
  way_points.append(copy.deepcopy(init_pose));
  
  #calculate the rotation matrix about input 'axis' for 5 degree
  step_angle = direction * 2.0 / 180 * 3.1415926
  current_angle = step_angle
  total_angle = total_angle / 180.0 * 3.1415926

  v_init = np.cross(center_point, axis)
  w_init = axis
  R_h_transform = calc_R_h_from_ksi(w_init, v_init, step_angle);

  #calcluate the matrix form of start point's position and orientation
  current_R_h = get_R_h_from_pose(init_pose);
  
  while (direction * current_angle <= direction * total_angle):
    #calculate the matrix form of next point after rotating about axis for 5 degree
    current_R_h = R_h_transform * current_R_h 

    #get position and orientation variable from matrix form
    new_target = get_pose_from_R_h(current_R_h)

    #add new waypoint to the way_points list
    way_points.append(new_target)

    #update current rotation angle 
    current_angle += step_angle


  display_axis(center_point, axis, odom_c_pub)

  # output is the waypoint list of the ARC trajectory  
  return way_points


def arc_get_tool_position(init_pose):
    #calculate center_point
  Frame_base_h = np.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')

  #calcluate the matrix form of start point's position and orientation
  current_R_h = get_R_h_from_pose(init_pose);

  Frame_tool_h = current_R_h * R_h_ee_tool * Frame_base_h

  center_point = [Frame_tool_h[0, 3], Frame_tool_h[1, 3], Frame_tool_h[2, 3]]

  return center_point

def calc_waypoints_tool_rotate(init_pose, input_axis, total_angle, odom_c_pub):
  global R_h_ee_tool

  if total_angle >= 0:
    direction = 1
  else:
    direction = -1

  way_points = []
  
  #first way point is the start point  
  way_points.append(copy.deepcopy(init_pose));
  

  #calculate center_point
  Frame_base_h = np.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')

  #calcluate the matrix form of start point's position and orientation
  current_R_h = get_R_h_from_pose(init_pose);

  Frame_tool_h = current_R_h * R_h_ee_tool * Frame_base_h

  center_point = [Frame_tool_h[0, 3], Frame_tool_h[1, 3], Frame_tool_h[2, 3]]


  axis = []

  if input_axis == 'x':
    axis = [Frame_tool_h[0, 0], Frame_tool_h[1, 0], Frame_tool_h[2, 0]]
  elif input_axis == 'y':
    axis = [Frame_tool_h[0, 1], Frame_tool_h[1, 1], Frame_tool_h[2, 1]]
  elif input_axis == 'z':
    axis = [Frame_tool_h[0, 2], Frame_tool_h[1, 2], Frame_tool_h[2, 2]]
  else:
    print 'Error! input axis should be x, y, z'
    return way_points

  display_axis(center_point, axis, odom_c_pub)

  #calculate the rotation matrix about input 'axis' for 2 degree
  step_angle = direction * 2.0 / 180 * 3.1415926
  current_angle = step_angle
  total_angle = total_angle / 180.0 * 3.1415926

  v_init = np.cross(center_point, axis)
  w_init = axis
  R_h_transform = calc_R_h_from_ksi(w_init, v_init, step_angle);

  #calcluate the matrix form of start point's position and orientation
  current_R_h = get_R_h_from_pose(init_pose);
  
  while (direction * current_angle <= direction * total_angle):
    #calculate the matrix form of next point after rotating about axis for 5 degree
    current_R_h = R_h_transform * current_R_h 

    #get position and orientation variable from matrix form
    new_target = get_pose_from_R_h(current_R_h)

    #add new waypoint to the way_points list
    way_points.append(new_target)

    #update current rotation angle 
    current_angle += step_angle

  # output is the waypoint list of the ARC trajectory  


  # display_axis(center_point, axis, odom_c_pub)
  rospy.sleep(0.5)

  return way_points



#solve rotation matrix using the formula to solve e^(ksi * theta)
def calc_R_h_from_ksi(w_init, v_init, theta):
  
  #initialize variables
  w_cross_v_m = np.matrix('0.0; 0.0; 0.0')
  R_ksi_1st = np.matrix('0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0')
  R_w_1st = np.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')
  R_w_hat = np.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')
  R_I = np.matrix('1.0 0.0 0.0; 0.0 1.0 0.0; 0 0 1')
  w_init_v = np.array([w_init[0], w_init[1], w_init[2]])
  v_init_v = np.array([v_init[0], v_init[1], v_init[2]])
  w_cross_v = np.cross(w_init_v, v_init_v)  
  w_cross_v_m[0, 0] = w_cross_v[0]
  w_cross_v_m[1, 0] = w_cross_v[1]
  w_cross_v_m[2, 0] = w_cross_v[2]
  
  # Solve the rotation matrix part
  theta_1st = theta
  R_w_hat = np.matrix([[0,             -w_init[2],  w_init[1]], [w_init[2],  0,              -w_init[0]], [-w_init[1], w_init[0],   0] ])
  R_w_1st = R_I + R_w_hat * sin(theta_1st) + R_w_hat * R_w_hat * (1 - cos(theta_1st));
  
  R_ksi_1st[0, 0] =  R_w_1st[0, 0]
  R_ksi_1st[0, 1] =  R_w_1st[0, 1]
  R_ksi_1st[0, 2] =  R_w_1st[0, 2]

  R_ksi_1st[1, 0] =  R_w_1st[1, 0]
  R_ksi_1st[1, 1] =  R_w_1st[1, 1]
  R_ksi_1st[1, 2] =  R_w_1st[1, 2]

  R_ksi_1st[2, 0] =  R_w_1st[2, 0]
  R_ksi_1st[2, 1] =  R_w_1st[2, 1]
  R_ksi_1st[2, 2] =  R_w_1st[2, 2]
  
  # Solve the translation part
  temp = (R_I - R_w_1st) * w_cross_v_m;
  R_ksi_1st[0, 3] = temp[0, 0]
  R_ksi_1st[1, 3] = temp[1, 0]
  R_ksi_1st[2, 3] = temp[2, 0]
  R_ksi_1st[3, 3] = 1

  # output is the trasformation matrix
  return R_ksi_1st


# get position and orientation variables from a transformation matrix
def get_pose_from_R_h(R_h):
  new_q = tf.quaternion_from_matrix(R_h)
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = new_q[0]
  pose_target.orientation.y = new_q[1]
  pose_target.orientation.z = new_q[2]
  pose_target.orientation.w = new_q[3]
  pose_target.position.x = R_h[0, 3]
  pose_target.position.y = R_h[1, 3]
  pose_target.position.z = R_h[2, 3]
  return pose_target

# get transformation matrix from position and orientation variables 
def get_R_h_from_pose(pose):
  init_q = []
  init_q.append(pose.orientation.x)
  init_q.append(pose.orientation.y)
  init_q.append(pose.orientation.z)
  init_q.append(pose.orientation.w)
  init_R_h = tf.quaternion_matrix(init_q)
  init_R_h[0, 3] = pose.position.x
  init_R_h[1, 3] = pose.position.y 
  init_R_h[2, 3] = pose.position.z
  return init_R_h

# display the axis, the end of the arrow is the position, the arrow direction is the x axis of the orientation 
def display_axis(center_point, axis, odom_c_pub):

  #to calculate orientation from input 'axis'
  #we have to set the other two axis of the coordinate frame
  
  # one possible solution for y axis
  v_init = np.cross(center_point, axis)  
  len_x_z = (v_init[0]**2 + v_init[1]**2 + v_init[2]**2)**0.5
  center_y_axis = [v_init[0]/len_x_z, v_init[1]/len_x_z, v_init[2]/len_x_z]

  # z axis = x axis cross product y axis
  center_z_axis = np.cross(axis, center_y_axis)

  #calculate the matrix form of coordinate frame
  R_h_z = np.matrix('0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 1.0')
  R_h_z[0, 0] = axis[0]
  R_h_z[1, 0] = axis[1]
  R_h_z[2, 0] = axis[2]

  R_h_z[0, 2] = center_z_axis[0]
  R_h_z[1, 2] = center_z_axis[1]
  R_h_z[2, 2] = center_z_axis[2]

  R_h_z[0, 1] = center_y_axis[0]
  R_h_z[1, 1] = center_y_axis[1]
  R_h_z[2, 1] = center_y_axis[2]

  #get position and orientation from the matrix form of the rotation axis
  center_pose = get_pose_from_R_h(R_h_z)

  #display it in rviz
  odom = Odometry()
  odom.header.stamp = rospy.Time.now()
  odom.header.frame_id = "world"
  odom.pose.pose.orientation = center_pose.orientation
  odom.pose.pose.position.x = center_point[0]
  odom.pose.pose.position.y = center_point[1]
  odom.pose.pose.position.z = center_point[2]

  odom_c_pub.publish(odom)
  