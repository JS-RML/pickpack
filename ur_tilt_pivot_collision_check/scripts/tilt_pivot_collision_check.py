#!/usr/bin/env python

#Author: Olivia
#Date: 2018/5/15


# Included headers
import sys
import copy
import rospy
import moveit_commander
from moveit_commander import planning_scene_interface
import moveit_msgs.msg
import geometry_msgs.msg
from tf import transformations as tf
import numpy as np
from math import *
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from arc_rotate import *
from nav_msgs.msg import Odometry
import tf_conversions



from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from std_msgs.msg import String

global gripper_command_publisher

#global motion group
group = ''
display_trajectory_trigger_pub = ''
scene = moveit_commander.PlanningSceneInterface()
scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=50)
table_pub = rospy.Publisher('/scene_object_switch', String, queue_size=50)

# Gripper command functions
def gripper_init():
  global gripper_command_publisher
  gripper_command_publisher.publish('a')
  rospy.sleep(1)

  gripper_command_publisher.publish('o')
  rospy.sleep(1)

  gripper_command_publisher.publish('c')
  rospy.sleep(1)

  gripper_command_publisher.publish('o') 
  rospy.sleep(1)

def gripper_open():
  global gripper_command_publisher
  gripper_command_publisher.publish('o')
  rospy.sleep(1)

def gripper_200():
  global gripper_command_publisher
  gripper_command_publisher.publish('200')
  rospy.sleep(1)

def gripper_open_degree(open_degree):
  global gripper_command_publisher
  gripper_command_publisher.publish(open_degree)
  rospy.sleep(1)


def gripper_close():
  global gripper_command_publisher
  gripper_command_publisher.publish('c')
  rospy.sleep(1)

#gripper command function end

def get_tool_position():
  pose_target = group.get_current_pose().pose
  return arc_get_tool_position(pose_target)


def group_move_tool(input_axis, total_angle):

  global odom_c_pub
  pose_target = group.get_current_pose().pose
  waypoints_new = calc_waypoints_tool_rotate(pose_target, input_axis, total_angle, odom_c_pub)

  # Before the execution of the real robot, turn on the display of the end effector's position and orientation 
  # display end effector's trajectory
  # subcriber of display_trajectory_trigger and corresponding function is implemented in 'display_markers.py'
  display_trajectory_trigger_pub.publish('on')
  rospy.sleep(1)

  # Utilize 'compute_cartesian_path' to get a smooth trajectory
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints_new,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  
  # Move the robot with the ARC trajectory
  group.execute(plan3)
  rospy.sleep(1)

  # Stop displaying end effector's posision and orientation
  display_trajectory_trigger_pub.publish('close')

def group_rotate_by_external_axis(center_point, axis, total_angle):
  pose_target = group.get_current_pose().pose
  waypoints_new = calc_waypoints_ARC(pose_target, center_point, axis, total_angle, odom_c_pub)
  # Before the execution of the real robot, turn on the display of the end effector's position and orientation 
  # display end effector's trajectory
  # subcriber of display_trajectory_trigger and corresponding function is implemented in 'display_markers.py'
  display_trajectory_trigger_pub.publish('on')
  rospy.sleep(1)

  # Utilize 'compute_cartesian_path' to get a smooth trajectory
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints_new,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  
  # Move the robot with the ARC trajectory
  group.execute(plan3)
  rospy.sleep(1)

  # Stop displaying end effector's posision and orientation
  display_trajectory_trigger_pub.publish('close')

def clean_scene():
  # Use the planning scene object to add or remove objects //Interface
  REFERENCE_FRAME = '/world'
  p = PlanningScene()
  p.is_diff = True    

  # Create a scene publisher to push changes to the scene //PlanningScene

  
  # Give each of the scene objects a unique name        
  Ground_id = 'ground'
  Card_id = 'card'

  # Remove leftover objects from a previous run
  scene.remove_world_object(Ground_id)
  scene.remove_world_object(Card_id)
  scene_pub.publish(p)
  rospy.sleep(1)
  table_pub.publish('off')
  rospy.sleep(2)

def add_planning_scene(card_pose):
  # Use the planning scene object to add or remove objects //Interface
  

  REFERENCE_FRAME = '/world'
  p = PlanningScene()
  p.is_diff = True    

  # Create a scene publisher to push changes to the scene //PlanningScene

  scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=50)

  # Give each of the scene objects a unique name        
  Ground_id = 'ground'
  Card_id = 'card'
  # Object2_id = 'box2'

  # Remove leftover objects from a previous run
  # scene.remove_world_object(Ground_id)
  # scene.remove_world_object(Card_id)
  # scene.remove_world_object(Object2_id)
  # scene.remove_world_object(target_id)

  # add ground into the scene
  pose_Ground = geometry_msgs.msg.PoseStamped()
  pose_Ground.header.frame_id = REFERENCE_FRAME
  pose_Ground.pose.position.x = -2.5
  pose_Ground.pose.position.y = -2.5
  pose_Ground.pose.position.z = -0.01

  scene.add_mesh(Ground_id, pose_Ground,'/home/sslrayray/table.stl')

  # add card into the scene
  pose_card = geometry_msgs.msg.PoseStamped()
  pose_card.header.frame_id = REFERENCE_FRAME
  pose_card.pose.position.x = card_pose[0]
  pose_card.pose.position.y = card_pose[1]
  pose_card.pose.position.z = card_pose[2]
  pose_card.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(card_pose[3], card_pose[4], card_pose[5]))

  scene.add_mesh(Card_id, pose_card,'/home/sslrayray/card.stl')

  scene.add_mesh(Ground_id, pose_Ground,'/home/sslrayray/table.stl')

  scene_pub.publish(p)
  rospy.sleep(1)
  table_pub.publish('on')

def move_to_current_state():
  pose_target = group.get_current_pose().pose
  group.set_pose_target(pose_target)
  plan = group.plan()
  return len(plan.joint_trajectory.points)

def check_collision():
  if move_to_current_state() == 0:
    return True
  else:
    return False

def tilt_pivot_start():
  
  global gripper_command_publisher
  global odom_pub
  global odom_c_pub

  odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
  
  # diaplay center point
  odom_c_pub = rospy.Publisher("/odom_c", Odometry, queue_size=50)

  global display_trajectory_trigger_pub
  display_trajectory_trigger_pub = rospy.Publisher(
                                      '/display_trigger',
                                      String,
                                      queue_size=20)


  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
  
  robot = moveit_commander.RobotCommander()

  global group
  group = moveit_commander.MoveGroupCommander("manipulator")

  #set max velocity and acceleration scaler
  group.set_max_velocity_scaling_factor(0.5);
  group.set_max_acceleration_scaling_factor(0.5);
  group.set_planning_time(3)

  #initialize the gripper publisher variable
  gripper_command_publisher = rospy.Publisher(
                                      'CModelRobotInputRob',
                                      String,
                                      queue_size=20)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(2)
  print "============ Starting tutorial "


  for phi in range(0, 91, 10):
    for alpha in range(0, -91, -10):
      for beta in range(0, 91, 10):
        for gamma in range(0, -91, -10):

          card_length = 0.17

          #go to camera point
          clean_scene()

          group_variable_values = [-55.77/ 180 * 3.1415926, -95.61/ 180 * 3.1415926, -72.13/ 180 * 3.1415926, 77.75/ 180 * 3.1415926, -89.71/ 180 * 3.1415926, 124.15/ 180 * 3.1415926]
          group.set_joint_value_target(group_variable_values)
          plan1 = group.plan()
          group.execute(plan1)

          # print 'go to the start point of the ARC'
          rospy.sleep(2)
          

          center_point = [-0.77998840649 + 0.5, 0.00084252595, 0.369628519 - 0.315]
          rotate_axis = [0, 0, 1] # Set the rotation axis as 'z axis'

          set_tool(-0.073, 0.005, 0.215)

          f = open('/home/sslrayray/output.txt','a')


          group_move_tool('z', alpha)
          group_move_tool('y', 8)


          pose_target = group.get_current_pose().pose

          pose_target.position.y += 0.3
          pose_target.position.z = 0.219 #0.269
          group.set_pose_target(pose_target)
          plan1 = group.plan()  
          group.execute(plan1)


          tool_pose = get_tool_position()

          external_axis_center = tool_pose
          external_axis_center[1] += card_length

          # print 'tilt center', external_axis_center
          group_rotate_by_external_axis(external_axis_center, [-1, 0, 0], phi)
          group_move_tool('y', beta)
          group_move_tool('x', gamma)
          
          final_tool_center = get_tool_position()
          # print 'final tool center', final_tool_center
          card_center = [(external_axis_center[0] + final_tool_center[0])/2, (external_axis_center[1] + final_tool_center[1])/2, (external_axis_center[2] + final_tool_center[2])/2 ]
          # print 'card center =', card_center
          # print 'angle = ', phi / 180.0 * 3.1415926
          card_pose = [final_tool_center[0] - 0.07, final_tool_center[1] - 0.001, final_tool_center[2]+0.003, 3.1415926 - phi / 180.0 * 3.1415926, 3.1415926, -3.1415926]

          add_planning_scene(card_pose)

          res = check_collision()


          if res:
            print 'Phi:',phi, ', Alpha:',alpha, ', Beta:', beta, ', Gamma:', gamma, ', Collision Result: Yes'
            string_val = str(phi) + ',' + str(alpha) + ',' + str(beta) + ',' + str(gamma) + ',' + 'Y\n' 
          else:
            print 'Phi:',phi, ', Alpha:',alpha, ', Beta:', beta, ', Gamma:', gamma, ', Collision Result: No'
            string_val = str(phi) + ',' + str(alpha) + ',' + str(beta) + ',' + str(gamma) + ',' + 'N\n' 

          f.write(string_val)
          f.close()

          raw_input()

          rospy.sleep(3)
  # Shutdown the program
  moveit_commander.roscpp_shutdown()

  print 'STOPPING'



if __name__=='__main__':
  try:
    tilt_pivot_start()
  except rospy.ROSInterruptException:
    pass

