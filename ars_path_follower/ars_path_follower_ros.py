#!/usr/bin/env python

import numpy as np
from numpy import *

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import nav_msgs.msg
from nav_msgs.msg import Path


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers

#
from ars_path_follower.ars_path_follower import *



class ArsPathFollowerRos(Node):

  #######

  # World frame
  world_frame = 'world'

  # Ctr command loop freq 
  # time step
  ctr_cmd_loop_freq = 10.0
  # Timer
  ctr_cmd_loop_timer = None

  # Robot traj subscriber
  robot_traj_sub = None

  # Robot pose subscriber
  robot_pose_sub = None
  # Robot velocity subscriber
  robot_vel_world_sub = None

  # Robot pose ref pub
  robot_pose_ref_pub = None
  # Robot velocity ref pub
  robot_vel_world_ref_pub = None
  #
  robot_vel_cmd_ref_pub = None

  #
  config_param = None

  # Path follower
  path_follower = ArsPathFollower()
  


  #########

  def __init__(self, node_name='ars_path_follower_node'):
    # Init ROS
    super().__init__(node_name)

    # Motion controller
    self.path_follower = ArsPathFollower()

    #
    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_path_follower_node'):

    # Package path
    try:
      pkg_path = get_package_share_directory('ars_path_follower')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")
    

    #### READING PARAMETERS ###
    
    # Config param
    default_config_param_yaml_file_name = os.path.join(pkg_path,'config','config_path_follower.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('config_param_path_follower_yaml_file', default_config_param_yaml_file_name)
    # Get the parameter value
    config_param_yaml_file_name_str = self.get_parameter('config_param_path_follower_yaml_file').get_parameter_value().string_value
    self.get_logger().info(config_param_yaml_file_name_str)
    self.config_param_yaml_file_name = os.path.abspath(config_param_yaml_file_name_str)

    ###


    # Load config param
    with open(self.config_param_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.config_param = yaml.load(file, Loader=SafeLoader)['path_follower']

    if(self.config_param is None):
      self.get_logger().info("Error loading config param path follower")
    else:
      self.get_logger().info("Config param path follower:")
      self.get_logger().info(str(self.config_param))


    # Parameters
    #
    self.world_frame = self.config_param['world_frame']
    #
    self.ctr_cmd_loop_freq = self.config_param['ctr_cmd_loop_freq']
    
    #
    self.path_follower.setConfigParameters(self.config_param['algorithm'])

    
    # End
    return


  def open(self):

    # Subscribers

    #
    self.robot_traj_sub = self.create_subscription(Path, 'robot_trajectory_ref', self.robotTrajectoryCallback, qos_profile=10)

    # 
    self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.robotPoseCallback, qos_profile=10)
    #
    self.robot_vel_world_sub = self.create_subscription(TwistStamped, 'robot_velocity_world', self.robotVelWorldCallback, qos_profile=10)
    


    # Publishers

    # 
    self.robot_pose_ref_pub = self.create_publisher(PoseStamped, 'robot_pose_ctr_ref', qos_profile=10)
    #
    self.robot_vel_world_ref_pub = self.create_publisher(TwistStamped, 'robot_velocity_world_ctr_ref', qos_profile=10)
    #
    self.robot_vel_cmd_ref_pub = self.create_publisher(TwistStamped, 'robot_ctr_cmd_ref', qos_profile=10)




    # Timers
    #
    self.ctr_cmd_loop_timer = self.create_timer(1.0/self.ctr_cmd_loop_freq, self.ctrCommandLoopTimerCallback)


    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def robotTrajectoryCallback(self, robot_trajectory_msg):

    # 
    robot_trajectory = []

    #
    for robot_trajectory_pose_i_msg in robot_trajectory_msg.poses:

      robot_trajectory_pose_i = ars_lib_helpers.PoseSimp()

      robot_trajectory_pose_i.position[0] = robot_trajectory_pose_i_msg.pose.position.x
      robot_trajectory_pose_i.position[1] = robot_trajectory_pose_i_msg.pose.position.y
      robot_trajectory_pose_i.position[2] = robot_trajectory_pose_i_msg.pose.position.z

      quat_i = ars_lib_helpers.Quaternion.zerosQuat()
      quat_i[0] = robot_trajectory_pose_i_msg.pose.orientation.w
      quat_i[1] = robot_trajectory_pose_i_msg.pose.orientation.x
      quat_i[2] = robot_trajectory_pose_i_msg.pose.orientation.y
      quat_i[3] = robot_trajectory_pose_i_msg.pose.orientation.z

      quat_i = ars_lib_helpers.Quaternion.normalize(quat_i)

      robot_trajectory_pose_i.attitude_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(quat_i)

      robot_trajectory.append(robot_trajectory_pose_i)

    #
    self.path_follower.setRobotTrajectory(robot_trajectory)


    # End
    return


  def robotPoseCallback(self, robot_pose_msg):

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_pose_msg.pose.position.x
    robot_posi[1] = robot_pose_msg.pose.position.y
    robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.path_follower.setRobotPose(robot_posi, robot_atti_quat_simp)

    #
    return


  def robotVelWorldCallback(self, robot_vel_msg):

    # Linear
    lin_vel_world = np.zeros((3,), dtype=float)
    lin_vel_world[0] = robot_vel_msg.twist.linear.x
    lin_vel_world[1] = robot_vel_msg.twist.linear.y
    lin_vel_world[2] = robot_vel_msg.twist.linear.z

    # Angular
    ang_vel_world = np.zeros((1,), dtype=float)
    ang_vel_world[0] = robot_vel_msg.twist.angular.z

    #
    self.path_follower.setRobotVelWorld(lin_vel_world, ang_vel_world)

    #
    return


  def robotPoseRefPublish(self, time_stamp_current):

    robot_posi_ref = self.path_follower.robot_posi_ref
    robot_atti_quat_simp_ref = self.path_follower.robot_atti_quat_simp_ref

    robot_pose_ref_msg = PoseStamped()

    robot_pose_ref_msg.header.stamp = time_stamp_current.to_msg()
    robot_pose_ref_msg.header.frame_id = self.world_frame

    robot_pose_ref_msg.pose.position.x = robot_posi_ref[0]
    robot_pose_ref_msg.pose.position.y = robot_posi_ref[1]
    robot_pose_ref_msg.pose.position.z = robot_posi_ref[2]

    robot_pose_ref_msg.pose.orientation.w = robot_atti_quat_simp_ref[0]
    robot_pose_ref_msg.pose.orientation.x = 0.0
    robot_pose_ref_msg.pose.orientation.y = 0.0
    robot_pose_ref_msg.pose.orientation.z = robot_atti_quat_simp_ref[1]


    self.robot_pose_ref_pub.publish(robot_pose_ref_msg)


    return


  def robotVelocityRefPublish(self, time_stamp_current):

    robot_velo_lin_world_ref = self.path_follower.robot_velo_lin_world_ref
    robot_velo_ang_world_ref = self.path_follower.robot_velo_ang_world_ref

    robot_velo_ref_msg = TwistStamped()

    robot_velo_ref_msg.header.stamp = time_stamp_current.to_msg()
    robot_velo_ref_msg.header.frame_id = self.world_frame

    robot_velo_ref_msg.twist.linear.x = robot_velo_lin_world_ref[0]
    robot_velo_ref_msg.twist.linear.y = robot_velo_lin_world_ref[1]
    robot_velo_ref_msg.twist.linear.z = robot_velo_lin_world_ref[2]

    robot_velo_ref_msg.twist.angular.x = 0.0
    robot_velo_ref_msg.twist.angular.y = 0.0
    robot_velo_ref_msg.twist.angular.z = robot_velo_ang_world_ref[0]


    self.robot_vel_world_ref_pub.publish(robot_velo_ref_msg)


    return


  def robotVelCmdRefPublish(self, time_stamp_current):

    robot_velo_lin_cmd_ref = self.path_follower.robot_velo_lin_cmd_ref
    robot_velo_ang_cmd_ref = self.path_follower.robot_velo_ang_cmd_ref

    robot_velo_cmd_ref_msg = TwistStamped()

    robot_velo_cmd_ref_msg.header.stamp = time_stamp_current.to_msg()
    robot_velo_cmd_ref_msg.header.frame_id = self.world_frame

    robot_velo_cmd_ref_msg.twist.linear.x = robot_velo_lin_cmd_ref[0]
    robot_velo_cmd_ref_msg.twist.linear.y = robot_velo_lin_cmd_ref[1]
    robot_velo_cmd_ref_msg.twist.linear.z = robot_velo_lin_cmd_ref[2]

    robot_velo_cmd_ref_msg.twist.angular.x = 0.0
    robot_velo_cmd_ref_msg.twist.angular.y = 0.0
    robot_velo_cmd_ref_msg.twist.angular.z = robot_velo_ang_cmd_ref[0]


    self.robot_vel_cmd_ref_pub.publish(robot_velo_cmd_ref_msg)


    return
  

  def ctrCommandLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    #
    self.path_follower.pathFollowerLoop(time_stamp_current)

    # Publish
    if(self.path_follower.flag_set_robot_pose_ref):
      self.robotPoseRefPublish(time_stamp_current)

    if(self.path_follower.flag_set_robot_velo_world_ref):
      self.robotVelocityRefPublish(time_stamp_current)

    if(self.path_follower.flag_set_robot_velo_cmd_ref):
      self.robotVelCmdRefPublish(time_stamp_current)
    
    # End
    return

  