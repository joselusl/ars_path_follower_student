#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS

import rospy

import tf_conversions as tf


import nav_msgs.msg
from nav_msgs.msg import Path

import visualization_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker



#
import ars_lib_helpers






class ArsPathFollower:

  #######

  # Robot size radius
  robot_size_radius = 0.3

  #
  safety_distance = 0.5

  #
  flag_set_robot_hover = False


  # Output: Pose, Velocity & Robot commands References
  #
  flag_set_robot_pose_ref = False
  robot_posi_ref = None
  robot_atti_quat_simp_ref = None
  #
  flag_set_robot_velo_world_ref = False
  robot_velo_lin_world_ref = None
  robot_velo_ang_world_ref = None
  #
  flag_set_robot_velo_cmd_ref = False
  # m/s
  robot_velo_lin_cmd_ref = None
  # rad/s
  robot_velo_ang_cmd_ref = None


  # Input: Pose & Velocity Feedback
  #
  flag_set_robot_pose = False
  robot_posi = None
  robot_atti_quat_simp = None
  #
  flag_set_robot_vel_world = False
  robot_velo_lin_world = None
  robot_velo_ang_world = None


  # Input: Trajectory reference
  flag_set_robot_traj = False
  robot_traj = None
  traj_keypoint = 0


  # Input: Obstacles detected
  obstacles_detected_msg = None


  # Tol
  tol_posi = 0.1
  tol_angle = 0.1
  




  #########

  def __init__(self):

    #
    self.flag_set_robot_hover = False

    # Trajectory
    #
    self.flag_set_robot_traj = False
    self.robot_traj = []
    self.traj_keypoint = 0

    # Feedback
    #
    self.flag_set_robot_pose = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_vel_world = False
    self.robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world = np.zeros((1,), dtype=float)


    # References
    #
    self.flag_set_robot_pose_ref = False
    self.robot_posi_ref = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_velo_world_ref = False
    self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
    #
    self.flag_set_robot_velo_cmd_ref = False
    self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)


    # Tol
    self.tol_posi = 0.1
    self.tol_angle = 0.1


    # End
    return


  def setRobotPose(self, robot_posi, robot_atti_quat_simp):
    
    self.flag_set_robot_pose = True

    self.robot_posi = robot_posi
    self.robot_atti_quat_simp = robot_atti_quat_simp

    return

  def setRobotVelWorld(self, lin_vel_world, ang_vel_world):

    self.flag_set_robot_vel_world = True

    self.robot_velo_lin_world = lin_vel_world
    self.robot_velo_ang_world = ang_vel_world

    return

  def setRobotTrajectory(self, robot_traj):

    self.flag_set_robot_traj = True

    self.traj_keypoint = 0

    self.robot_traj = robot_traj

    return


  def setHoverMode(self):

    if(self.flag_set_robot_pose):

      if(self.flag_set_robot_hover == False):

        print("Path follower: Setting hover mode")

        self.flag_set_robot_pose_ref = True
        self.robot_posi_ref = self.robot_posi
        self.robot_atti_quat_simp_ref = self.robot_atti_quat_simp
        #
        self.flag_set_robot_velo_world_ref = True
        self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
        self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
        #
        self.flag_set_robot_velo_cmd_ref = True
        self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
        self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)

        self.flag_set_robot_hover = True

    #
    return


  def setMoveMode(self):

    if(self.flag_set_robot_hover == True):

      print("Path follower: Setting move mode")

      self.flag_set_robot_hover = False

    return


  def pathPlannerLoop(self, time_stamp_current):


    if(self.flag_set_robot_traj):

      if(self.flag_set_robot_pose):

        #
        if(not self.robot_traj):

          #
          self.setHoverMode()

        else:

          #
          self.setMoveMode()


          ######


          # TODO by STUDENT!!

          # Note: you can use the following helpers
          #
          # ars_lib_helpers.PoseAlgebra.computePoseSimpDifference(posi_1, atti_quat_simp_1, posi_2, atti_quat_simp_2)
          # ars_lib_helpers.PoseAlgebra.computeScalarDiffFromDiffQuatSimp(delta_atti_quat_simp)


          ######    


    return

