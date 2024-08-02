#!/usr/bin/env python

import numpy as np
from numpy import *


import threading

from rclpy.logging import get_logger


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers



class ArsPathFollower:

  #######

  #
  flag_set_robot_hover = True


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
  robot_traj_waypoint_idx = 0


  # Timer
  timer_reach_waypoint = None


  # Tol
  tol_posi = 0.0
  tol_angle = 0.0
  

  # ROS2 logger
  logger = None



  #########

  def __init__(self):

    #
    self.flag_set_robot_hover = True

    # Trajectory
    #
    self.flag_set_robot_traj = False
    self.robot_traj = []
    self.robot_traj_waypoint_idx = 0

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


    # Timer
    self.timer_reach_waypoint = None


    # Tol
    self.tol_posi = 0.0
    self.tol_angle = 0.0

    #
    self.logger = get_logger('ars_path_follower')


    # End
    return


  def setConfigParameters(self, config_param):

    # Tol
    self.tol_posi = config_param['tol_posi']
    self.tol_angle = config_param['tol_angle']

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

    self.logger.info("New Trajectory set!")

    self.flag_set_robot_traj = True

    self.robot_traj_waypoint_idx = -1

    self.robot_traj = robot_traj

    # Cancel previous timer (if any)
    if(self.timer_reach_waypoint):
      self.timer_reach_waypoint.cancel()

    return


  def setHoverMode(self):

    if(self.flag_set_robot_pose):

      if(self.flag_set_robot_hover == False):

        self.logger.info("New Trajectory set!")

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

      self.logger.info("New Trajectory set!")

      self.flag_set_robot_hover = False

    return


  def timeoutReachRobotTrajWaypoint(self):
    self.logger.info("Waypoint " + str(self.robot_traj_waypoint_idx) + " Timed Out")
    self.advanceRobotTrajWaypoint()
    return


  def advanceRobotTrajWaypoint(self):
    if( self.robot_traj_waypoint_idx < len(self.robot_traj) ):
      
      # Cancel previous timer (if any)
      if(self.timer_reach_waypoint):
        self.timer_reach_waypoint.cancel()

      # Advance waypoint
      self.robot_traj_waypoint_idx+=1
      if(self.robot_traj_waypoint_idx < len(self.robot_traj)):
        self.logger.info("Waypoint " + str(self.robot_traj_waypoint_idx) + " Next")

      # Set timeout timer
      ###### TODO By student
      # Use: 
      # self.robot_traj_waypoint_idx, self.robot_traj
      # ars_lib_helpers.PoseAlgebra.computePoseSimpDifference()
      # ars_lib_helpers.PoseAlgebra.computeScalarDiffFromDiffQuatSimp()
      # self.timer_reach_waypoint
      # self.timeoutReachRobotTrajWaypoint()
      # threading.Timer()



      ###########

    return


  def pathFollowerLoop(self, time_stamp_current):

    if(self.flag_set_robot_traj):

      if(self.flag_set_robot_pose):

        #
        if(not self.robot_traj):

          # Empty trajectory
          self.setHoverMode()

        else:

          # Trajectory is not empty
          self.setMoveMode()

          # Trajectory has not started
          while(self.robot_traj_waypoint_idx<0):
            self.advanceRobotTrajWaypoint()


          # Check if waypoint has been visited and advance to the next waypoint

          ###### TODO By student
          # Use: 
          # self.robot_traj_waypoint_idx, self.robot_traj
          # ars_lib_helpers.PoseAlgebra.computePoseSimpDifference()
          # ars_lib_helpers.PoseAlgebra.computeScalarDiffFromDiffQuatSimp()
          # self.tol_posi, self.tol_angle
          # self.advanceRobotTrajWaypoint()



          ###########

          # Set ctr cmd
          self.setCmdRef()     

    return


  def setCmdRef(self):

    if(self.robot_traj_waypoint_idx < len(self.robot_traj)):
      self.flag_set_robot_pose_ref = True
      self.robot_posi_ref = self.robot_traj[self.robot_traj_waypoint_idx].position
      self.robot_atti_quat_simp_ref = self.robot_traj[self.robot_traj_waypoint_idx].attitude_quat_simp
      #
      self.flag_set_robot_velo_world_ref = True
      self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
      self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
      #
      self.flag_set_robot_velo_cmd_ref = True
      self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
      self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)

    #
    return
