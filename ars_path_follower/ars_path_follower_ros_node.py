#!/usr/bin/env python3

import rclpy

from ars_path_follower.ars_path_follower_ros import ArsPathFollowerRos


def main(args=None):

  rclpy.init(args=args)

  ars_path_follower_ros = ArsPathFollowerRos()

  ars_path_follower_ros.open()

  try:
      ars_path_follower_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_path_follower_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()
