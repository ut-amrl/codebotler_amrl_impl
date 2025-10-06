import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import yaml
from std_msgs.msg import String, Bool
import cv2
import numpy as np
import time
import torch
import os
import sys
from PIL import Image
import shutil
import signal
import threading
import subprocess

# External message types
from amrl_msgs.msg import NavStatusMsg, Localization2DMsg
from cobot_codebotler_actions.action import GoTo, GetCurrentLocation, IsInRoom, Say, GetAllRooms, Ask, Pick, Place


class RobotActions(Node):
    def __init__(self):
        super().__init__('test_pick_actions')

        self.pick_status = False

        # Action servers
        self.pick_server = ActionServer(self, Pick, "/pick_server", self.pick_callback)

        self.pick_status_sub = self.create_subscription(Bool, "/pick_goal_status", self.pick_status_callback, 1)
        self.get_logger().info("======= Started all robot action servers =======")

    def pick_status_callback(self, msg):
        # True = done, False = not done
        self.pick_status = msg.data

    def pick_callback(self, goal_handle):
        print(f"Recieved a pick request!!")
        goal = goal_handle.request
        result = Pick.Result()
        while self.pick_status == False:
            time.sleep(0.1)

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    
    robot_actions = RobotActions()
    
    def signal_handler(sig, frame):
        print("Ctrl+C detected! Killing server...")
        robot_actions.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    executor = MultiThreadedExecutor()
    executor.add_node(robot_actions)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_actions.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
