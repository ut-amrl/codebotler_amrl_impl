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

        # Subscribers
        self.pick_status_sub = self.create_subscription(Bool, "/pick_goal_status", self.pick_status_callback, 1)
        
        # Publishers
        self.pick_request_pub = self.create_publisher(String, "/pick_request", 10)
        
        self.get_logger().info("======= Started all robot action servers =======")

    def pick_status_callback(self, msg):
        # True = done, False = not done
        self.pick_status = msg.data

    def pick_callback(self, goal_handle):
        self.get_logger().info(f"Received a pick request!")
        print("HERE?")
        print(goal_handle)
        goal = goal_handle.request
        result = Pick.Result()
        
        # Extract object name from the goal
        object_name = goal.object if hasattr(goal, 'object') else str(goal)
        self.get_logger().info(f"Pick object: {object_name}")
        
        # Reset status and publish the pick request
        self.pick_status = False
        pick_msg = String()
        pick_msg.data = object_name
        self.pick_request_pub.publish(pick_msg)
        self.get_logger().info(f"Published pick request to /pick_request: {object_name}")
        
        # Wait for pick to complete
        while self.pick_status == False:
            time.sleep(0.1)

        self.get_logger().info("Pick completed!")
        goal_handle.succeed()
        result.success = True
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
