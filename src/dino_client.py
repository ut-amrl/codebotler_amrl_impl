import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from typing import List
import time
import signal
import sys
from cobot_codebotler_actions.action import (
    IsInRoom,
)

class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface')
        
        # Action clients
        self.is_in_room_client = ActionClient(self, IsInRoom, "/dino")

        print("====== Waiting for robot action servers... ======")
        self.is_in_room_client.wait_for_server()
        
        print("======= Connected to robot action servers =======")
    def _handle_client(self, client, goal, action_name):
        goal_handle = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle)
        
        goal_handle = goal_handle.result()
        if not goal_handle.accepted:
            raise Exception(f"{action_name}() goal was rejected!")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            raise RobotExecutionInterrupted(f"{action_name}()")
        elif result.status != GoalStatus.STATUS_SUCCEEDED:
            raise Exception(f"{action_name}() failed with status {result.status}")
        
        return result.result

    def is_in_room(self, obj: str) -> bool:
        goal = IsInRoom.Goal(object=obj)
        print(f"Asking if {obj} is in the room")
        return self._handle_client(self.is_in_room_client, goal, "is_in_room").result

def main(args=None):
    rclpy.init(args=args)
    r = RobotInterface()
    r.is_in_room("soda can")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
