import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from cobot_codebotler_actions.action import (
    Pick,
)


class RobotExecutionInterrupted(Exception):
    pass


class RobotInterface(Node):
    def __init__(self):
        super().__init__('pick_test_client')
        
        # Action clients
        self.pick_client = ActionClient(self, Pick, "/pick_server")

        print("====== Waiting for robot action servers... ======")
        self.pick_client.wait_for_server()
        
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

    def pick(self, obj: str):
        goal = Pick.Goal(obj=obj)
        print(f"Requesting pick of {obj}")
        return self._handle_client(self.pick_client, goal, "pick")

def main(args=None):
    rclpy.init(args=args)
    r = RobotInterface()
    result = r.pick("soda can")
    print(f"Pick result: success={result.success}, message={result.message}")
    r.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
