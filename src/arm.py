# --- Minimal inline helper for pick execution (MoveGroup + Execute + Gripper) ---
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory

class _GripperClient:
    def __init__(self, node, name="/robotiq_gripper_controller/gripper_cmd"):
        self.node = node
        self.client = ActionClient(node, GripperCommand, name)

    def move(self, position_m: float, max_effort: float = 100.0) -> bool:
        if not self.client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("Gripper action server not available")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position_m)
        goal.command.max_effort = float(max_effort)
        gh = self._send_goal_sync(goal)
        if not gh: return False
        res = self._get_result_sync(gh)
        if not res: return False
        r = res.result
        ok = bool(r.reached_goal) and not bool(r.stalled)
        return ok

    def _send_goal_sync(self, goal):
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        return gh if gh and gh.accepted else None

    def _get_result_sync(self, gh):
        fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, fut)
        return fut.result() if fut.result() else None


class _MoveItClient:
    def __init__(self, node, ns=""):
        self.node = node
        prefix = ("" if ns == "" else (ns.rstrip("/") + "/"))
        self.move_client = ActionClient(node, MoveGroup, f"{prefix}move_action")
        self.exec_client = ActionClient(node, ExecuteTrajectory, f"{prefix}execute_trajectory")

    def _goal_constraints(self, frame, ee_link, pos_tol, ori_tol, xyzquat):
        x,y,z,qx,qy,qz,qw = xyzquat
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        region_pose = Pose()
        region_pose.position.x = x; region_pose.position.y = y; region_pose.position.z = z
        region_pose.orientation.w = 1.0
        bv = BoundingVolume(primitives=[sphere], primitive_poses=[region_pose])

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = ee_link
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = ee_link
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        return c

    def plan(self, group, frame, ee_link, pos_tol, ori_tol, vel_scale, acc_scale, xyzquat):
        if not self.move_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("MoveGroup server missing")
            return None
        req = MotionPlanRequest()
        req.group_name = group
        req.goal_constraints = [self._goal_constraints(frame, ee_link, pos_tol, ori_tol, xyzquat)]
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale
        req.allowed_planning_time = 5.0

        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.plan_only = True
        g.planning_options.replan = False

        gh = self._send_goal_sync(self.move_client, g)
        if not gh: return None
        res = self._get_result_sync(gh)
        if not res or res.result.error_code.val != 1:
            return None
        return res.result.planned_trajectory

    def execute(self, traj: RobotTrajectory, timeout_sec=20.0) -> bool:
        if traj is None: return False
        if not self.exec_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("ExecuteTrajectory server missing")
            return False
        g = ExecuteTrajectory.Goal(); g.trajectory = traj
        gh = self._send_goal_sync(self.exec_client, g)
        if not gh: return False
        res = self._get_result_sync(gh)
        return bool(res and res.result.error_code.val == 1)

    def _send_goal_sync(self, client, goal):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        return gh if gh and gh.accepted else None

    def _get_result_sync(self, gh):
        fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, fut)
        return fut.result()


class ArmPickHelper:
    """
    Minimal pick primitive:
      1) open gripper
      2) plan to pre-grasp (offset along -X of frame)
      3) execute
      4) plan to grasp
      5) execute
      6) close gripper
    """
    def __init__(self, node,
                 group="manipulator", frame="base_link", ee_link="end_effector_link",
                 pos_tol=0.01, ori_tol=0.03, vel_scale=0.2, acc_scale=0.2,
                 gripper_action="/robotiq_gripper_controller/gripper_cmd"):
        self.node = node
        self.group = group
        self.frame = frame
        self.ee_link = ee_link
        self.pos_tol = pos_tol
        self.ori_tol = ori_tol
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.moveit = _MoveItClient(node)
        self.grip = _GripperClient(node, gripper_action)

    def pick_at(self, x,y,z, qx=0.5,qy=0.5,qz=0.5,qw=1.5, pre_grasp_offset=0.20, open_pos=0.0, close_pos=0.5) -> bool:
        # 0) open
        self.grip.move(open_pos)

        # 1) pre-grasp (move in from -X of frame)
        pre = [x - pre_grasp_offset, y, z, qx, qy, qz, qw]
        traj = self.moveit.plan(self.group, self.frame, self.ee_link, self.pos_tol, self.ori_tol,
                                self.vel_scale, self.acc_scale, pre)
        if traj is None or not self.moveit.execute(traj): return False

        # 2) grasp
        grasp = [x, y, z, qx, qy, qz, qw]
        traj = self.moveit.plan(self.group, self.frame, self.ee_link, self.pos_tol, self.ori_tol,
                                self.vel_scale, self.acc_scale, grasp)
        if traj is None or not self.moveit.execute(traj): return False

        # 3) close
        ok = self.grip.move(close_pos)
        return bool(ok)
