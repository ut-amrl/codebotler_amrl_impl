#!/usr/bin/env python3

import argparse
import base64
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from amrl_msgs.msg import Localization2DMsg, NavStatusMsg
from cobot_codebotler_actions.action import (
    Ask,
    GetAllRooms,
    GetCurrentLocation,
    GoTo,
    IsInRoom,
    Pick,
    Place,
    Say,
)


def load_data() -> dict:
    with open(Path(__file__).resolve().parent / "data.yaml", "r") as f:
        return yaml.safe_load(f)


def active_locations(data: dict) -> dict:
    return data["LOCATIONS"][data["MAP"]]


def normalize_location(location: str) -> str:
    return location.replace("'s", "").replace("-", " ").lower()


def publish_robot_say(ctx, message: str):
    msg = String()
    msg.data = message
    ctx.robot_say_pub.publish(msg)


def speak(instruction: str):
    espeak = subprocess.Popen(
        ["/usr/bin/espeak", "--stdout", "-s", "105", "-p", "75", f"\"{instruction}\""],
        stdout=subprocess.PIPE,
    )
    aplay = subprocess.Popen(
        ["/usr/bin/aplay", "-D", "sysdefault:CARD=P20"],
        stdin=espeak.stdout,
    )
    espeak.wait()
    aplay.wait()


def load_openai_api_key() -> str:
    key_path = Path(__file__).resolve().parent / ".openai_api_key"
    if key_path.exists():
        key = key_path.read_text().strip()
        if key:
            return key

    env_key = os.getenv("OPENAI_API_KEY", "").strip()
    if env_key:
        return env_key

    raise RuntimeError(
        "OpenAI API key not found. Create '.openai_api_key' in the "
        "codebotler_amrl_impl repo or set OPENAI_API_KEY."
    )


def send_cobot_request_and_wait(
    *,
    node: Node,
    ctx,
    action_name: str,
    request_pub,
    request_text: str,
    done_event: threading.Event,
    result_attr: str,
    timeout_s: float = 180.0,
) -> tuple[bool, str]:
    setattr(ctx, result_attr, None)
    done_event.clear()

    msg = String()
    msg.data = request_text
    request_pub.publish(msg)
    node.get_logger().info(f"Published {action_name} request: {request_text}")

    if not done_event.wait(timeout_s):
        return False, f"{action_name} timed out waiting for Cobot result"

    result = getattr(ctx, result_attr)
    setattr(ctx, result_attr, None)
    done_event.clear()

    if result is None:
        return False, f"{action_name} completed without a result"
    if not result:
        return False, f"{action_name} failed in Cobot"
    return True, f"{action_name} completed"


class RobotContext:
    def __init__(self, node: Node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()
        self.data = load_data()

        self.latest_image_msg = None
        self.ask_response = None
        self.ask_done_event = threading.Event()
        self.pick_result = None
        self.place_result = None
        self.pick_done_event = threading.Event()
        self.place_done_event = threading.Event()
        self.nav_status = None
        self.cur_coords = (None, None, None)

        self.nav_goal_pub = node.create_publisher(
            Localization2DMsg, self.data["NAV_GOAL_TOPIC"], 1
        )
        self.robot_say_pub = node.create_publisher(
            String, self.data["ROBOT_SAY_TOPIC"], 1
        )
        self.robot_ask_pub = node.create_publisher(
            String, self.data["ROBOT_ASK_TOPIC"], 1
        )
        self.pick_request_pub = node.create_publisher(String, "/pick_request", 10)
        self.place_request_pub = node.create_publisher(String, "/place_request", 10)

        self.localization_sub = node.create_subscription(
            Localization2DMsg,
            self.data["LOCALIZATION_TOPIC"],
            self.localization_callback,
            1,
            callback_group=self.callback_group,
        )
        self.nav_status_sub = node.create_subscription(
            NavStatusMsg,
            self.data["NAV_STATUS_TOPIC"],
            self.nav_status_callback,
            1,
            callback_group=self.callback_group,
        )
        self.human_response_sub = node.create_subscription(
            String,
            self.data["HUMAN_RESPONSE_TOPIC"],
            self.human_response_callback,
            10,
            callback_group=self.callback_group,
        )
        self.pick_status_sub = node.create_subscription(
            Bool,
            "/pick_goal_status",
            self.pick_status_callback,
            5,
            callback_group=self.callback_group,
        )
        self.place_status_sub = node.create_subscription(
            Bool,
            "/place_goal_status",
            self.place_status_callback,
            5,
            callback_group=self.callback_group,
        )
        self.image_sub = node.create_subscription(
            Image,
            self.data["CAM_IMG_TOPIC"],
            self.image_callback,
            5,
            callback_group=self.callback_group,
        )
        self.bridge = CvBridge()

    def pick_status_callback(self, msg):
        self.pick_result = msg.data
        self.pick_done_event.set()

    def place_status_callback(self, msg):
        self.place_result = msg.data
        self.place_done_event.set()

    def human_response_callback(self, msg):
        self.ask_response = msg.data
        self.ask_done_event.set()

    def nav_status_callback(self, msg):
        self.nav_status = msg.status

    def localization_callback(self, msg):
        self.cur_coords = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def image_callback(self, msg):
        self.latest_image_msg = msg


class BaseActionServer:
    action_type = None
    server_name = None

    def __init__(self, node: Node, ctx: RobotContext):
        self.node = node
        self.ctx = ctx
        self.server = ActionServer(
            node,
            self.action_type,
            self.server_name,
            self.execute,
            callback_group=ctx.callback_group,
        )

    @property
    def data(self):
        return self.ctx.data

    @property
    def logger(self):
        return self.node.get_logger()

    def execute(self, goal_handle):
        raise NotImplementedError


class GoToActionServer(BaseActionServer):
    action_type = GoTo
    server_name = "/go_to_server"

    def stop_robot(self):
        if self.ctx.cur_coords[0] is None:
            return
        goal_msg = Localization2DMsg()
        goal_msg.pose.x = self.ctx.cur_coords[0]
        goal_msg.pose.y = self.ctx.cur_coords[1]
        goal_msg.pose.theta = self.ctx.cur_coords[2]
        self.ctx.nav_goal_pub.publish(goal_msg)

    def current_distance(self, goal_xy):
        if self.ctx.cur_coords[0] is None:
            return None
        curr_loc = np.array(self.ctx.cur_coords)[:2]
        return np.linalg.norm(curr_loc - goal_xy)

    def execute(self, goal_handle):
        print("Received a go_to request:")
        goal = goal_handle.request
        result = GoTo.Result()
        print(f"Goal location: {goal.location}")

        location = normalize_location(goal.location)
        locations = active_locations(self.data)
        if location not in locations:
            print(f"Location {location} not found")
            publish_robot_say(
                self.ctx,
                "I don't know the location of the " + str(goal.location) + ". Aborting this mission.",
            )
            time.sleep(self.data["SLEEP_AFTER_SAY"] * 6 * 2)
            goal_handle.abort()
            return result

        publish_robot_say(self.ctx, "I am going to the " + str(goal.location))
        time.sleep(self.data["SLEEP_AFTER_SAY"] * 6 * 2)

        goal_xytheta = locations[location]
        goal_xy = np.array(goal_xytheta[:2])

        dist = self.current_distance(goal_xy)
        if dist is not None and dist < self.data["DIST_THRESHOLD"]:
            print(f"Already at the {location} (distance {dist:.2f}m). No need to move.")
            goal_handle.succeed()
            return result

        goal_msg = Localization2DMsg()
        goal_msg.pose.x = goal_xytheta[0]
        goal_msg.pose.y = goal_xytheta[1]
        goal_msg.pose.theta = goal_xytheta[2]
        self.ctx.nav_goal_pub.publish(goal_msg)
        time.sleep(0.2)

        wait_start = time.time()
        while self.ctx.nav_status is None:
            if time.time() - wait_start > 2.0:
                break
            time.sleep(0.05)

        nav_deadline = time.time() + 120.0
        motion_started = False
        resend_count = 0

        while time.time() < nav_deadline:
            status = self.ctx.nav_status
            if status in [1, 2, 3]:
                motion_started = True

            if status == 0 and motion_started:
                dist = self.current_distance(goal_xy)
                if dist is None:
                    time.sleep(0.5)
                    goal_handle.succeed()
                    return result
                if dist is not None and dist < self.data["DIST_THRESHOLD"]:
                    goal_handle.succeed()
                    return result
                if resend_count < 2:
                    self.ctx.nav_goal_pub.publish(goal_msg)
                    resend_count += 1
                    motion_started = False
                    time.sleep(0.2)
                    continue
                break

            time.sleep(0.1)

        self.stop_robot()
        goal_handle.abort()
        return result


class GetCurrentLocationActionServer(BaseActionServer):
    action_type = GetCurrentLocation
    server_name = "/get_current_location_server"

    def check_and_update_locations(self, new_loc):
        min_dist = np.inf
        closest_loc = None
        locations = active_locations(self.data)
        for loc, coords in locations.items():
            dist = np.sqrt((coords[0] - new_loc[0])**2 + (coords[1] - new_loc[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_loc = loc
        if min_dist <= self.data["DIST_THRESHOLD"]:
            return closest_loc
        locations["starting location"] = list(new_loc)
        return "starting location"

    def execute(self, goal_handle):
        result = GetCurrentLocation.Result()
        result.result = self.check_and_update_locations(self.ctx.cur_coords)
        goal_handle.succeed()
        return result


class IsInRoomActionServer(BaseActionServer):
    action_type = IsInRoom
    server_name = "/is_in_room_server"

    def __init__(self, node: Node, ctx: RobotContext, vlm_model: str):
        self.openai_client = None
        self.openai_vlm_model = vlm_model
        super().__init__(node, ctx)

    def get_openai_client(self):
        if self.openai_client is None:
            from openai import OpenAI
            self.openai_client = OpenAI(api_key=load_openai_api_key())
        return self.openai_client

    def latest_image_data_url(self) -> str:
        img_bgr = self.ctx.bridge.imgmsg_to_cv2(
            self.ctx.latest_image_msg,
            desired_encoding="bgr8",
        )
        ok, encoded = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            raise RuntimeError("Failed to encode latest camera image as JPEG")
        image_b64 = base64.b64encode(encoded.tobytes()).decode("ascii")
        return f"data:image/jpeg;base64,{image_b64}"

    def ask_openai_is_in_room(self, obj: str) -> tuple[bool, str, float]:
        client = self.get_openai_client()
        image_url = self.latest_image_data_url()
        object_name = obj.strip()

        response = client.chat.completions.create(
            model=self.openai_vlm_model,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You answer object-presence questions for a mobile robot using only the provided camera image. "
                        "Answer yes only when the requested object is clearly visible in the current room image. "
                        "If the object is absent, occluded, too ambiguous, or only inferable from context, answer no."
                    ),
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": (
                                f"Is there a {object_name} in this room? "
                                "Return a structured yes/no answer based only on this image."
                            ),
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image_url,
                                "detail": "low",
                            },
                        },
                    ],
                },
            ],
            response_format={
                "type": "json_schema",
                "json_schema": {
                    "name": "room_object_presence",
                    "strict": True,
                    "schema": {
                        "type": "object",
                        "properties": {
                            "answer": {"type": "string", "enum": ["yes", "no"]},
                            "confidence": {"type": "number"},
                            "reason": {"type": "string"},
                        },
                        "required": ["answer", "confidence", "reason"],
                        "additionalProperties": False,
                    },
                },
            },
            temperature=0,
            max_completion_tokens=120,
        )

        message = response.choices[0].message
        refusal = getattr(message, "refusal", None)
        if refusal:
            raise RuntimeError(f"OpenAI VLM refused the request: {refusal}")

        data = json.loads(message.content)
        answer = str(data["answer"]).strip().lower()
        confidence = float(data["confidence"])
        reason = str(data["reason"]).strip()
        return answer == "yes", reason, confidence

    def execute(self, goal_handle):
        goal = goal_handle.request
        obj = goal.object
        print(f"Received is_in_room request for {obj}")
        answer = IsInRoom.Result()
        if self.ctx.latest_image_msg is None:
            print("No image available from camera!")
            answer.result = False
            goal_handle.abort()
            return answer

        try:
            present, reason, confidence = self.ask_openai_is_in_room(obj)
        except Exception as e:
            self.logger.error(f"OpenAI VLM is_in_room failed: {e}")
            answer.result = False
            goal_handle.abort()
            return answer

        answer.result = present
        goal_handle.succeed()
        print(f"OpenAI VLM is_in_room('{obj}') -> {present} confidence={confidence:.2f}: {reason}")
        return answer


class SayActionServer(BaseActionServer):
    action_type = Say
    server_name = "/say_server"

    def execute(self, goal_handle):
        goal = goal_handle.request
        result = Say.Result()
        message = goal.message
        speak(message)

        publish_robot_say(self.ctx, message)
        print(f"Robot says: \"{message}\"")
        word_len = len(message.split(" "))
        time.sleep(self.data["SLEEP_AFTER_SAY"] * word_len * 2)
        goal_handle.succeed()
        return result


class GetAllRoomsActionServer(BaseActionServer):
    action_type = GetAllRooms
    server_name = "/get_all_rooms_server"

    def execute(self, goal_handle):
        result = GetAllRooms.Result()
        result.result = list(active_locations(self.data).keys())
        goal_handle.succeed()
        return result


class AskActionServer(BaseActionServer):
    action_type = Ask
    server_name = "/ask_server"

    def execute(self, goal_handle):
        goal = goal_handle.request
        person = goal.person
        question = goal.question
        options = list(goal.options)
        result = Ask.Result()

        print(f"Robot asks {person}: \"{question}\" with options {options}")
        self.ctx.ask_response = None
        self.ctx.ask_done_event.clear()

        msg = String()
        msg.data = json.dumps({
            "person": person,
            "question": question,
            "options": options,
        })
        self.ctx.robot_ask_pub.publish(msg)

        timeout_s = float(self.data.get("ASK_TIMEOUT_S", 120.0))
        if self.ctx.ask_done_event.wait(timeout_s):
            response = self.ctx.ask_response or "no answer"
        else:
            response = "no answer"
            self.logger.warning(f"ask timed out after {timeout_s:.1f}s: {question}")

        self.ctx.ask_response = None
        self.ctx.ask_done_event.clear()
        print(f"Response: {response}")
        word_len = len(question.split(" "))
        time.sleep(self.data["SLEEP_AFTER_ASK"] * word_len * 2)
        result.result = response
        goal_handle.succeed()
        return result


class PickActionServer(BaseActionServer):
    action_type = Pick
    server_name = "/pick_server"

    def execute(self, goal_handle):
        print("Received a pick request:")
        goal = goal_handle.request
        result = Pick.Result()
        object_name = goal.obj if hasattr(goal, "obj") else str(goal)

        success, message = send_cobot_request_and_wait(
            node=self.node,
            ctx=self.ctx,
            action_name="pick",
            request_pub=self.ctx.pick_request_pub,
            request_text=object_name,
            done_event=self.ctx.pick_done_event,
            result_attr="pick_result",
        )

        if success:
            goal_handle.succeed()
        else:
            self.logger.error(message)
            goal_handle.abort()
        return result


class PlaceActionServer(BaseActionServer):
    action_type = Place
    server_name = "/place_server"

    def execute(self, goal_handle):
        print("Received a place request")
        goal = goal_handle.request
        result = Place.Result()

        success, message = send_cobot_request_and_wait(
            node=self.node,
            ctx=self.ctx,
            action_name="place",
            request_pub=self.ctx.place_request_pub,
            request_text=goal.obj if hasattr(goal, "obj") and goal.obj else "place",
            done_event=self.ctx.place_done_event,
            result_attr="place_result",
        )

        if success:
            goal_handle.succeed()
        else:
            self.logger.error(message)
            goal_handle.abort()
        return result


class RobotActions(Node):
    def __init__(self, vlm_model: str):
        super().__init__("robot_low_level_actions")
        self.ctx = RobotContext(self)
        self.action_servers = [
            GoToActionServer(self, self.ctx),
            GetCurrentLocationActionServer(self, self.ctx),
            IsInRoomActionServer(self, self.ctx, vlm_model),
            SayActionServer(self, self.ctx),
            GetAllRoomsActionServer(self, self.ctx),
            AskActionServer(self, self.ctx),
            PickActionServer(self, self.ctx),
            PlaceActionServer(self, self.ctx),
        ]
        self.get_logger().info("======= Started all robot action servers =======")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--vlm-model", default="gpt-5-mini", help="OpenAI vision model for is_in_room")
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)

    robot_actions = RobotActions(vlm_model=parsed_args.vlm_model)

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
