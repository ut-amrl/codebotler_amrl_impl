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
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String

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

CONSOLE_LOG_DIR = Path(os.getenv("CODEBOTLER_CONSOLE_LOG_DIR", "/tmp/codebotler_console_logs"))
CONSOLE_LOG_FILE = CONSOLE_LOG_DIR / "actions.log"


def enable_console_log():
    CONSOLE_LOG_DIR.mkdir(parents=True, exist_ok=True)
    CONSOLE_LOG_FILE.write_text("")

    original_stdout_fd = os.dup(1)
    read_fd, write_fd = os.pipe()
    os.dup2(write_fd, 1)
    os.dup2(write_fd, 2)
    os.close(write_fd)

    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    def pump_console():
        with open(CONSOLE_LOG_FILE, "ab", buffering=0) as log:
            while True:
                try:
                    data = os.read(read_fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                log.write(data)
                try:
                    os.write(original_stdout_fd, data)
                except OSError:
                    pass

    threading.Thread(target=pump_console, name="actions_console_log", daemon=True).start()


enable_console_log()


class ActionCancelRequested(Exception):
    pass


def load_data() -> dict:
    with open(Path(__file__).resolve().parent / "data.yaml", "r") as f:
        return yaml.safe_load(f)


def active_locations(data: dict) -> dict:
    return data["LOCATIONS"][data["MAP"]]


def normalize_location(location: str) -> str:
    # TODO: make a cheap llm call to match location string to a known location name
    return location.replace("'s", "").replace("-", " ").lower()


def publish_robot_say(ctx, message: str):
    msg = String()
    msg.data = message
    ctx.robot_say_pub.publish(msg)


def publish_robot_ask(ctx, request: dict | None):
    msg = String()
    msg.data = "" if request is None else json.dumps(request)
    ctx.robot_ask_pub.publish(msg)


def speak(instruction: str) -> tuple[bool, str]:
    try:
        espeak = subprocess.Popen(
            ["/usr/bin/espeak", "--stdout", "-s", "105", "-p", "75", instruction],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except OSError as e:
        return False, f"failed to start espeak: {e}"

    try:
        aplay = subprocess.Popen(
            ["/usr/bin/aplay", "-D", "sysdefault:CARD=P20"],
            stdin=espeak.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
    except OSError as e:
        if espeak.stdout is not None:
            espeak.stdout.close()
        espeak.terminate()
        espeak.wait()
        return False, f"failed to start aplay: {e}"

    if espeak.stdout is not None:
        espeak.stdout.close()

    _, aplay_stderr = aplay.communicate()
    espeak_stderr = espeak.stderr.read() if espeak.stderr is not None else b""
    espeak_returncode = espeak.wait()

    errors = []
    if espeak_returncode != 0:
        errors.append(
            f"espeak exited with {espeak_returncode}: "
            f"{espeak_stderr.decode(errors='replace').strip()}"
        )
    if aplay.returncode != 0:
        errors.append(
            f"aplay exited with {aplay.returncode}: "
            f"{aplay_stderr.decode(errors='replace').strip()}"
        )
    if errors:
        return False, "; ".join(errors)
    return True, "speech playback completed"


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


def model_supports_custom_temperature(model: str) -> bool:
    return not model.startswith("gpt-5")


def query_openai_room_presence(
    client,
    model: str,
    image_url: str,
    obj: str,
) -> tuple[bool, str, float]:
    object_name = obj.strip()

    request = {
        "model": model,
        "messages": [
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
        "response_format": {
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
        "max_completion_tokens": 1024,
    }
    if model_supports_custom_temperature(model):
        request["temperature"] = 0

    response = client.chat.completions.create(**request)

    message = response.choices[0].message
    refusal = getattr(message, "refusal", None)
    if refusal:
        raise RuntimeError(f"OpenAI VLM refused the request: {refusal}")
    if not message.content:
        finish_reason = response.choices[0].finish_reason
        raise RuntimeError(f"OpenAI VLM returned empty content; finish_reason={finish_reason}")

    data = json.loads(message.content)
    answer = str(data["answer"]).strip().lower()
    confidence = float(data["confidence"])
    reason = str(data["reason"]).strip()
    return answer == "yes", reason, confidence


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
    missing_result_ok: bool = False,
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
        if missing_result_ok:
            return True, f"{action_name} completed without a result; continuing"
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
        self.latest_image_received_at = None
        self.image_event = threading.Event()
        self.ask_response = None
        self.ask_done_event = threading.Event()
        self.pick_result = None
        self.place_result = None
        self.pick_done_event = threading.Event()
        self.place_done_event = threading.Event()
        self.nav_status = None
        self.nav_status_event = threading.Event()
        self.cur_coords = (None, None, None)

        self.nav_goal_pub = node.create_publisher(
            Localization2DMsg, self.data["NAV_GOAL_TOPIC"], 1
        )
        self.nav_reset_pub = node.create_publisher(
            Empty, self.data["NAV_RESET_TOPIC"], 1
        )
        self.robot_say_pub = node.create_publisher(
            String, self.data["ROBOT_SAY_TOPIC"], 1
        )
        self.robot_ask_pub = node.create_publisher(
            String, self.data["ROBOT_ASK_TOPIC"], 1
        )
        self.pick_request_pub = node.create_publisher(
            String,
            self.data["PICK_REQUEST_TOPIC"],
            10,
        )
        self.place_request_pub = node.create_publisher(
            String,
            self.data["PLACE_REQUEST_TOPIC"],
            10,
        )

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
            self.data["PICK_STATUS_TOPIC"],
            self.pick_status_callback,
            5,
            callback_group=self.callback_group,
        )
        self.place_status_sub = node.create_subscription(
            Bool,
            self.data["PLACE_STATUS_TOPIC"],
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
        self.nav_status_event.set()

    def localization_callback(self, msg):
        self.cur_coords = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def image_callback(self, msg):
        self.latest_image_msg = msg
        self.latest_image_received_at = time.monotonic()
        self.image_event.set()


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
            cancel_callback=self.cancel,
            callback_group=ctx.callback_group,
        )

    @property
    def data(self):
        return self.ctx.data

    @property
    def logger(self):
        return self.node.get_logger()

    def cancel(self, goal_handle):
        self.logger.info(f"Cancel requested for {self.server_name}")
        self.on_cancel_requested(goal_handle)
        return CancelResponse.ACCEPT

    def on_cancel_requested(self, goal_handle):
        pass

    def is_cancel_requested(self, goal_handle) -> bool:
        return bool(getattr(goal_handle, "is_cancel_requested", False))

    def raise_if_cancel_requested(self, goal_handle):
        if self.is_cancel_requested(goal_handle):
            raise ActionCancelRequested

    def cancel_goal(self, goal_handle, result, detail: str):
        self.logger.info(detail)
        goal_handle.canceled()
        return result

    def execute(self, goal_handle):
        raise NotImplementedError


class GoToActionServer(BaseActionServer):
    action_type = GoTo
    server_name = "/go_to_server"
    NAV_STOPPED = 0
    NAV_ACTIVE = {1, 2}

    def on_cancel_requested(self, goal_handle):
        self.publish_reset_nav_goals()

    def publish_reset_nav_goals(self):
        msg = Empty()
        self.ctx.nav_reset_pub.publish(msg)
        self.logger.info(f"Published navigation reset to {self.data['NAV_RESET_TOPIC']}")

    def wait_for_nav_status(self, predicate, timeout_s: float, goal_handle=None):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if goal_handle is not None:
                self.raise_if_cancel_requested(goal_handle)
            status = self.ctx.nav_status
            if status is not None and predicate(status):
                return status

            self.ctx.nav_status_event.clear()
            status = self.ctx.nav_status
            if status is not None and predicate(status):
                return status

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.ctx.nav_status_event.wait(min(0.5, remaining))
        if goal_handle is not None:
            self.raise_if_cancel_requested(goal_handle)
        return None

    def publish_goal(self, goal_xytheta):
        goal_msg = Localization2DMsg()
        goal_msg.pose.x = goal_xytheta[0]
        goal_msg.pose.y = goal_xytheta[1]
        goal_msg.pose.theta = goal_xytheta[2]
        self.ctx.nav_goal_pub.publish(goal_msg)

    def execute(self, goal_handle):
        print("Received a go_to request:")
        goal = goal_handle.request
        result = GoTo.Result()
        print(f"Goal location: {goal.location}")

        try:
            self.raise_if_cancel_requested(goal_handle)
            location = normalize_location(goal.location)
            locations = active_locations(self.data)
            if location not in locations:
                print(f"Location {location} not found")
                goal_handle.abort()
                return result

            start_timeout_s = float(self.data["NAV_START_TIMEOUT_S"])
            complete_timeout_s = float(self.data["NAV_COMPLETE_TIMEOUT_S"])

            current_status = self.wait_for_nav_status(
                lambda status: status is not None,
                start_timeout_s,
                goal_handle,
            )
            if current_status is None:
                self.logger.error(
                    f"No navigation status received from {self.data['NAV_STATUS_TOPIC']}"
                )
                goal_handle.abort()
                return result
            if current_status != self.NAV_STOPPED:
                self.logger.error(
                    f"Cannot start go_to while navigation status is {current_status}; expected 0"
                )
                goal_handle.abort()
                return result

            goal_xytheta = locations[location]
            self.ctx.nav_status_event.clear()
            self.publish_goal(goal_xytheta)
            self.logger.info(
                f"Published go_to goal '{location}' to {self.data['NAV_GOAL_TOPIC']}: {goal_xytheta}"
            )

            started_status = self.wait_for_nav_status(
                lambda status: status in self.NAV_ACTIVE,
                start_timeout_s,
                goal_handle,
            )
            if started_status is None:
                self.logger.info(
                    f"Navigation stayed stopped for '{location}' after {start_timeout_s:.1f}s; treating goal as already complete"
                )

            stopped_status = self.wait_for_nav_status(
                lambda status: status == self.NAV_STOPPED,
                complete_timeout_s,
                goal_handle,
            )
            if stopped_status is None:
                self.logger.error(
                    f"Navigation to '{location}' did not stop within {complete_timeout_s:.1f}s"
                )
                goal_handle.abort()
                return result

        except ActionCancelRequested:
            self.publish_reset_nav_goals()
            return self.cancel_goal(goal_handle, result, "go_to canceled")

        goal_handle.succeed()
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
        if self.is_cancel_requested(goal_handle):
            return self.cancel_goal(goal_handle, result, "get_current_location canceled")
        if self.ctx.cur_coords[0] is None:
            self.logger.error(
                f"No localization available from {self.data['LOCALIZATION_TOPIC']}"
            )
            goal_handle.abort()
            return result

        result.result = self.check_and_update_locations(self.ctx.cur_coords)
        if self.is_cancel_requested(goal_handle):
            return self.cancel_goal(goal_handle, result, "get_current_location canceled")
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

    def image_is_recent(self) -> bool:
        if self.ctx.latest_image_msg is None or self.ctx.latest_image_received_at is None:
            return False
        max_age_s = float(self.data["CAM_IMG_MAX_AGE_S"])
        return time.monotonic() - self.ctx.latest_image_received_at <= max_age_s

    def wait_for_recent_image(self, goal_handle) -> bool:
        deadline = time.monotonic() + float(self.data["CAM_IMG_WAIT_TIMEOUT_S"])
        while time.monotonic() < deadline:
            self.raise_if_cancel_requested(goal_handle)
            if self.image_is_recent():
                return True
            self.ctx.image_event.clear()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.ctx.image_event.wait(min(0.5, remaining))
        self.raise_if_cancel_requested(goal_handle)
        return self.image_is_recent()

    def ask_openai_is_in_room(self, obj: str) -> tuple[bool, str, float]:
        client = self.get_openai_client()
        image_url = self.latest_image_data_url()
        return query_openai_room_presence(
            client,
            self.openai_vlm_model,
            image_url,
            obj,
        )

    def execute(self, goal_handle):
        goal = goal_handle.request
        obj = goal.object.strip()
        print(f"Received is_in_room request for {obj}")
        answer = IsInRoom.Result()
        if not obj:
            self.logger.error("is_in_room received an empty object name")
            answer.result = False
            goal_handle.abort()
            return answer

        try:
            if not self.wait_for_recent_image(goal_handle):
                self.logger.error(
                    f"No recent image available from {self.data['CAM_IMG_TOPIC']}"
                )
                answer.result = False
                goal_handle.abort()
                return answer

            self.raise_if_cancel_requested(goal_handle)
            present, reason, confidence = self.ask_openai_is_in_room(obj)
            self.raise_if_cancel_requested(goal_handle)
        except Exception as e:
            if isinstance(e, ActionCancelRequested):
                answer.result = False
                return self.cancel_goal(goal_handle, answer, "is_in_room canceled")
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

        publish_robot_say(self.ctx, message)
        print(f"Robot says: \"{message}\"")

        try:
            success, detail = speak(message)
        except Exception as e:
            success = False
            detail = f"unexpected speech playback error: {e}"
        finally:
            publish_robot_say(self.ctx, "")

        if self.is_cancel_requested(goal_handle):
            return self.cancel_goal(goal_handle, result, "say canceled")
        if not success:
            self.logger.error(f"say failed: {detail}")
            goal_handle.abort()
            return result

        goal_handle.succeed()
        return result


class GetAllRoomsActionServer(BaseActionServer):
    action_type = GetAllRooms
    server_name = "/get_all_rooms_server"

    def execute(self, goal_handle):
        result = GetAllRooms.Result()
        if self.is_cancel_requested(goal_handle):
            return self.cancel_goal(goal_handle, result, "get_all_rooms canceled")
        result.result = list(active_locations(self.data).keys())
        if self.is_cancel_requested(goal_handle):
            return self.cancel_goal(goal_handle, result, "get_all_rooms canceled")
        goal_handle.succeed()
        return result


class AskActionServer(BaseActionServer):
    action_type = Ask
    server_name = "/ask_server"
    ASK_OPTION_GAP_S = 1.0
    ASK_SUFFIX = "Select on the screen"

    def on_cancel_requested(self, goal_handle):
        self.clear_request()
        self.clear_response_state()

    def clear_request(self):
        publish_robot_ask(self.ctx, None)

    def clear_response_state(self):
        self.ctx.ask_response = None
        self.ctx.ask_done_event.clear()

    def wait_for_response(self, goal_handle, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.raise_if_cancel_requested(goal_handle)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self.ctx.ask_done_event.wait(min(0.25, remaining)):
                self.raise_if_cancel_requested(goal_handle)
                return self.ctx.ask_response
        self.raise_if_cancel_requested(goal_handle)
        return None

    def wait_with_cancel(self, goal_handle, duration_s: float):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.raise_if_cancel_requested(goal_handle)
            time.sleep(max(0.0, min(0.1, deadline - time.monotonic())))
        self.raise_if_cancel_requested(goal_handle)

    def speak_ask_prompt(self, goal_handle, question: str, options: list[str]) -> tuple[bool, str]:
        segments = [question] + [str(option) for option in options] + [self.ASK_SUFFIX]
        segments = [segment.strip() for segment in segments if segment and segment.strip()]

        for index, segment in enumerate(segments):
            if index > 0:
                self.wait_with_cancel(goal_handle, self.ASK_OPTION_GAP_S)

            self.raise_if_cancel_requested(goal_handle)
            success, detail = speak(segment)
            self.raise_if_cancel_requested(goal_handle)
            if not success:
                return False, f"speech failed for {segment!r}: {detail}"

        return True, "ask speech playback completed"

    def execute(self, goal_handle):
        goal = goal_handle.request
        person = goal.person
        question = goal.question
        options = list(goal.options)
        result = Ask.Result()

        print(f"Robot asks {person}: \"{question}\" with options {options}")
        self.clear_response_state()

        try:
            self.raise_if_cancel_requested(goal_handle)
            publish_robot_ask(self.ctx, {
                "person": person,
                "question": question,
                "options": options,
            })

            speech_success, speech_detail = self.speak_ask_prompt(
                goal_handle,
                question,
                options,
            )
            self.raise_if_cancel_requested(goal_handle)

            if not speech_success:
                self.clear_request()
                self.clear_response_state()
                self.logger.error(f"ask speech failed: {speech_detail}")
                goal_handle.abort()
                return result

            timeout_s = float(self.data.get("ASK_TIMEOUT_S", 120.0))
            response = self.wait_for_response(goal_handle, timeout_s)
            if response is None:
                self.clear_request()
                self.clear_response_state()
                self.logger.warning(f"ask timed out after {timeout_s:.1f}s: {question}")
                goal_handle.abort()
                return result

        except ActionCancelRequested:
            self.clear_request()
            self.clear_response_state()
            return self.cancel_goal(goal_handle, result, "ask canceled")
        except Exception as e:
            self.clear_request()
            self.clear_response_state()
            self.logger.error(f"ask failed: {e}")
            goal_handle.abort()
            return result

        self.clear_request()
        self.clear_response_state()
        print(f"Response: {response}")
        result.result = response
        goal_handle.succeed()
        return result


class PickActionServer(BaseActionServer):
    action_type = Pick
    server_name = "/pick_server"

    def cancel(self, goal_handle):
        self.logger.info("pick cancel requested; downstream pick cancel is not wired yet")
        return CancelResponse.REJECT

    def execute(self, goal_handle):
        print("Received a pick request:")
        goal = goal_handle.request
        result = Pick.Result()
        object_name = goal.obj.strip()
        if not object_name:
            self.logger.error("pick received an empty object name")
            goal_handle.abort()
            return result

        success, message = send_cobot_request_and_wait(
            node=self.node,
            ctx=self.ctx,
            action_name="pick",
            request_pub=self.ctx.pick_request_pub,
            request_text=object_name,
            done_event=self.ctx.pick_done_event,
            result_attr="pick_result",
            timeout_s=float(self.data["PICK_TIMEOUT_S"]),
            missing_result_ok=True,
        )

        if success:
            if "without a result" in message:
                self.logger.warning(message)
            else:
                self.logger.info(message)
            goal_handle.succeed()
        else:
            self.logger.error(message)
            goal_handle.abort()
        return result


class PlaceActionServer(BaseActionServer):
    action_type = Place
    server_name = "/place_server"

    def cancel(self, goal_handle):
        self.logger.info("place cancel requested; downstream place cancel is not wired yet")
        return CancelResponse.REJECT

    def execute(self, goal_handle):
        print("Received a place request")
        goal = goal_handle.request
        result = Place.Result()
        object_name = goal.obj.strip()
        if not object_name:
            self.logger.error("place received an empty object name")
            goal_handle.abort()
            return result

        success, message = send_cobot_request_and_wait(
            node=self.node,
            ctx=self.ctx,
            action_name="place",
            request_pub=self.ctx.place_request_pub,
            request_text=object_name,
            done_event=self.ctx.place_done_event,
            result_attr="place_result",
            timeout_s=float(self.data["PLACE_TIMEOUT_S"]),
            missing_result_ok=True,
        )

        if success:
            if "without a result" in message:
                self.logger.warning(message)
            else:
                self.logger.info(message)
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
