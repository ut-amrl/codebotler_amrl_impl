#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import base64
import json
import yaml
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
import sys
from pathlib import Path
import signal
import threading
import subprocess

# External message types
from amrl_msgs.msg import NavStatusMsg, Localization2DMsg
from cobot_codebotler_actions.action import GoTo, GetCurrentLocation, IsInRoom, Say, GetAllRooms, Ask, Pick, Place


class RobotActions(Node):
    def __init__(self):
        super().__init__('robot_low_level_actions')
        self.callback_group = ReentrantCallbackGroup()
        with open('../data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)

        self.latest_image_msg = None
        self.openai_client = None
        self.openai_vlm_model = os.getenv("COBOT_OPENAI_VLM_MODEL", "gpt-4o-mini")
        self.ask_response = None
        self.ask_done_event = threading.Event()
        self.pick_result = None
        self.place_result = None
        self.pick_done_event = threading.Event()
        self.place_done_event = threading.Event()
        self.nav_status = None
        self.cur_coords = (None, None, None)  # (x, y, theta)

        # Action servers
        self.go_to_server = ActionServer(self, GoTo, "/go_to_server", self.go_to_callback, callback_group=self.callback_group)
        self.get_current_location_server = ActionServer(self, GetCurrentLocation, "/get_current_location_server", self.get_current_location_callback, callback_group=self.callback_group)
        self.is_in_room_server = ActionServer(self, IsInRoom, "/is_in_room_server", self.is_in_room_callback, callback_group=self.callback_group)
        self.say_server = ActionServer(self, Say, "/say_server", self.say_callback, callback_group=self.callback_group)
        self.get_all_rooms_server = ActionServer(self, GetAllRooms, "/get_all_rooms_server", self.get_all_rooms_callback, callback_group=self.callback_group)
        self.ask_server = ActionServer(self, Ask, "/ask_server", self.ask_callback, callback_group=self.callback_group)
        self.pick_server = ActionServer(self, Pick, "/pick_server", self.pick_callback, callback_group=self.callback_group)
        self.place_server = ActionServer(self, Place, "/place_server", self.place_callback, callback_group=self.callback_group)

        # Publishers
        self.nav_goal_pub = self.create_publisher(Localization2DMsg, self.DATA['NAV_GOAL_TOPIC'], 1)
        self.robot_say_pub = self.create_publisher(String, self.DATA['ROBOT_SAY_TOPIC'], 1)
        self.robot_ask_pub = self.create_publisher(String, self.DATA['ROBOT_ASK_TOPIC'], 1)
        self.pick_request_pub = self.create_publisher(String, "/pick_request", 10)
        self.place_request_pub = self.create_publisher(String, "/place_request", 10)

        # Subscribers
        self.localization_sub = self.create_subscription(Localization2DMsg, self.DATA['LOCALIZATION_TOPIC'], self.localization_callback, 1, callback_group=self.callback_group)
        self.nav_status_sub = self.create_subscription(NavStatusMsg, self.DATA['NAV_STATUS_TOPIC'], self.nav_status_callback, 1, callback_group=self.callback_group)
        self.human_response_sub = self.create_subscription(String, self.DATA['HUMAN_RESPONSE_TOPIC'], self.human_response_callback, 10, callback_group=self.callback_group)
        self.pick_status_sub = self.create_subscription(Bool, "/pick_goal_status", self.pick_status_callback, 5, callback_group=self.callback_group)
        self.place_status_sub = self.create_subscription(Bool, "/place_goal_status", self.place_status_callback, 5, callback_group=self.callback_group)
        self.image_sub = self.create_subscription(Image, self.DATA['CAM_IMG_TOPIC'], self.image_callback, 5, callback_group=self.callback_group)
        self.bridge = CvBridge()
        self.get_logger().info("======= Started all robot action servers =======")
    
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

    def go_to_callback(self, goal_handle):
        print(f"Recieved a go_to request:")
        goal = goal_handle.request
        result = GoTo.Result()
        print(f"Goal location: {goal.location}")
        
        def stop_robot():
            if type(self.cur_coords[0]) == type(None):
                return
            goal_msg = Localization2DMsg()
            goal_msg.pose.x = self.cur_coords[0]
            goal_msg.pose.y = self.cur_coords[1]
            goal_msg.pose.theta = self.cur_coords[2]
            self.nav_goal_pub.publish(goal_msg)
        
        location = goal.location

        # process location
        location = location.replace("'s", "")
        location = location.replace("-", " ")
        location = location.lower()

        success = True
        goal_msg = Localization2DMsg()
        if location not in self.DATA['LOCATIONS'][self.DATA['MAP']].keys():
            print(f"Location {location} not found")
            msg = String()
            msg.data = "I don't know the location of the " + str(goal.location) + ". Aborting this mission."
            self.robot_say_pub.publish(msg)
            time.sleep(self.DATA['SLEEP_AFTER_SAY'] * 6 * 2)
            goal_handle.succeed()
            return result
        
        msg = String()
        msg.data = "I am going to the " + str(goal.location)
        self.robot_say_pub.publish(msg)
        time.sleep(self.DATA['SLEEP_AFTER_SAY'] * 6 * 2)
            
        goal_xytheta = self.DATA['LOCATIONS'][self.DATA['MAP']][location]
        goal_xy = np.array(goal_xytheta[:2])

        def current_distance():
            if type(self.cur_coords[0]) == type(None):
                return None
            curr_loc = np.array(self.cur_coords)[:2]
            return np.linalg.norm(curr_loc - goal_xy)

        dist = current_distance()
        if dist is not None and dist < self.DATA['DIST_THRESHOLD']:
            print(f"Already at the {location} (distance {dist:.2f}m). No need to move.")
            goal_handle.succeed()
            return result

        goal_msg.pose.x = goal_xytheta[0]
        goal_msg.pose.y = goal_xytheta[1]
        goal_msg.pose.theta = goal_xytheta[2]
        self.nav_goal_pub.publish(goal_msg)
        time.sleep(0.2)

        wait_start = time.time()
        while self.nav_status is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                stop_robot()
                return result
            if time.time() - wait_start > 2.0:
                break
            time.sleep(0.05)

        nav_deadline = time.time() + 120.0
        motion_started = False
        resend_count = 0

        while time.time() < nav_deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                stop_robot()
                return result

            status = self.nav_status
            # if status in [2, 3]: # TODO
            if status in [1, 2, 3]: # TODO
                motion_started = True

            if status == 0 and motion_started:
                dist = current_distance()
                if dist is None:
                    time.sleep(0.5)
                    goal_handle.succeed()
                    return result
                if dist is not None and dist < self.DATA['DIST_THRESHOLD']:
                    goal_handle.succeed()
                    return result
                if resend_count < 2:
                    self.nav_goal_pub.publish(goal_msg)
                    resend_count += 1
                    motion_started = False
                    time.sleep(0.2)
                    continue
                break

            time.sleep(0.1)

        stop_robot()
        goal_handle.abort()
        return result

    def _check_and_update_locations(self, new_loc):
        min_dist = np.inf
        closest_loc = None
        for loc, coords in self.DATA['LOCATIONS'][self.DATA['MAP']].items():
            dist = np.sqrt((coords[0] - new_loc[0])**2 + (coords[1] - new_loc[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_loc = loc
        if min_dist <= self.DATA['DIST_THRESHOLD']:
            return closest_loc
        else:
            self.DATA['LOCATIONS'][self.DATA['MAP']][f"starting location"] = list(new_loc)  # Add new location to the dictionary
            # self.new_loc_counter += 1
            return "starting location"

    def get_current_location_callback(self, goal_handle):
        result = GetCurrentLocation.Result()
        result.result = self._check_and_update_locations(self.cur_coords)
        goal_handle.succeed()
        return result

    def _load_openai_api_key(self) -> str:
        env_key = os.getenv("OPENAI_API_KEY", "").strip()
        if env_key:
            return env_key

        candidates = []
        seen = set()

        def add_candidate(path: Path):
            resolved = path.expanduser()
            if resolved not in seen:
                candidates.append(resolved)
                seen.add(resolved)

        search_roots = []
        for path in (Path(__file__).resolve(), Path.cwd().resolve()):
            root = path if path.is_dir() else path.parent
            search_roots.append(root)
            search_roots.extend(root.parents)

        for root in search_roots:
            for codebotler_dir in (root / "codebotler", root / "src" / "codebotler"):
                add_candidate(codebotler_dir / ".openai_api_key")
                add_candidate(codebotler_dir / ".openai")
            add_candidate(root / ".openai_api_key")
            add_candidate(root / ".openai")

        for path in candidates:
            if path.is_file():
                key = path.read_text().strip()
                if key:
                    return key

        raise RuntimeError("OpenAI API key not found in OPENAI_API_KEY, src/codebotler/.openai_api_key, or src/codebotler/.openai")

    def _get_openai_client(self):
        if self.openai_client is None:
            from openai import OpenAI
            self.openai_client = OpenAI(api_key=self._load_openai_api_key())
        return self.openai_client

    def _latest_image_data_url(self) -> str:
        img_bgr = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        ok, encoded = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            raise RuntimeError("Failed to encode latest camera image as JPEG")
        image_b64 = base64.b64encode(encoded.tobytes()).decode("ascii")
        return f"data:image/jpeg;base64,{image_b64}"

    def _ask_openai_is_in_room(self, obj: str) -> tuple[bool, str, float]:
        client = self._get_openai_client()
        image_url = self._latest_image_data_url()
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

    def is_in_room_callback(self, goal_handle):
        goal = goal_handle.request
        obj = goal.object
        print(f"Recieved is_in_room request for {obj}")
        answer = IsInRoom.Result()
        if self.latest_image_msg is None:
            print(f"No image available from camera!")
            answer.result = False
            goal_handle.succeed()
            return answer

        try:
            present, reason, confidence = self._ask_openai_is_in_room(obj)
        except Exception as e:
            self.get_logger().error(f"OpenAI VLM is_in_room failed: {e}")
            answer.result = False
            goal_handle.succeed()
            return answer

        answer.result = present
        goal_handle.succeed()
        print(f"OpenAI VLM is_in_room('{obj}') -> {present} confidence={confidence:.2f}: {reason}")
        return answer

    def say_callback(self, goal_handle):
        goal = goal_handle.request
        result = Say.Result()
        message = goal.message
        #if "===SING===" in message:
        #    self.sing(message)
        #    goal_handle.succeed()
        #    return result
        self.say(message)
        
        msg = String()
        msg.data = message
        self.robot_say_pub.publish(msg)
        print(f"Robot says: \"{message}\"")
        word_len = len(message.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_SAY'] * word_len * 2)
        goal_handle.succeed()
        return result

    def say(self, instruction: str):
        # espeak --stdout -s 75 -p 75 "Ask me what I can do" | aplay -D sysdefault:CARD=P20
        espeak = subprocess.Popen(["/usr/bin/espeak", "--stdout", "-s", "105", "-p", "75", f"\"{instruction}\""], stdout=subprocess.PIPE)
        aplay = subprocess.Popen(["/usr/bin/aplay", "-D", "sysdefault:CARD=P20"], stdin=espeak.stdout)
        espeak.wait()
        aplay.wait()

    def get_all_rooms_callback(self, goal_handle):
        result = GetAllRooms.Result()
        result.result = list(self.DATA['LOCATIONS'][self.DATA['MAP']].keys())
        goal_handle.succeed()
        return result

    def ask_callback(self, goal_handle):
        goal = goal_handle.request
        person = goal.person
        question = goal.question
        options = list(goal.options)
        result = Ask.Result()

        print(f"Robot asks {person}: \"{question}\" with options {options}")
        self.ask_response = None
        self.ask_done_event.clear()

        msg = String()
        msg.data = json.dumps({
            "person": person,
            "question": question,
            "options": options,
        })
        self.robot_ask_pub.publish(msg)

        timeout_s = float(self.DATA.get('ASK_TIMEOUT_S', 120.0))
        if self.ask_done_event.wait(timeout_s):
            response = self.ask_response or "no answer"
        else:
            response = "no answer"
            self.get_logger().warning(f"ask timed out after {timeout_s:.1f}s: {question}")

        self.ask_response = None
        self.ask_done_event.clear()
        print(f"Response: {response}")
        word_len = len(question.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_ASK'] * word_len * 2)
        result.result = response
        goal_handle.succeed()
        return result

    def _send_cobot_request_and_wait(
        self,
        *,
        action_name: str,
        request_pub,
        request_text: str,
        done_event: threading.Event,
        result_attr: str,
        timeout_s: float = 180.0,
    ) -> tuple[bool, str]:
        setattr(self, result_attr, None)
        done_event.clear()

        msg = String()
        msg.data = request_text
        request_pub.publish(msg)
        self.get_logger().info(f"Published {action_name} request: {request_text}")

        if not done_event.wait(timeout_s):
            return False, f"{action_name} timed out waiting for Cobot result"

        result = getattr(self, result_attr)
        setattr(self, result_attr, None)
        done_event.clear()

        if result is None:
            return False, f"{action_name} completed without a result"
        if not result:
            return False, f"{action_name} failed in Cobot"
        return True, f"{action_name} completed"

    def pick_callback(self, goal_handle):
        print(f"Recieved a pick request:")
        goal = goal_handle.request
        result = Pick.Result()
        
        # Extract object name from the goal
        object_name = goal.obj if hasattr(goal, 'obj') else str(goal)
        
        success, message = self._send_cobot_request_and_wait(
            action_name="pick",
            request_pub=self.pick_request_pub,
            request_text=object_name,
            done_event=self.pick_done_event,
            result_attr="pick_result",
        )

        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            self.get_logger().error(message)
            goal_handle.abort()
        return result

    def place_callback(self, goal_handle):
        print(f"Received a place request")
        goal = goal_handle.request
        result = Place.Result()

        success, message = self._send_cobot_request_and_wait(
            action_name="place",
            request_pub=self.place_request_pub,
            request_text=goal.obj if hasattr(goal, 'obj') and goal.obj else "place",
            done_event=self.place_done_event,
            result_attr="place_result",
        )

        if success:
            goal_handle.succeed()
        else:
            self.get_logger().error(message)
            goal_handle.abort()
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
