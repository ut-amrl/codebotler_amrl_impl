#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import yaml
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import torch
import os
import sys
from PIL import Image as Img
import shutil
import signal
import threading
import subprocess

# External message types
from amrl_msgs.msg import NavStatusMsg, Localization2DMsg
from amrl_msgs.srv import GroundedSAM2Srv
from cobot_codebotler_actions.action import GoTo, GetCurrentLocation, IsInRoom, Say, GetAllRooms, Ask, Pick, Place


class RobotActions(Node):
    def __init__(self):
        super().__init__('robot_low_level_actions')
        with open('../data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)

        # Create GSAM2 service client
        self.gsam2_client = self.create_client(GroundedSAM2Srv, 'gsam2/infer')
        self.get_logger().info("Waiting for GSAM2 service...")
        while not self.gsam2_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GSAM2 service not available, waiting...')

        self.latest_image_msg = None
        self.current_image_num = 0
        if os.path.exists(os.path.join("..", "images")):
            shutil.rmtree(os.path.join("..", "images"))
        self.pick_status = False
        self.place_status = False
        self.is_in_room_status = False
        self.nav_status = None
        self.new_loc_counter = 0
        self.cur_coords = (None, None, None)  # (x, y, theta)

        # Action servers
        self.go_to_server = ActionServer(self, GoTo, "/go_to_server", self.go_to_callback)
        self.get_current_location_server = ActionServer(self, GetCurrentLocation, "/get_current_location_server", self.get_current_location_callback)
        self.is_in_room_server = ActionServer(self, IsInRoom, "/is_in_room_server", self.is_in_room_callback)
        self.say_server = ActionServer(self, Say, "/say_server", self.say_callback)
        self.get_all_rooms_server = ActionServer(self, GetAllRooms, "/get_all_rooms_server", self.get_all_rooms_callback)
        self.ask_server = ActionServer(self, Ask, "/ask_server", self.ask_callback)
        self.pick_server = ActionServer(self, Pick, "/pick_server", self.pick_callback)
        self.place_server = ActionServer(self, Place, "/place_server", self.place_callback)

        # Publishers
        self.nav_goal_pub = self.create_publisher(Localization2DMsg, self.DATA['NAV_GOAL_TOPIC'], 1)
        self.robot_say_pub = self.create_publisher(String, self.DATA['ROBOT_SAY_TOPIC'], 1)
        self.robot_ask_pub = self.create_publisher(String, self.DATA['ROBOT_ASK_TOPIC'], 1)
        self.pick_request_pub = self.create_publisher(String, "/pick_request", 10)
        self.place_request_pub = self.create_publisher(String, "/place_request", 10)
        self.is_in_room_pub = self.create_publisher(String, "/is_in_room_request", 10)

        # Subscribers
        self.localization_sub = self.create_subscription(Localization2DMsg, self.DATA['LOCALIZATION_TOPIC'], self.localization_callback, 1)
        self.nav_status_sub = self.create_subscription(NavStatusMsg, self.DATA['NAV_STATUS_TOPIC'], self.nav_status_callback, 1)
        self.pick_status_sub = self.create_subscription(Bool, "/pick_goal_status", self.pick_status_callback, 5)
        self.place_status_sub = self.create_subscription(Bool, "/place_goal_status", self.place_status_callback, 5)
        self.is_in_room_status_sub = self.create_subscription(Bool, "/is_in_room_status", self.is_in_room_status_callback, 5)
        self.image_sub = self.create_subscription(Image, self.DATA['CAM_IMG_TOPIC'], self.image_callback, 5)
        self.bridge = CvBridge()
        self.get_logger().info("======= Started all robot action servers =======")
    
    def pick_status_callback(self, msg):
        # True = done, False = not done
        self.pick_status = msg.data

    def place_status_callback(self, msg):
        # True = done, False = not done
        self.place_status = msg.data

    def is_in_room_status_callback(self, msg):
        # True = done, False = not done
        self.is_in_room_status = msg.data

    def nav_status_callback(self, msg):
        self.nav_status = msg.status

    def localization_callback(self, msg):
        self.cur_coords = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def go_to_callback(self, goal_handle):
        goal = goal_handle.request
        result = GoTo.Result()
        
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

    # def is_in_room_callback(self, goal_handle):
    #     goal = goal_handle.request
    #     obj = goal.object
    #     print(f"Recieved is_in_room request for {obj}")
    #     answer = IsInRoom.Result()
    #     if self.latest_image_msg is None:
    #         print(f"No image available from camera!")
    #         answer.result = False
    #         goal_handle.succeed()
    #         return answer

    #     # Convert image to BGR for GSAM2
    #     img_bgr = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        
    #     # Create GSAM2 service request
    #     request = GroundedSAM2Srv.Request()
    #     request.image = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
    #     request.text_prompt = obj if obj.endswith('.') else obj + '.'
    #     request.box_threshold = self.DATA['DINO']['box_threshold'] if 'DINO' in self.DATA else 0.35
    #     request.text_threshold = self.DATA['DINO']['text_threshold'] if 'DINO' in self.DATA else 0.45
    #     request.multimask_output = False
        
    #     # Call GSAM2 service
    #     future = self.gsam2_client.call_async(request)
        
    #     # Wait for the future without spinning (we're already in a callback)
    #     timeout = 30.0
    #     start_time = time.time()
    #     while not future.done() and (time.time() - start_time) < timeout:
    #         time.sleep(0.01)
        
    #     if not future.done() or future.result() is None:
    #         print(f"GSAM2 service call failed or timed out!")
    #         answer.result = False
    #         goal_handle.succeed()
    #         return answer
        
    #     response = future.result()
    #     num_detections = int(response.n)

    #     answer.result = (num_detections > 0)
    #     goal_handle.succeed()
    #     print(f"Detected {num_detections} instances of '{obj}'")
    #     return answer

    def is_in_room_callback(self, goal_handle):
        print(f"Received is_in_room request for {goal_handle.request.object}")
        goal = goal_handle.request
        result = IsInRoom.Result()
        
        # Extract object name from the goal
        object_name = goal.object if hasattr(goal, 'object') else str(goal)
        
        # Reset status and publish the is_in_room request
        self.is_in_room_status = False
        is_in_room_msg = String()
        is_in_room_msg.data = object_name
        self.is_in_room_pub.publish(is_in_room_msg)
        
        # Wait for is_in_room to complete
        while self.is_in_room_status == False:
            time.sleep(0.05)

        result.result = self.is_in_room_status
        goal_handle.succeed()
        self.is_in_room_status = False  # reset the status here
        return result

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

    def sing(self, instruction: str):
        # handle here
        instruction = instruction.lower()
        # yt-dlp -x -o "a.mp3" "https://www.youtube.com/watch?v=gm3-m2CFVWM" --audio-format mp3
        msg = String()
        msg.data = "Give me a second. Let me look up on Youtube!"
        self.robot_say_pub.publish(msg)
        
        os.system("rm song.mp3")
        os.system("rm output_audio.mp3")
        os.system(f'yt-dlp -x -o "song.mp3" "ytsearch1:song:{instruction}" --audio-format mp3')
        os.system("ffmpeg -i song.mp3 -ss 00:00:20 -t 00:00:15 -acodec copy output_audio.mp3")
        os.system('mpg123 output_audio.mp3')
        
        msg = String()
        msg.data = "Here is your free trial. If you want to hear more, PAY ME!"
        self.robot_say_pub.publish(msg)

    def get_all_rooms_callback(self, goal_handle):
        result = GetAllRooms.Result()
        result.result = list(self.DATA['LOCATIONS'][self.DATA['MAP']].keys())
        goal_handle.succeed()
        return result

    def ask_callback(self, goal_handle):
        goal = goal_handle.request
        person = goal.person
        question = goal.question
        options = goal.options
        result = Ask.Result()
        response = "no answer"
        if options == None:
            print(f"Robot asks {person}: \"{question}\"")
        else:
            print(f"Robot asks {person}: \"{question}\" with options {options}")
            options.append(question)
            msg = String()
            msg.data = str(options)
            self.robot_ask_pub.publish(msg)
            # TODO: take care of transitioning this part properly yourselves
            # ROS2 equivalent of wait_for_message needs to be implemented
            response = "no answer"  # Placeholder - need ROS2 message waiting
        print(f"Response: {response}")
        word_len = len(question.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_ASK'] * word_len * 2)
        result.result = response
        goal_handle.succeed()
        return result

    def pick_callback(self, goal_handle):
        print(f"Recieved a pick request:")
        goal = goal_handle.request
        result = Pick.Result()
        
        # Extract object name from the goal
        object_name = goal.obj if hasattr(goal, 'obj') else str(goal)
        
        # Reset status and publish the pick request
        self.pick_status = False
        pick_msg = String()
        pick_msg.data = object_name
        self.pick_request_pub.publish(pick_msg)
        
        # Wait for pick to complete
        while self.pick_status == False:
            time.sleep(0.05)

        goal_handle.succeed()
        self.pick_status = False # reset the status here
        return result

    def place_callback(self, goal_handle):
        print(f"Received a place request")
        goal = goal_handle.request
        result = Place.Result()
        
        # Reset status and publish the place request
        self.place_status = False
        place_msg = String()
        place_msg.data = "place"  # Simple trigger message
        self.place_request_pub.publish(place_msg)
        
        # Wait for place to complete
        while self.place_status == False:
            time.sleep(0.05)

        goal_handle.succeed()
        self.place_status = False  # reset the status here
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
