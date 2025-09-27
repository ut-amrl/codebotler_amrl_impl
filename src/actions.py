#!/usr/bin/env python3

from zero_shot_object_detector import GroundingDINO
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
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

# External message types
from amrl_msgs.msg import NavStatusMsg, Localization2DMsg
from cobot_codebotler_actions.action import GoTo, GetCurrentLocation, IsInRoom, Say, GetAllRooms, Ask, Pick, Place


class RobotActions(Node):
    def __init__(self):
        super().__init__('robot_low_level_actions')
        with open('../data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO", "weights", "groundingdino_swint_ogc.pth")
        self.object_detector_model = GroundingDINO(box_threshold=self.DATA['DINO']['box_threshold'], text_threshold=self.DATA['DINO']['text_threshold'], device=self.device, config_path=config_path, weights_path=weights_path)
        self.latest_image_data = None
        self.nav_status = None
        self.current_image_num = 0
        if os.path.exists(os.path.join("..", "images")):
            shutil.rmtree(os.path.join("..", "images"))
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

        # Subscribers
        self.localization_sub = self.create_subscription(Localization2DMsg, self.DATA['LOCALIZATION_TOPIC'], self.localization_callback, 1)
        self.nav_status_sub = self.create_subscription(NavStatusMsg, self.DATA['NAV_STATUS_TOPIC'], self.nav_status_callback, 1)
        self.image_sub = self.create_subscription(CompressedImage, self.DATA['CAM_IMG_TOPIC'], self.image_callback, 1)
        self.get_logger().info("======= Started all robot action servers =======")

    def nav_status_callback(self, msg):
        self.nav_status = msg.status

    def localization_callback(self, msg):
        self.cur_coords = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def image_callback(self, msg):
        self.latest_image_data = msg.data

    def go_to_callback(self, goal_handle):
        goal = goal_handle.request
        result = GoTo.Result()
        
        def stop_robot():
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
            
        if type(self.cur_coords[0]) != type(None):
            curr_loc = np.array(self.cur_coords)[:2]
            goal_loc = np.array([self.DATA['LOCATIONS'][self.DATA['MAP']][location][0], self.DATA['LOCATIONS'][self.DATA['MAP']][location][1]])
            if np.linalg.norm(curr_loc - goal_loc) < self.DATA['DIST_THRESHOLD']:
                goal_handle.succeed()
                return result

        goal_msg.pose.x = self.DATA['LOCATIONS'][self.DATA['MAP']][location][0]
        goal_msg.pose.y = self.DATA['LOCATIONS'][self.DATA['MAP']][location][1]
        goal_msg.pose.theta = self.DATA['LOCATIONS'][self.DATA['MAP']][location][2]
        self.nav_goal_pub.publish(goal_msg)
        time.sleep(0.2)
        while self.nav_status == 0:  # to ensure that the robot has started moving
            time.sleep(0.1)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                stop_robot()
                return result
        while self.nav_status in [2, 3]:  # to ensure that the robot has reached the goal
            time.sleep(0.1)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                stop_robot()
                return result
        time.sleep(1)  # to ensure that the robot has stopped moving
        goal_handle.succeed()
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

    def is_in_room_callback(self, goal_handle):
        goal = goal_handle.request
        object = goal.object
        result = IsInRoom.Result()
        img1 = np.frombuffer(self.latest_image_data, np.uint8)
        img2 = cv2.imdecode(img1, cv2.IMREAD_COLOR)
        img3 = np.array(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
        boxes, logits, phrases, annotated_frame = self.object_detector_model.predict_from_image(img3, object)

        image_dir = os.path.join("..", "images")
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)

        ann_image = Image.fromarray(np.array(annotated_frame).astype(np.uint8))
        ann_image.save(os.path.join(image_dir, f"annotated_frame_{self.current_image_num}.png"))
        self.current_image_num += 1
        result.result = (len(boxes) > 0)
        goal_handle.succeed()
        return result

    def say_callback(self, goal_handle):
        goal = goal_handle.request
        result = Say.Result()
        message = goal.message
        if "===SING===" in message:
            self.sing(message)
            goal_handle.succeed()
            return result
        
        msg = String()
        msg.data = message
        self.robot_say_pub.publish(msg)
        print(f"Robot says: \"{message}\"")
        word_len = len(message.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_SAY'] * word_len * 2)
        goal_handle.succeed()
        return result
            
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
        # TODO: take care of transitioning this part properly yourselves
        # Implement pick functionality
        result = Pick.Result()
        goal_handle.succeed()
        return result

    def place_callback(self, goal_handle):
        # TODO: take care of transitioning this part properly yourselves
        # Implement place functionality
        result = Place.Result()
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
