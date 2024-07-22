#!/usr/bin/env python3

import roslib
roslib.load_manifest('amrl_msgs')
from zero_shot_object_detector import GroundingDINO
import rospy
import yaml
from amrl_msgs.msg import NavStatusMsg
from amrl_msgs.msg import Localization2DMsg
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
import actionlib
from robot_actions_pkg.msg import GoToAction, GetCurrentLocationAction, IsInRoomAction, SayAction, GetAllRoomsAction, AskAction, PickAction, PlaceAction
from robot_actions_pkg.msg import GoToResult, GetCurrentLocationResult, IsInRoomResult, SayResult, GetAllRoomsResult, AskResult, PickResult, PlaceResult
from StoppableThread import StoppableThread

class RobotActions:
    def __init__(self):
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
        self.go_to_server = actionlib.SimpleActionServer("/go_to_server", GoToAction, self.go_to, auto_start=False)
        self.get_current_location_server = actionlib.SimpleActionServer("/get_current_location_server", GetCurrentLocationAction, self.get_current_location, auto_start=False)
        self.is_in_room_server = actionlib.SimpleActionServer("/is_in_room_server", IsInRoomAction, self.is_in_room, auto_start=False)
        self.say_server = actionlib.SimpleActionServer("/say_server", SayAction, self.say, auto_start=False)
        self.get_all_rooms_server = actionlib.SimpleActionServer("/get_all_rooms_server", GetAllRoomsAction, self.get_all_rooms, auto_start=False)
        self.ask_server = actionlib.SimpleActionServer("/ask_server", AskAction, self.ask, auto_start=False)
        self.pick_server = actionlib.SimpleActionServer("/pick_server", PickAction, self.get_all_rooms, auto_start=False)
        self.place_server = actionlib.SimpleActionServer("/place_server", PlaceAction, self.get_all_rooms, auto_start=False)
        self.go_to_server.start()
        self.get_current_location_server.start()
        self.is_in_room_server.start()
        self.say_server.start()
        self.get_all_rooms_server.start()
        self.ask_server.start()
        self.pick_server.start()
        self.place_server.start()

        # Publishers
        self.nav_goal_pub = rospy.Publisher(self.DATA['NAV_GOAL_TOPIC'], Localization2DMsg, queue_size=1)
        self.robot_say_pub = rospy.Publisher(self.DATA['ROBOT_SAY_TOPIC'], String, queue_size=1)
        self.robot_ask_pub = rospy.Publisher(self.DATA['ROBOT_ASK_TOPIC'], String, queue_size=1)

        # Subscribers
        rospy.Subscriber(self.DATA['LOCALIZATION_TOPIC'], Localization2DMsg, self.localization_callback, queue_size=1)
        rospy.Subscriber(self.DATA['NAV_STATUS_TOPIC'], NavStatusMsg, self.nav_status_callback, queue_size=1)
        rospy.Subscriber(self.DATA['CAM_IMG_TOPIC'], CompressedImage, self.image_callback, queue_size=1, buff_size=2**32)
        print("======= Started all robot action servers =======")

    def nav_status_callback(self, msg):
        self.nav_status = msg.status

    def localization_callback(self, msg):
        self.cur_coords = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def image_callback(self, msg):
        self.latest_image_data = msg.data

    def go_to(self, goal):
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
            success = True
            if self.say_server.is_preempt_requested():
                self.say_server.set_preempted()
                success = False
            if success:
                self.say_server.set_succeeded()
            return
        
        msg = String()
        msg.data = "I am going to the " + str(goal.location)
        self.robot_say_pub.publish(msg)
        time.sleep(self.DATA['SLEEP_AFTER_SAY'] * 6 * 2)
        success = True
        if self.say_server.is_preempt_requested():
            self.say_server.set_preempted()
            success = False
        if success:
            self.say_server.set_succeeded()
            
        if type(self.cur_coords[0]) != type(None):
            curr_loc = np.array(self.cur_coords)[:2]
            goal_loc = np.array([self.DATA['LOCATIONS'][self.DATA['MAP']][location][0], self.DATA['LOCATIONS'][self.DATA['MAP']][location][1]])
            if np.linalg.norm(curr_loc - goal_loc) < self.DATA['DIST_THRESHOLD']:
                self.go_to_server.set_succeeded()
                return

        goal_msg.pose.x = self.DATA['LOCATIONS'][self.DATA['MAP']][location][0]
        goal_msg.pose.y = self.DATA['LOCATIONS'][self.DATA['MAP']][location][1]
        goal_msg.pose.theta = self.DATA['LOCATIONS'][self.DATA['MAP']][location][2]
        self.nav_goal_pub.publish(goal_msg)
        time.sleep(0.2)
        while self.nav_status == 0 and success:  # to ensure that the robot has started moving
            time.sleep(0.1)
            if self.go_to_server.is_preempt_requested():
                self.go_to_server.set_preempted()
                success = False
                stop_robot()
                break
        while self.nav_status in [2, 3] and success:  # to ensure that the robot has reached the goal
            time.sleep(0.1)
            if self.go_to_server.is_preempt_requested():
                self.go_to_server.set_preempted()
                success = False
                stop_robot()
                break
        time.sleep(1)  # to ensure that the robot has stopped moving
        if success:
            self.go_to_server.set_succeeded()

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

    def get_current_location(self, goal):
        r = GetCurrentLocationResult()
        success = True
        r.result = self._check_and_update_locations(self.cur_coords)
        if self.get_current_location_server.is_preempt_requested():
            self.get_current_location_server.set_preempted()
            success = False
        if success:
            self.get_current_location_server.set_succeeded(r)

    def is_in_room(self, goal):
        object = goal.object
        r = IsInRoomResult()
        success = True
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
        r.result = (len(boxes) > 0)
        if self.is_in_room_server.is_preempt_requested():
            self.is_in_room_server.set_preempted()
            success = False
        if success:
            self.is_in_room_server.set_succeeded(r)

    def say(self, goal):
        t = StoppableThread(target=self._say, args=[goal])
        t.start()
        while True:
            if self.say_server.is_preempt_requested():
                t.stop()
                self.say_server.set_preempted()
                break
            elif not self.say_server.isActive():
                break            

    def _say(self, goal):
        message = goal.message
        if "===SING===" in message:
            self.sing(message)
            time.sleep(self.DATA['SLEEP_AFTER_SAY'] * word_len * 2)
            self.say_server.set_succeeded()
            return
        msg = String()
        msg.data = message
        self.robot_say_pub.publish(msg)
        print(f"Robot says: \"{message}\"")
        word_len = len(message.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_SAY'] * word_len * 2)
        self.say_server.set_succeeded()
            
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

    def get_all_rooms(self, goal):
        r = GetAllRoomsResult()
        success = True
        r.result = list(self.DATA['LOCATIONS'][self.DATA['MAP']].keys())
        if self.get_all_rooms_server.is_preempt_requested():
            self.get_all_rooms_server.set_preempted()
            success = False
        if success:
            self.get_all_rooms_server.set_succeeded(r)

    def ask(self, goal):
        t = StoppableThread(target=self._ask, args=[goal])
        t.start()
        while True:
            if self.ask_server.is_preempt_requested():
                t.stop()
                self.ask_server.set_preempted()
                break
            elif not self.ask_server.isActive():
                break    

    def _ask(self, goal):
        person = goal.person
        question = goal.question
        options = goal.options
        r = AskResult()
        response = "no answer"
        if options == None:
            print(f"Robot asks {person}: \"{question}\"")
        else:
            print(f"Robot asks {person}: \"{question}\" with options {options}")
            options.append(question)
            msg = String()
            msg.data = str(options)
            self.robot_ask_pub.publish(msg)
            response = rospy.wait_for_message(self.DATA['HUMAN_RESPONSE_TOPIC'], String).data
        print(f"Response: {response}")
        word_len = len(question.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_ASK'] * word_len * 2)
        r.result = response
        self.ask_server.set_succeeded(r)


if __name__ == "__main__":
    rospy.init_node('robot_low_level_actions', anonymous=False)
    ra = RobotActions()

    def signal_handler(sig, frame):
        print("Ctrl+C detected! Killing server...")
        rospy.sleep(5)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time.sleep(1)
    rospy.spin()
