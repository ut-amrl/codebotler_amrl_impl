#!/usr/bin/env python
import time 
import rospy 
import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import Localization2DMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from amrl_msgs.msg import NavStatusMsg
import numpy as np 
from sensor_msgs.msg import CompressedImage
import cv2 

import json 

from grounded_dino_interface import *

box_threshold = 0.4
text_threshold = 0.4
device="cuda"
STATE = {"x" : -1, "y" : -1, "theta":-1}
current_path = os.path.abspath(__file__)
file_path = os.path.join(os.path.dirname(current_path), 'ahg_locations.json')
with open(file_path, "r") as f:
        all_rooms = json.load(f)

RAW_IMAGE_DATA = None
navigation_status = 0

def location_cb(location):
    global STATE
    STATE["x"] = location.pose.x
    STATE["y"] = location.pose.y
    STATE["theta"] = location.pose.theta

def go_to(location_str):
    global STATE, navigation_status
    location_str = location_str.lower()
    print(f"Robot goes to {location_str}")
    if not all_rooms.get(location_str):
        print(f"{location_str} does not exist")
        return 
    gx, gy = all_rooms[location_str]
    goal_loc = np.array([gx,gy])
    print("goal loc: ", goal_loc)
    x = STATE["x"]
    y = STATE["y"]
    
    curr_loc = np.array([x,y])
    if np.linalg.norm(curr_loc-goal_loc) < 1:
        return

    goal = PoseStamped()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.orientation.w = 1

    goal_pub.publish(goal)

    while navigation_status == 0:
        rate.sleep()

    while navigation_status in [2,3]:
        rate.sleep()

    print("goal reached")
    print("start sleeping: 5s", navigation_status)
    time.sleep(3)
    print("finish sleeping")
    # finish and sleep 2s to allow me to record

def get_current_location():
    global STATE
    x = STATE["x"]
    y = STATE["y"]
    all_rooms["start location"] = (x,y)
    print("start position is", (x,y))
    # curr = np.array([x,y])
    # min_dist = 100
    # loc = "nowhere"
    # for key, value in all_rooms.items():
    #     value = np.array(value)
    #     dist = np.linalg.norm(curr - value)
    #     if dist < min_dist:
    #         dist = min_dist
    #         loc = key
    return "start location"


def object_in_room(text_prompt):
    img = get_processed_image()

    print(img.shape)
    boxes_filt, pred_phrases = get_grounding_output(
        DINO_MODEL, img, text_prompt, box_threshold, text_threshold, device=device
    )
    if len(pred_phrases) > 0:
        print(f"{text_prompt} is in room" )
    else:
        print(f"{text_prompt} is not in room" )
    return len(pred_phrases) > 0
    # object_location = object_locations.get(object, None)
    # current_location = get_current_location()
    # if object_location == current_location:
    #     print(f"{object} is in {current_location}")
    #     return True
    # print(f"{object} is not in {current_location}")
    # return False

def is_in_room(text_prompt):
    img = get_processed_image()
    print(img.shape)
    boxes_filt, pred_phrases = get_grounding_output(
        DINO_MODEL, img, "human", box_threshold, text_threshold, device=device
    )
    return len(pred_phrases) > 0
    # object_location = object_locations.get(object, None)
    # return object_location == get_current_location()

def say(message):
    msg = String()
    msg.data = message
    robot_says_pub.publish(msg)
    print(f"Robot says: \"{message}\"")
    rate.sleep()

def get_all_rooms():
    global all_rooms
    return list(all_rooms.keys())

def ask(person, question, options=None):
    response = "no answer"
    if options != None:
        options.append(question)
        msg = String()
        msg.data = str(options)
        robot_asks_pub.publish(msg)
        response = rospy.wait_for_message('human_response', String).data

    if options == None:
        print(f"Robot asks {person}: \"{question}\"")
    else:
        print(f"Robot asks {person}: \"{question}\" with options {options}")
    # response = next(question_responses)
    # response = "no answer"
    print(f"Response: {response}")
    return response

def execute():
    pass 

def command_cb(commands: String):
    print("eval: ", commands)
    commands = commands.data
    exec(commands)

def image_cb(msg: CompressedImage):
    global RAW_IMAGE_DATA
    RAW_IMAGE_DATA = msg.data

def get_processed_image():
    global RAW_IMAGE_DATA
    compressed_img_data = np.frombuffer(RAW_IMAGE_DATA, np.uint8)
    img = cv2.imdecode(compressed_img_data, cv2.IMREAD_COLOR)
    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
    rgb_img = torch.tensor(rgb_img)
    rgb_img = rgb_img.permute(2,0,1)
    return rgb_img

def initialize_DINO():
    base_dir = "/home/amrl_user/zichaohu/robot_commands/Grounded-Segment-Anything/"
    config_file = base_dir + "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py" # change the path of the model config file
    grounded_checkpoint = base_dir + "groundingdino_swint_ogc.pth" # change the path of the model
    device = "cuda"

    model = load_model(config_file, grounded_checkpoint, device=device)
    return model

def goal_cb(nav_status):
    global navigation_status
    navigation_status = nav_status.status

if __name__ =="__main__":
    rospy.init_node('ros_interface')
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    robot_says_pub = rospy.Publisher('robot_say', String, queue_size=10)
    robot_asks_pub = rospy.Publisher('robot_ask', String, queue_size=10)

    commands_sub = rospy.Subscriber('chat_commands', String, command_cb)
    curr_loc_sub = rospy.Subscriber('localization', Localization2DMsg, location_cb)
    nav_status_sub = rospy.Subscriber('navigation_goal_status', NavStatusMsg, goal_cb)
    img_sub = rospy.Subscriber('/zed/zed_node/stereo/image_rect_color/compressed', CompressedImage, image_cb)

    DINO_MODEL = initialize_DINO()
    rate = rospy.Rate(10)
    rospy.spin()

    
 