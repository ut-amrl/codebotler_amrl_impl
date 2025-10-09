import yaml
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

from cobot_codebotler_actions.action import GoTo, GetCurrentLocation, IsInRoom, Say, GetAllRooms, Ask, Pick, Place
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RobotActions(Node):
    def __init__(self):
        super().__init__('test_dino')
        with open('../data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        config_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 
            "../third_party/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 
            "../third_party/GroundingDINO", "weights", "groundingdino_swint_ogc.pth")
        from zero_shot_object_detector import GroundingDINO
        self.object_detector_model = GroundingDINO(
            box_threshold=self.DATA['DINO']['box_threshold'],
            text_threshold=self.DATA['DINO']['text_threshold'], 
            device=self.device, 
            config_path=config_path, 
            weights_path=weights_path)

        self.latest_image_msg = None
        self.current_image_num = 0
        if os.path.exists(os.path.join("..", "images")):
            shutil.rmtree(os.path.join("..", "images"))

        # Action servers
        self.is_in_room_server = ActionServer(self, IsInRoom, "/dino", self.is_in_room_callback)

        # Subscribers
        self.image_sub = self.create_subscription(Image, self.DATA['CAM_IMG_TOPIC'], self.image_callback, 5)
        self.bridge = CvBridge()
        self.get_logger().info("======= Started all robot action servers =======")

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def is_in_room_callback(self, goal_handle):
        goal = goal_handle.request
        obj = goal.object
        print(f"Recieved is_in_room request for {obj}")
        answer = IsInRoom.Result()
        if self.latest_image_msg is None:
            print(f"No image available from camera!")
            answer.result = False
            return answer

        img1 = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='rgb8')

        #img1 = np.frombuffer(self.latest_image_data, np.uint8)
        #img2 = cv2.imdecode(img1, cv2.IMREAD_COLOR)
        #img3 = np.array(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
        boxes, logits, phrases, annotated_frame = self.object_detector_model.predict_from_image(img1, obj)

        image_dir = os.path.join("..", "images")
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)

        ann_image = Img.fromarray(np.array(annotated_frame).astype(np.uint8))
        ann_image.save(os.path.join(image_dir, f"annotated_frame_{self.current_image_num}.png"))
        self.current_image_num += 1
        num_boxes = len(boxes)
        answer.result = (num_boxes > 0)
        goal_handle.succeed()
        print(f"Detected {num_boxes} boxes")
        return answer
    

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
