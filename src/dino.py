from zero_shot_object_detector import GroundingDINO
import os
import shutil
import yaml
import torch
import numpy as np
from PIL import Image
import cv2
from rclpy.node import Node
from rclpy.action import ActionServer

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node_for_dino')
        device = "cuda" if torch.cuda.is_available() else "cpu"

        with open('../data.yaml', 'r') as f:
            DATA = yaml.safe_load(f)
        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO", "weights", "groundingdino_swint_ogc.pth")
        self.object_detector_model = GroundingDINO(
                box_threshold=DATA['DINO']['box_threshold'], 
                text_threshold=DATA['DINO']['text_threshold'], 
                device=device, 
                config_path=config_path, 
                weights_path=weights_path)
        print("object detector initialized")
        if os.path.exists(os.path.join("..", "images")):
            shutil.rmtree(os.path.join("..", "images"))
        
        self.is_in_room_server = ActionServer(self, IsInRoom, "/is_in_room_server", self.is_in_room_callback)

    def is_in_room_callback(self, object_str):
        image_dir = os.path.join("..", "images")
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)
        obj = object_str
        img1 = np.frombuffer(latest_image_data, np.uint8)
        img2 = cv2.imdecode(img1, cv2.IMREAD_COLOR)
        img3 = np.array(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
        boxes, logits, phrases, annotated_frame = self.object_detector_model.predict_from_image(img3, obj)

        ann_image = Image.fromarray(np.array(annotated_frame).astype(np.uint8))
        ann_image.save(os.path.join(image_dir, f"annotated_frame_{self.current_image_num}.png"))
        current_image_num += 1
        num_boxes = len(boxes)
        return (num_boxes > 0), num_boxes
