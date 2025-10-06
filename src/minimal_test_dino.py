import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

def image_handler(msg):
    print(f"Recieved {msg.header.stamp}")


#rclpy.init()
#n = Node("test")
#n.create_subscription(Image, "/rgb/image_raw", image_handler, 5)
#rclpy.spin(n)
#n.destroy_node()
#rclpy.shutdown()

class TestClass(Node):
    def __init__(self):
        super().__init__('test_dino')
        self.image_sub = self.create_subscription(Image, "/rgb/image_raw", self.image_callback, 5)

    def image_callback(self, msg):
        print(f"Recieved {msg.header.stamp}")

rclpy.init()
n = TestClass()
rclpy.spin(n)
n.destroy_node()
rclpy.shutdown()
