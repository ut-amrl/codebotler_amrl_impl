#!/usr/bin/env python
import rospy 
from std_msgs.msg import String


rospy.init_node("my_node")

commands_pub = rospy.Publisher("chat_commands", String, queue_size=10)
data = """
start_loc = get_current_location()
go_to(start_loc)
milk_carton_found = is_in_room("milk carton")
go_to(start_loc)
if milk_carton_found:
    say("There is a milk carton here")
else:
    say("There is no milk carton here")
"""


commands_pub.publish(data)
rate = rospy.Rate(1)
rate.sleep()
print("??")
rospy.spin()