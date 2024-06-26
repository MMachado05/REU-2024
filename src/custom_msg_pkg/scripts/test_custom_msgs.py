#!/usr/bin/env python3
import rospy
from custom_msg_pkg.msg import TrafficMessage

# Initialize node
rospy.init_node('custom_message_publisher')

# Create a publisher for your custom message
pub = rospy.Publisher('sensor_reading_topic', TrafficMessage, queue_size=10)

# Create a SensorReading message
msg = TrafficMessage()
msg.state = 1
msg.time = 256

# Publish the message
while not rospy.is_shutdown():
	pub.publish(msg)

# Spin to keep the node alive
rospy.spin()

