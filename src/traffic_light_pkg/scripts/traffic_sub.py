#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
# from ....cfg import Config
from std_msgs.msg import Bool
from std_msgs.msg import Time
import threading

last_published_message = None


class LaneFollowBlob:

    def __init__(self):
        # self.dynamic_reconfigure_server = Server(BlobConfig, self.dynamic_reconfigure_callback)
        rospy.Subscriber("/north/state", Bool, self.state_callback, queue_size=10)
        # rospy.Subscriber("/north/time", Bool, self.time_callback, queue_size=1)
        global last_published_message
        if last_published_message is not None:
            rospy.loginfo("Using last published message: %s", last_published_message)
            self.state_callback(last_published_message)


    # def dynamic_reconfigure_callback(self, config, level):
    #     rospy.logwarn('Got config!')
    #     self.config = config
    #     return config


    def state_callback(self, msg):
        print(msg)


rospy.init_node('north')
blob = LaneFollowBlob()
rospy.spin()