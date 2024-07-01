#!/usr/bin/env python3

import rospy
import rospkg
import cv2
import numpy
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
# from ....cfg import Config
from std_msgs.msg import Bool
from std_msgs.msg import Time
import threading
from cv_bridge import CvBridge, CvBridgeError

class RSUManager:
  
    def __init__(self):
        self.green_dur = 7
        self.all_red_dur = 2
        
        self.timeN = rospy.Time()
        self.timeS = rospy.Time()

        self.north_pub = rospy.Publisher("/north/state", Bool, queue_size=10)
        self.south_pub = rospy.Publisher("/south/state", Bool, queue_size=10)
        
        self.north_image = rospy.Publisher("/north/image", Image, queue_size=10)
        self.south_image = rospy.Publisher("/south/image", Image, queue_size=10)
        
        self.timeN_pub = rospy.Publisher("/north/time_to_state", Time, queue_size=1)
        self.timeS_pub = rospy.Publisher("/south/time_to_state", Time, queue_size=1)
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('traffic_light_pkg')
        state1_path = f"{package_path}/images/demo_go.jpg"
        state2_path = f"{package_path}/images/demo_stop.jpg"
        state1 = cv2.imread(state1_path)
        state2 = cv2.imread(state2_path)
        bridge = CvBridge()
        try:
            self.go_msg = bridge.cv2_to_imgmsg(state1, encoding="bgr8")
            self.stop_msg = bridge.cv2_to_imgmsg(state2, encoding="bgr8")

        except CvBridgeError as e:
            rospy.logerr(f"Error converting images: {e}")
            return
        
    
    def publish_time(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if (self.timeN.to_sec() >= rospy.Time.now().to_sec()):
                self.timeN_pub.publish(self.timeN - (rospy.Time.now()))
            if (self.timeS.to_sec() >= rospy.Time.now().to_sec()):
                self.timeS_pub.publish(self.timeS - (rospy.Time.now()))
            rate.sleep()
    
    def intersection(self):
        
        self.south_pub.publish(0)
        self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur + self.green_dur)
        
        while not rospy.is_shutdown():
            
            self.north_pub.publish(1)
            self.timeN = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.green_dur)
            rospy.sleep(self.green_dur)
            
            self.north_pub.publish(0)
            self.timeN = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur * 2 + self.green_dur)
            rospy.sleep(self.all_red_dur)
            
            self.south_pub.publish(1)
            self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.green_dur)
            rospy.sleep(self.green_dur)
            
            self.south_pub.publish(0)
            self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur * 2 + self.green_dur)
            rospy.sleep(self.all_red_dur)
    
    def crosswalk(self):
        
        while not rospy.is_shutdown():
            
            self.north_pub.publish(1)
            self.south_pub.publish(1)
            self.north_image.publish(self.go_msg)
            self.south_image.publish(self.go_msg)
            self.timeN = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.green_dur)
            self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.green_dur)
            rospy.sleep(self.green_dur)
            
            self.north_pub.publish(0)
            self.south_pub.publish(0)
            self.north_image.publish(self.stop_msg)
            self.south_image.publish(self.stop_msg)
            self.timeN = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur * 2 + self.green_dur)
            self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur * 2 + self.green_dur)
            rospy.sleep(self.all_red_dur * 2 + self.green_dur)
            
        


if __name__ == '__main__':
    rospy.init_node("traffic_light", anonymous=True)
    manager = RSUManager()
    try:
        state_thread = threading.Thread(target=manager.crosswalk)
        time_thread = threading.Thread(target=manager.publish_time)
        
        state_thread.start()
        time_thread.start()
        
        rospy.spin()  # Keeps Python from exiting until this node is stopped
            
        state_thread.join()
        time_thread.join()
    except rospy.ROSInterruptException:
        pass