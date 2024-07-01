#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
# from ....cfg import Config
from std_msgs.msg import Bool
from std_msgs.msg import Time
import threading

last_published_message = None

class RSUManager:
  
    def __init__(self):
        self.green_dur = 7
        self.all_red_dur = 2
        
        self.timeN = rospy.Time()
        self.timeS = rospy.Time()

        self.north_pub = rospy.Publisher("/north/state", Bool, queue_size=10)
        self.south_pub = rospy.Publisher("/south/state", Bool, queue_size=10)
        
        self.timeN_pub = rospy.Publisher("/north/time_to_state", Time, queue_size=1)
        self.timeS_pub = rospy.Publisher("/south/time_to_state", Time, queue_size=1)
        # self.srv = Server(_Config, self.dyn_rcfg_cb)
    
    def dyn_rcfg_cb(self, config, level):
        self.thresh_value = config.thresh
        self.speed = config.speed
        self.enable_drive = config.enable_drive
        return config
    
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
            last_published_message = 1
            self.timeN = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.green_dur)
            rospy.sleep(self.green_dur)
            
            self.north_pub.publish(0)
            last_published_message = 0
            self.timeN = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur * 2 + self.green_dur)
            rospy.sleep(self.all_red_dur)
            
            self.south_pub.publish(1)
            self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.green_dur)
            rospy.sleep(self.green_dur)
            
            self.south_pub.publish(0)
            self.timeS = rospy.Time.from_sec(rospy.Time.now().to_sec() + self.all_red_dur * 2 + self.green_dur)
            rospy.sleep(self.all_red_dur)


if __name__ == '__main__':
    rospy.init_node("traffic_light", anonymous=True)
    manager = RSUManager()
    try:
        state_thread = threading.Thread(target=manager.intersection)
        time_thread = threading.Thread(target=manager.publish_time)
        
        state_thread.start()
        time_thread.start()
        
        rospy.spin()  # Keeps Python from exiting until this node is stopped
            
        state_thread.join()
        time_thread.join()
    except rospy.ROSInterruptException:
        pass