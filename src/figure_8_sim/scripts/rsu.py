#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool, Float64

# stoplight simulation
# lights change every 10 seconds
# only one light for both directions so far
def stop_light():

    # publish light status and remaining time
    stop_pub = rospy.Publisher('/robot1/stoplight', Bool, queue_size=1)
    time_pub = rospy.Publisher('/robot1/time', Float64, queue_size=1)
    
    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        curr = time.time()
        
        # Stop 
        while (time.time() - curr) < 10:
            stop = Bool(data=True)
            left_time = Float64(data=time.time() - curr)
            stop_pub.publish(stop)
            time_pub.publish(left_time)
            rate.sleep()
        
        curr = time.time()
        
        # Go 
        while (time.time() - curr) < 10:
            stop = Bool(data=False)
            left_time = Float64(data=time.time() - curr)
            stop_pub.publish(stop)
            time_pub.publish(left_time)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('rsu')
    stop_light()
