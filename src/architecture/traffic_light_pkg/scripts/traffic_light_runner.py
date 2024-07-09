#!/usr/bin/env python3

import rospy

from std_msgs.msg import Time
import time

class TrafficLightRunner:
    """
    A node whose sole purpose is to keep the traffic light alive.

    This node only exists because I couldn't figure out how to call spin() while also
    maintaining a "forever" loop to publish time constantly.
    """

    run_pub: rospy.Publisher

    def __init__(self):
        rospy.init_node("traffic_light", anonymous=True)

        self.run_pub = rospy.Publisher(rospy.get_param("~run_light_topic"), Time, queue_size=1)

    def run(self):
        # TODO: This doesn't work. For some reason, changing the rate changes how the traffic
        #       light perceives the passage of time. I think that, instead of just publishing
        #       to keep the other node alive, this will *actually* be a time keeper that the other
        #       node will use o keep track of time itself. So, the traffic light's notion
        #       of time will depend on this node directly.
        #       
        #       Unless I can fix this shit. No idea why it's happening this way.
        while not rospy.is_shutdown():
            now_in_ns = time.time_ns()
            now_s = now_in_ns // 1_000_000_000
            now_ns = now_in_ns % 1_000_000_000
            now = Time(rospy.Time(now_s, now_ns))
            self.run_pub.publish(now)
            time.sleep(0.2)

if __name__ == "__main__":
    try:
        runner = TrafficLightRunner()
        runner.run()
    except rospy.ROSInterruptException:
        pass
