#!/usr/bin/env python3

# Import required Python code.
import rospy
from geometry_msgs.msg import Pose2D
import numpy as np



# Robot tester class definition
class CommandPosition():
    def __init__(self):

        # Define robot path information
        radius = 2.0
        x_robot = radius
        y_robot = 0.0
        theta_robot = np.pi
        freq = 0.1 
        
        # Create a command position publisher
        pub = rospy.Publisher('/robot1/cmd_pos', Pose2D, queue_size=1)

        # Define ROS rate
        rate = rospy.Rate(10)

        # Get the starting reference time
        time_0 = rospy.get_time()

        while not rospy.is_shutdown():

            # Compute robot position
            t = rospy.get_time() - time_0
            x = radius * np.cos(2*np.pi*freq*t)
            y = radius * np.sin(2*np.pi*freq*t)
            theta = (np.pi/2.0 + 2*np.pi*freq*t) % (2*np.pi)

            # Build and publish messsage
            msg = Pose2D()
            msg.x = x
            msg.y = y
            msg.theta = theta
            pub.publish(msg)
            
            rate.sleep()

        return



#################
# Main function #
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('command_position_node')
    print("Command Position Node Initialized")
    
    # Start tester
    try:
        CommandPosition()
    except rospy.ROSInterruptException: pass
