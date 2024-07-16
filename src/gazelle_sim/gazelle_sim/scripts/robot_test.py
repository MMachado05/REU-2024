#!/usr/bin/env python3

# Import required Python code.
import rospy
import sys
import rospy
import math
import message_filters
import cv2 as cv
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt


from dynamic_reconfigure.server import Server
from gazelle_sim.cfg import GazelleSimConfig
#import dynamic_reconfigure.client

# Support functions
def interp0(x, xp, yp):
    """Zeroth order hold interpolation w/ same
    (base)   signature  as numpy.interp."""
    
    def func(x0):
        if x0 <= xp[0]:
            return yp[0]
        if x0 >= xp[-1]:
            return yp[-1]
        k = 0
        while x0 > xp[k]:
            k += 1
        return yp[k-1]
    
    if isinstance(x,float):
        return func(x)
    elif isinstance(x, list):
        return [func(x) for x in x]
    elif isinstance(x, np.ndarray):
        return np.asarray([func(x) for x in x])
    else:
        raise TypeError('argument must be float, list, or ndarray')


# Robot tester class definition
class RobotTester():
    def __init__(self):

        # Define robot path information
        self.Time = 0
        self.Xr = 0
        self.Yr = 0

        # Initialize dynamic configure
        self.dyn_config = []

        # Dynamic configuration
        #client = dynamic_reconfigure.client.Client("gazelle_sim",
        #                                           timeout=30,
        #                                           config_callback=self.dynamic_reconfig_callback)
        
        srv = Server(GazelleSimConfig, self.dynamic_reconfig_callback)
        
        # Define the image subscriber
        self.image_sub = rospy.Subscriber('camera_view', Image,
                                          self.camera_callback)

        # Define the odometry subscriber - gathering data too slow
        self.odom_sub = rospy.Subscriber('odom', Odometry,
                                         self.odom_callback)

        # Create a command velocity publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define ROS rate
        self.rate = rospy.Rate(10)

        # Get the starting reference time
        self.time_begin = rospy.Time.now()

        ##################################
        #
        # MAIN ROS LOOP
        # Execute while server is active
        #
        #################################
        rospy.spin()
        
        #while not rospy.is_shutdown():
            #self.drive_time_based_robot()
            #self.drive_to_waypoint(2.0, 1.0)
        #    self.rate.sleep()



    def dynamic_reconfig_callback(self, config, level):
        self.dyn_config = config
        print(config)
        rospy.loginfo("CONFIG UPDATE")
        return config

    def drive_to_waypoint(self, x, y):
        # Create a Twist message
        msg_cmd_vel = Twist()
    
        # Get the current velocity and steer
        Kv = 0.2
        msg_cmd_vel.linear.x = Kv * math.sqrt((self.Xr-x)**2. + (self.Yr-y)**2.)
        msg_cmd_vel.angular.z = math.atan2( y-self.Yr, x-self.Xr)
        self.pub.publish(msg_cmd_vel)
        rospy.loginfo('(%.2f,%.2f) -> (%2.f,%.2f)' % (self.Xr, self.Yr, x, y))
        rospy.loginfo('Vel/Steer: (%.2f,%.2f)' % (msg_cmd_vel.linear.x,
                                                  msg_cmd_vel.angular.z))
        

        
    def drive_time_based_robot(self):

        # Get the current time (relative to node start time)
        time_now = rospy.Time.now()
        duration = time_now - self.time_begin
        time_sec = duration.to_sec()

        # Define drive commmands as a function of time
        ang = 0.20 # L = 0.20 m, R = 1.0 m

        # Drive straight 2 m, reverse 1 m, then drive in circle
        time_array =   np.array([0.0, 1.0, 3.0, 4.0, 5.0, 6.0, 20.0])
        vel_array =    np.array([0.0, 1.0, 0.0,-1.0, 0.0, 1.0,  0.0])
        steer_array =  np.array([0.0, 0.0, 0.0, 0.0, 0.0, ang,  0.0])

        # Create a Twist message
        msg_cmd_vel = Twist()
    
        # Get the current velocity and steer
        msg_cmd_vel.linear.x = interp0(time_sec, time_array, vel_array)
        msg_cmd_vel.angular.z = interp0(time_sec, time_array, steer_array)
        self.pub.publish(msg_cmd_vel)


        # Plot the path data
        #if(time_sec > 20 and time_sec < 20.2):
        #    plt.cla()
        #    plt.plot(self.Xr, self.Yr, '.')
        #    plt.axis("equal")
        #    plt.draw()
        #    plt.pause(0.001)

            
    def odom_callback(self, odom_msg):
        # Set robot position
        self.Xr = odom_msg.pose.pose.position.x
        self.Yr = odom_msg.pose.pose.position.y
        
        # Append pose information
        #self.Xr.append(odom_msg.pose.pose.position.x)
        #self.Yr.append(odom_msg.pose.pose.position.y)
        return


    def birds_eye_transform(self, img ):
        # Define sizes
        height, width, depth = img.shape

        # Define camera parameters
        
        alpha = (self.dyn_config["alpha_x"] - 90)* np.pi/180.0
        beta = (self.dyn_config["beta_y"] - 90)* np.pi/180.0
        gamma = (self.dyn_config["gamma_z"] - 90)* np.pi/180.0
        dist =  self.dyn_config["birds_d"]
        focal = self.dyn_config["birds_f"]
        #rospy.loginfo("DATA %.2f, %.2f, %.2f, %.2f, %.2f" %
        #              (alpha, beta, gamma, dist, focal))
        
        # Build matrices
        A1 = np.array([ [1, 0, -width/2],
                        [0, 1, -height/2],
                        [0, 0, 0],
                        [0, 0, 1] ] )

        Rx = np.array([ [1,               0,                0, 0],
                        [0, math.cos(alpha), -math.sin(alpha), 0],
                        [0, math.sin(alpha),  math.cos(alpha), 0],
                        [0,               0,                0, 1] ])

        Ry = np.array([ [math.cos(beta), 0, -math.sin(beta), 0],
                        [             0, 1,               0, 0],
                        [math.sin(beta), 0,  math.cos(beta), 0],
                        [             0, 0,               0, 1] ])

        Rz = np.array([ [math.cos(gamma), -math.sin(gamma), 0, 0],
                        [math.sin(gamma),  math.cos(gamma), 0, 0],
                        [              0,                0, 1, 0],
                        [              0,                0, 0, 1] ])

        T =  np.array([ [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, dist],
                        [0, 0, 0, 1] ])

        K =  np.array([ [focal,     0,  width/2, 0],
                        [    0, focal, height/2, 0],
                        [    0,     0,        1, 0] ])

        #Transform = K * T * Rx * Ry * Rz * A1;
        Transform = np.matmul(K,np.matmul(T,np.matmul(Rx,np.matmul(Ry,np.matmul(Rz,A1)))))
                
        img_dst = cv.warpPerspective(img, Transform, (width,height))

        cv.namedWindow('Birds Eye', cv.WINDOW_NORMAL)
        cv.imshow('Birds Eye', img_dst)
        cv.waitKey(3)

        
        
    def camera_callback(self, rgb_msg):
        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        img_test = img.copy()

        self.birds_eye_transform(img_test )
        
        # Get the image details
        rows, cols, depth = img.shape

        # Get the contours of the lane lines
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, img_thres = cv.threshold(img_gray, 25, 255, 0)
        contours, hierarchy = cv.findContours(img_thres, cv.RETR_TREE,
                                              cv.CHAIN_APPROX_SIMPLE)

        # Check the contours
        if( len(contours) != 2 ):
            #rospy.logwarn("Could not identify lane lines")
            return
    
        # Compute and draw polynominal fit through contours
        lanes = []
        for con in contours:
            # Get contour points
            x = con[:,0,0]
            y = con[:,0,1]
    
            # Fit polynominal through the contour
            poly = np.polyfit(y, x, 2)
    
            # Get poly points
            xpts = range(rows)
            ypts = np.polyval(poly, xpts)

            # Plot the polynomial fit
            for ipt in range(rows):
                cv.circle(img_test,(int(ypts[ipt]),xpts[ipt]),5,(0,0,255),-1)

            # Append the lane line polynominal
            lanes.append(poly)

        # Compute and draw steering path image
        img_steer_path = np.zeros(shape=(rows,cols,1))
        xpts = range(rows)
        ypts = (np.polyval(lanes[0], xpts) + np.polyval(lanes[1], xpts))/2.0
        for ipt in range(rows):
            cv.circle(img_steer_path,(int(ypts[ipt]),xpts[ipt]), 10, 255, -1)
        
        # Use P control
        if( 1 ):
            target_row = int(0.75*rows)
            target_row = int(0.85*rows)
            #target_row = int(0.5*rows)
            error = ypts[ target_row ] - int(cols/2)
            #rospy.loginfo("ERROR: %.2f", error)
            cmd_angular_z = -error / 100.0;

            # Plot target on image
            cv.circle(img_steer_path,(int(cols/2),target_row), 10, 255, -1)

            
        # Create a Twist message
        msg_cmd_vel = Twist()
    
        # Get the current velocity and steer
        msg_cmd_vel.linear.x = 0.2 * 0.0
        msg_cmd_vel.angular.z = cmd_angular_z
        self.pub.publish(msg_cmd_vel)


        # Display images
        cv.namedWindow('Input Image', cv.WINDOW_NORMAL)
        cv.imshow('Input Image', img_test)
        cv.waitKey(3)

        cv.namedWindow('Steer Path Image', cv.WINDOW_NORMAL)
        cv.imshow('Steer Path Image', img_steer_path)
        cv.waitKey(3)
        return

    
# Main function.
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('robottester')
    print("Robot Tester Node Initialized")
    
    # Start tester
    try:
        ne = RobotTester()
    except rospy.ROSInterruptException: pass
