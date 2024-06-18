#!/usr/bin/env python3

# Import base ROS
import rospy

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.srv import Reconfigure
from gazelle_sim.cfg import GazelleRobotParamDynCfgConfig

# Import the GazelleSim robot refresh service
from gazelle_sim.srv import Refresh

# Import supporting libraries
import os
import math

####################################
# RobotParamAdjust class definition
####################################
class RobotParamAdjustNode():
    def __init__(self):
        """Robot Parameter Adjustment Node"""

        # Initialize dynamic reconfigure
        self.initialize = False
        self.robot_name = ''
        self.camera_z = 0.0
        self.camera_phi_deg = 0.0
        self.camera_f = 0

        # Set up dynamic reconfigure
        self.srv = Server(GazelleRobotParamDynCfgConfig,
                          self.dyn_reconfig_callback)
        
        # Define ROS rate
        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():


            if self.initialize:
                self.initialize = False

                params = { 'camera_z':self.camera_z,
                           'camera_phi_deg':self.camera_phi_deg,
                           'camera_f':self.camera_f }
                
                node_name = rospy.get_name()
                #rospy.loginfo('Name = %s' % tmp)
                #client = Client('robot_param_adjust_node')
                client = Client(node_name)
                client.update_configuration(params)
 
            self.rate.sleep()
            
        
        return


    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        # Create parameter names
        camera_z_param = os.path.join(config['robot_name'], 'camera_z')
        camera_f_param = os.path.join(config['robot_name'], 'camera_f')
        camera_phi_rad_param = os.path.join(config['robot_name'], 'camera_phi')
        
        if not (rospy.has_param( camera_z_param ) and \
                rospy.has_param( camera_phi_rad_param ) and \
                rospy.has_param( camera_f_param )):
            return config
        


        if( config['robot_name'] != self.robot_name ):
            #
            # Set parameters from the parameter server
            #
            
            # Set robot namespace
            self.robot_name = config['robot_name']
            
            # Get current parameters
            self.camera_z = rospy.get_param( camera_z_param )
            angle_rad = rospy.get_param( camera_phi_rad_param )
            self.camera_phi_deg = round( angle_rad*180.0/math.pi, 2)
            self.camera_f = rospy.get_param( camera_f_param )
            
            # Set initialization state
            self.initialize = True
                               
        else:
            #
            # Set parameters from dynamic reconfigure
            #
            if rospy.has_param( camera_z_param ):
                rospy.set_param( camera_z_param,
                                 config['camera_z'] )

            if rospy.has_param( camera_phi_rad_param ):
                rospy.set_param( camera_phi_rad_param,
                                 config['camera_phi_deg']*math.pi/180 )
            
            if rospy.has_param( camera_f_param ):
                rospy.set_param( camera_f_param,
                                 config['camera_f'] )

                
            rospy.wait_for_service('refresh')
            refresh = rospy.ServiceProxy('refresh', Refresh)
            try:
                resp1 = refresh(config['robot_name'])
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                

        return config
        

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node
    rospy.init_node('robot_param_adjust_node')
    print('Robot Param Adjust Node initialized')
    
    # Start node
    try:
        RobotParamAdjustNode()
    except rospy.ROSInterruptException:
        pass
