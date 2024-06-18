ROS Network
------
GazelleSim integrates into and is controlled in a ROS network.  Users may interface with GazelleSim via ROS topics, ROS services and the ROS parameter server.  The sections below outline the support GazelleSim communication protocols.


Published Topics
------
GazelleSim publishes sensor topics for each robot in the simulation.  Each topic begins the robot name defined in the robot_list parameter.  For each robot, Gazelle will publish:

* robot_name/camera/image_raw [sensor_msgs/Image]  The camera topic supports ROS standard Image message.  In addition, the associated compressed and packet image message are published.

* robot_name/gps [sensor_msgs/NavSatFix]  The gps topic supports gps NavSatFix messages to report the current robot CG location relative the (1) and origin of the playing field and (2) the orientation of the north vector.

* robot_name/odom [nav_msgs/Odometry]  The odom topic supports Odometry messages detailing the location and orientation of a robot at the CG.

* robot_name/scan [sensor_msgs/LaserScan] The scan topic supports LaserScan messages of the robot lidar system.  A total of 360 lidar points are published (1 degree increments) with 0 degrees aligned with the x-axis of the robot (forward direction)


Subscribed Topics
------
GazelleSim subscribes to robot command topics for robot in the simulation.  Each topic begins the robot name defined in the robot_list parameter.  For each robot, GazelleSim will subscribe to:

* robot_name/cmd_pos [geometry_msgs/Pose2D] - this topic may be used to place a robot at a given location and orientation.

* robot_name/cmd_vel [geometry_msgs/Twist] - this topic may be used to drive the robot.  msg.linear.x is used for the robot forward speed and msg.angular.z is used for the robot yaw rate.  Please note that GazelleSim will hold the last subscribed velocity command until another message is received.

Services
------
GazelleSim supports a number of services to manipulate the simulation environment during run time.  The available service are:

* delete - service to delete a robot from a simulation.  Required argument: robot name.

* list - service to generate the list of robots in the simulation.  No required arguments.

* refresh - service to reload the parameters that define a robot.  Required argument: robot name.

* spawn - service to spawn a new robot in the simulation.  Required arguments: robot name, x position, y position and angle orientation.

* teleport_absolute - service to teleport a robot to a new location and orientation.  Required arguments: robot name, x position, y position and angle orientation.

Next: [Creating GazelleSim Models](../creating_models/creating-models.md)
