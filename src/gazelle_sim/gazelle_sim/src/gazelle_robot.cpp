/*
 * Copyright (c) 2023, Giuseppe DeRose Jr.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gazelle_sim/gazelle_robot.h"
#include "gazelle_sim/gazelle_sim.h"

///////////////////////////////////////////////////////////
// Gazelle Robot constructor for ROS parameter definition
///////////////////////////////////////////////////////////
GazelleRobot::GazelleRobot(std::string robot_name, ros::NodeHandle *nh)
  : robot_name_(robot_name), nh_(nh)
{
  // Set image transport
  image_transport::ImageTransport it_(*nh);
  it_vec_.push_back(it_);

  // Initialize the robot
  loadROSParameters(nh,true);
  initializeROS(nh);
  initializeRobotRefPoints();

  // Set position and velocity
  Xr_ = Xr_init_;
  Yr_ = Yr_init_;
  Theta_ = Theta_init_;
  linear_x_ = 0.0;
  angular_z_ = 0.0;
  omega_ = 0.0;

  // Set lidar location
  Xl_ = Xr_;
  Yl_ = Yr_;
}
// End of GazelleRobot



/////////////////////////////
// Gazelle Robot destructor
/////////////////////////////
GazelleRobot::~GazelleRobot()
{
  // NEED MEMORY CLEAN UP
  
  // Stop robot
  linear_x_ = 0.0;
  angular_z_ = 0.0;
}
// End of ~GazelleRobot



/*******************************************************************************
*
* Initialization functions
*
*******************************************************************************/

///////////////////////////////////////////////////////////
// Set robot default parameters and load robot parameters
///////////////////////////////////////////////////////////
bool GazelleRobot::loadROSParameters(ros::NodeHandle *nh, bool init_flag)
{
  // Define the robot and camera default locations (in plane)
  if( init_flag )
    setDefaultParameters();
  
  // Read ROS parameters
  if( !loadParameters() )
    ROS_WARN("Robot [%s] missing robot_type_ parameter - default value used",
	     robot_name_.c_str() );

  return true;
}
// End of loadROSParameters



//////////////////////////////////////////////////
// Initialize the ROS publishers and subscribers
//////////////////////////////////////////////////
bool GazelleRobot::initializeROS(ros::NodeHandle *nh)
{
  // Initialize odometry
  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
		      0, 0.1,   0,   0,   0, 0,
		      0,   0, 1e6,   0,   0, 0,
		      0,   0,   0, 1e6,   0, 0,
		      0,   0,   0,   0, 1e6, 0,
		      0,   0,   0,   0,   0, 0.2};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  // Define camera view points
  img_cam_ = cv::Mat::zeros(cv::Size(camera_img_height_,camera_img_width_),
			    CV_8UC1);
  for( int i = 0; i < 4; i++ )
    {
      camera_corners_[i] = cv::Point2f(0.0,0.0);
      camera_corners_img_[i] = cv::Point2i(0,0);
    }


  // Initialize publishers
  std::string camera_img_topic = robot_name_ + "/camera/image_raw";
  camera_view_pub_ = it_vec_[0].advertise(camera_img_topic, 50);

  std::string odom_topic = robot_name_ + "/odom";
  odom_pub_= nh->advertise<nav_msgs::Odometry>(odom_topic, 50);

  std::string lidar_topic = robot_name_ + "/scan";
  lidar_scan_pub_ = nh->advertise<sensor_msgs::LaserScan>(lidar_topic, 50);

  std::string gps_topic = robot_name_ + "/gps";
  gps_pub_ = nh->advertise<sensor_msgs::NavSatFix>(gps_topic, 50);
  
  // Initialize subscribers
  std::string cmd_vel_topic = robot_name_ + "/cmd_vel";
  cmd_vel_sub_  = nh->subscribe(cmd_vel_topic, 50,
				 &GazelleRobot::cmdVelocityCb, this);
    
  std::string cmd_pos_topic = robot_name_ + "/cmd_pos";
  cmd_pos_sub_  = nh->subscribe(cmd_pos_topic, 50,
				&GazelleRobot::cmdPositionCb, this);
    

  return true;
}
// End of GazelleRobot::initializeROS



/////////////////////////////////////////////////////////////////////////
// Initialize the rotation matrix and the camera/lidar reference points
/////////////////////////////////////////////////////////////////////////
void GazelleRobot::initializeRobotRefPoints( void )
{
  // Initialize robot and sensors
  setRotationZ();
  setRobotRefPoints();
  setLidarGlobalLocation();
  getRefCameraCorners();
  initializePen();
}
// End of initializeRobotRefPoints


/*******************************************************************************
*
* Parameter functions
*
*******************************************************************************/

///////////////////////////////
// Load ROS integer parameter
///////////////////////////////
bool GazelleRobot::loadROSIntParameter(std::string topic, int &var )
{
  
  std::string param = robot_name_ + "/" + topic;
  if(nh_->hasParam(param)) {
    ros::param::get(param, var);
    return true;
  }
  return false;
}
// End of loadROSIntParameter


//////////////////////////////
// Load ROS double parameter
//////////////////////////////
bool GazelleRobot::loadROSDoubleParameter(std::string topic, double &var )
{
  
  std::string param = robot_name_ + "/" + topic;
  if(nh_->hasParam(param)) {
    ros::param::get(param, var);
    return true;
  }
  return false;
}
// End of loadROSDoubleParameter



//////////////////////////////
// Load ROS string parameter
//////////////////////////////
bool GazelleRobot::loadROSStringParameter(std::string topic, std::string &var )
{
  
  std::string param = robot_name_ + "/" + topic;
  if(nh_->hasParam(param)) {
    ros::param::get(param, var);
    return true;
  }
  return false;
}
// End of loadROSStringParameter



///////////////////////////
// Set default parameters
//////////////////////////
void GazelleRobot::setDefaultParameters()
{

  // Robot location variables
  Xr_init_ = 0.0;
  Yr_init_ = 0.0;
  Theta_init_= 0.0;
  Xr_ = 0.0;
  Yr_ = 0.0;
  Theta_ = 0.0;
  cv::Matx22f Rz_(1, 0, 0, 1);

  // Robot parameters
  robot_type_ = DIFF_DRIVE_CIRC;
  robot_radius_ = 0.200;
  rwheel_ = 0.070;
  wheel_width_ = 0.040;
  trackwidth_ = 0.340;
  axle_dst_ = 0.000;
  robot_width_ = 0.200;
  robot_length_ = 0.300;
  wheelbase_ = 0.20;
  max_steer_ = CV_PI/6.0;

  // GPS variables
  latitude_ = latitude_base_;
  longitude_ = longitude_base_;

  double ang_rad = north_angle_ - CV_PI/2.0;
  Rz_gps_vel_ = cv::Matx22f( cos(ang_rad), -sin(ang_rad),
			     sin(ang_rad),  cos(ang_rad));;
  gps_ned_vel_ = cv::Point2f(0.0,0.0);
  
  // Camera parameters and location - will not draw camera
  Zc_ = 0.5;
  camera_phi_ = 60.0*CV_PI/180.0;
  camera_f_ = 700;
  camera_img_width_ = 640;
  camera_img_height_ = 480;
  camera_width_ = 0.05; 
  camera_length_= 0.10;
  camera_x_ref_ = 0.18;
  camera_y_ref_ = 0.0;

  // Lidar locations and parameters - will not draw lidar
  Xl_ = Xr_;
  Yl_ = Yr_;
  lidar_x_ref_ = 0.0;
  lidar_y_ref_ = 0.0;
  lidar_radius_ref_ = 0.05;

  // Robot parameters
  wheelbase_ = 1.0;
  max_steer_ = CV_PI/6.0;

  // Pen parameters
  pen_width_ = 2;
  
  // Colors - BGR
  robot_color_ = cv::Scalar(  0, 125,   0);
  tire_color_  = cv::Scalar( 30,  30,  30);
  camera_color_= cv::Scalar(  0,   0, 255);
  lidar_color_ = cv::Scalar(255,   0,   0);
}
// End of set DefaultParameters



////////////////////////////
// Load the ROS parameters
////////////////////////////
bool GazelleRobot::loadParameters()
{
  // Get the robot type
  std::string robot_str;
  std::string type_param = robot_name_ + "/robot_type";
  if(nh_->hasParam(type_param))
    {
      ros::param::get(type_param, robot_str);
      if( robot_str.compare("diff_drive_rect") == 0 )
	robot_type_ = DIFF_DRIVE_RECT;
      
      else if( robot_str.compare("diff_drive_circ") == 0 )
	robot_type_ = DIFF_DRIVE_CIRC;
      
      else if( robot_str.compare("acker_steer_rect") == 0 )
	robot_type_ = ACKER_STEER_RECT;
      
      else
	{
	  ROS_WARN_STREAM("Invalid parameter " + type_param);
	  return false;
	}
    }
  else
    {
      ROS_WARN_STREAM("Missing parameter " + type_param);
      return false;
    }       
  
  // Load robot initial location
  loadROSDoubleParameter("Xr_init", Xr_init_);
  loadROSDoubleParameter("Yr_init", Yr_init_);
  double tmp_theta;
  if( loadROSDoubleParameter("Theta_init", tmp_theta) )
    {
      loadROSDoubleParameter("Theta_init", Theta_init_);
      Theta_init_ = Theta_init_;
    }

  // Robot size parameters
  loadROSDoubleParameter("robot_width", robot_width_);
  loadROSDoubleParameter("robot_length", robot_length_);
  loadROSDoubleParameter("robot_radius", robot_radius_);
  loadROSDoubleParameter("rwheel", rwheel_);
  loadROSDoubleParameter("wheel_width", wheel_width_);
  loadROSDoubleParameter("trackwidth", trackwidth_);
  loadROSDoubleParameter("axle_dst", axle_dst_);
  loadROSDoubleParameter("wheelbase", wheelbase_);
  loadROSDoubleParameter("max_steer", max_steer_);

  // Camera location and dimensions
  loadROSDoubleParameter("camera_z", Zc_);
  double tmp_phi;
  if( loadROSDoubleParameter("camera_phi", tmp_phi) ) {
    loadROSDoubleParameter("camera_phi", camera_phi_);
    camera_phi_ = camera_phi_;
  }
  loadROSIntParameter("camera_f", camera_f_);
  loadROSIntParameter("camera_img_width", camera_img_width_);
  loadROSIntParameter("camera_img_height", camera_img_height_);
  loadROSDoubleParameter("camera_x", camera_x_ref_);
  loadROSDoubleParameter("camera_y", camera_y_ref_);
  loadROSDoubleParameter("camera_width", camera_width_);
  loadROSDoubleParameter("camera_length", camera_length_);

  
  // Lidar location and dimensions
  loadROSDoubleParameter("lidar_x", lidar_x_ref_);
  loadROSDoubleParameter("lidar_y", lidar_y_ref_);
  loadROSDoubleParameter("lidar_r", lidar_radius_ref_);

  // Pen width parameter
  loadROSIntParameter("pen_width", pen_width_);
  
  // Load colors
  std::string color_param = robot_name_ + "/robot_color";
  if(nh_->hasParam(color_param))
    {
      std::vector<int> tmp;
      ros::param::get(color_param, tmp);
      if( tmp.size() == 3 )
	robot_color_ = cv::Scalar(tmp[2], tmp[1], tmp[0]);
    }

  color_param = robot_name_ + "/tire_color";
  if(nh_->hasParam(color_param))
    {
      std::vector<int> tmp;
      ros::param::get(color_param, tmp);
      if( tmp.size() == 3 )
	tire_color_ = cv::Scalar(tmp[2], tmp[1], tmp[0]);
    }

  color_param = robot_name_ + "/camera_color";
  if(nh_->hasParam(color_param))
    {
      std::vector<int> tmp;
      ros::param::get(color_param, tmp);
      if( tmp.size() == 3 )
	camera_color_ = cv::Scalar(tmp[2], tmp[1], tmp[0]);
    }

  color_param = robot_name_ + "/lidar_color";
  if(nh_->hasParam(color_param))
    {
      std::vector<int> tmp;
      ros::param::get(color_param, tmp);
      if( tmp.size() == 3 )
	lidar_color_ = cv::Scalar(tmp[2], tmp[1], tmp[0]);
    }

  // Successful completion
  return true;
}
// End of loadParameters




/////////////////////////////
// Check the ROS parameters    ===>>> NOT USED CURRENTLY <<<===
/////////////////////////////
bool GazelleRobot::checkParameters(std::string rbt_name, std::string err_str)
{
  // Required paramter list
  std::vector<std::string> req_non_zero_params;
  std::vector<std::string> req_defined_params;
  
  // Get the robot type
  int rbt_type;
  std::string robot_str;
  std::string type_param = rbt_name + "/robot_type";
  if(nh_->hasParam(type_param))
    {
      ros::param::get(type_param, robot_str);
      if( robot_str.compare("diff_drive_rect") == 0 )
	{
	  rbt_type = DIFF_DRIVE_RECT;
	  std::vector<std::string> diff_rect_params = { "/robot_width",
							"/robot_length",
							"/axle_dst"};
	}
      else if( robot_str.compare("diff_drive_circ") == 0 )
	{
	  rbt_type = DIFF_DRIVE_CIRC;
	  std::vector<std::string> diff_circ_params = {"/robot_radius" };
	}
      else if( robot_str.compare("acker_steer_rect") == 0 )
	{
	  rbt_type = ACKER_STEER_RECT;
	  std::vector<std::string> acker_params = { "/robot_width",
						    "/robot_length",
						    "/axle_dst",
						    "/wheelbase",
						    "/max_steer"};
	}
      else
	{
	  err_str = "[" + rbt_name + "]" "must have a valid robot_type defined.";
	  return false;
	}
    }
  else
    {
      err_str = "[" + rbt_name + "]" "missing robot_type parameter.";
      return false;
    }       

  // Get sensing variables
  std::vector<std::string> sensing_params = { "camera_x", "camera_y",
					      "camera_width", "camera_length",
					      "camera_z", "camera_phi",
					      "camera_f",
					      "lidar_x", "lidar_y", "lidar_r" };
  
  // Successful completion
  return true;
}
// End of checkParameters



/////////////////////////
// Set robot parameters    ===>>> NOT USED CURRENTLY <<<===
/////////////////////////
void GazelleRobot::setParameters( std::string robot_type,
				  double x, double y, double theta,
				  double robot_width, double robot_length,
				  double robot_radius, double wheelbase,
				  double max_steer, double rwheel,
				  double wheel_width, double trackwidth,
				  double axle_dst,
				  double camera_x, double camera_y,
				  double camera_z, double camera_phi,
				  double camera_f,
				  double camera_width, double camera_length,
				  double lidar_x, double lidar_y,
				  double lidar_r )
{
  // Get the robot type
  if( robot_type.compare("diff_drive_rect") == 0 )
    robot_type_ = DIFF_DRIVE_RECT;
  else if( robot_type.compare("diff_drive_circ") == 0 )
    robot_type_ = DIFF_DRIVE_CIRC;
  else if( robot_type.compare("acker_steer_rect") == 0 )
    robot_type_ = ACKER_STEER_RECT;
  else
    {
      ROS_ERROR_STREAM("Invalid parameter " + robot_type);
      return;
    }

  // Set robot initial location
  Xr_init_ = x;
  Yr_init_ = y;
  Theta_init_ = theta;


  // Set the robot size parameters
  robot_width_ = robot_width;
  robot_length_ = robot_length;
  robot_radius_ = robot_radius;
  rwheel_ = rwheel;
  wheel_width_ = wheel_width;
  trackwidth_ = trackwidth;
  axle_dst_ = axle_dst;
  wheelbase_ = wheelbase;
  max_steer_ = max_steer;

  // Camera location and dimensions
  camera_x_ref_ = camera_x;
  camera_y_ref_ = camera_y;
  Zc_ = camera_z;
  camera_phi_ = camera_phi;
  camera_f_ = camera_f;
  camera_width_ = camera_width;
  camera_length_ = camera_length;

  // Lidar location and size
  lidar_x_ref_ = lidar_x;
  lidar_y_ref_ = lidar_y;
  lidar_radius_ref_ = lidar_r;

}
// End of GazelleRobot::setParameters




/*******************************************************************************
*
* ROS callback functions
*
*******************************************************************************/

///////////////////////////////
// Callback: Command Velocity
///////////////////////////////
void GazelleRobot::cmdVelocityCb(const geometry_msgs::Twist::ConstPtr &msg)
{
  linear_x_ = msg->linear.x;
  angular_z_ = msg->angular.z;
}
// End of GazelleRobot::cmdVelocityCb



///////////////////////////////
// Callback: Command Position
///////////////////////////////
void GazelleRobot::cmdPositionCb(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  Xr_ = msg->x;
  Yr_ = msg->y;
  Theta_ = msg->theta;
}
// End of GazelleRobot::cmdPositionCb




/*******************************************************************************
*
* Publish functions
*
*******************************************************************************/

///////////////////////////
// Publish the lidar scan
///////////////////////////
void GazelleRobot::publishLidarScan(float lidar_max_dist)
{

  // Setting
  unsigned int num_readings = 360;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];
  
  // Get the current time
  ros::Time scan_time = ros::Time::now();
  
  // Populate the LaserScan message
  sensor_msgs::LaserScan scan;
  scan.header.stamp = scan_time;
  //scan.header.frame_id = "odom";
  scan.header.frame_id = "base_link";
  scan.angle_min = 0;
  scan.angle_max = 6.28;
  scan.angle_increment = 6.28 / 360;
  scan.time_increment = (1 / laser_frequency) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  int theta_off = (int) round(Theta_*180.0/CV_PI);
  for(unsigned int i = 0; i < num_readings; ++i)
    {
      int idx = (i+theta_off+360)%360;
      //ROS_INFO("(%.2f, %3d): %3d -> %3d", Theta_*180.0/CV_PI, theta_off, i, idx);
      //idx = i;
      if( scan_data_[idx] > 0.0 && scan_data_[idx] < lidar_max_dist )
	{
	  scan.ranges[i] = scan_data_[idx];
	  scan.intensities[i] = 100.0;
	}
      else
	{
	  scan.ranges[i] = 0.0;
	  scan.intensities[i] = 0.0;
	}
    }

  // Publish message
  lidar_scan_pub_.publish(scan);

}
// End of publishLidarScan



/////////////////////////////
// Publish the GPS location
/////////////////////////////
void GazelleRobot::publishGPS()
{
  
  // Get the current time
  ros::Time cur_time = ros::Time::now();
  
  // Populate the GPS message
  sensor_msgs::NavSatFix msg;
  msg.header.stamp = cur_time;
  msg.header.frame_id = "";
  msg.latitude = latitude_;
  msg.longitude = longitude_;
  msg.altitude = NAN;
  msg.position_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  msg.position_covariance_type = 0u; // COVARIANCE_TYPE_UNKNOWN

  // Publish message
  gps_pub_.publish(msg);

}
// End of publishGPS



/////////////////////////////////
// Publish the robot transforms
/////////////////////////////////
void GazelleRobot::publishTransform( const ros::Time time_now )
{

  // Set robot location
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = time_now;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = Xr_;
  transformStamped.transform.translation.y = Yr_;
  transformStamped.transform.translation.z = 0.0;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(Theta_);
  transformStamped.transform.rotation.x = q.x;
  transformStamped.transform.rotation.y = q.y;
  transformStamped.transform.rotation.z = q.z;
  transformStamped.transform.rotation.w = q.w;

  // Publish the base_link transform
  tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(transformStamped); 


  // Define the odom transform
  odom_.header.stamp = time_now;
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "odom";
  odom_.pose.pose.position.x = Xr_;
  odom_.pose.pose.position.y = Yr_;
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = q;
  odom_.twist.twist.linear.x  = linear_x_;
  odom_.twist.twist.angular.z = omega_;
  
  // Publish odometry message
  odom_pub_.publish(odom_);

}
// End of publishTransform



/*******************************************************************************
*
* Utility functions
*
*******************************************************************************/

//////////////////////////
// Return the robot name
//////////////////////////
std::string GazelleRobot::getName( void )
{
  return robot_name_;
}
// ENd of getName



////////////////////////////////////////////
// Return if the robot body is rectangular
////////////////////////////////////////////
bool GazelleRobot::isRectBody( void )
{
    if( robot_type_ == DIFF_DRIVE_RECT || robot_type_ == ACKER_STEER_RECT )
      return true;

    return false;
}
// End of isRectBody



///////////////////////////////////////
// Return the rectangular body points
///////////////////////////////////////
void GazelleRobot::getRobotBoundaryRect( std::vector<std::vector<cv::Point2f>> &rect_vec,
					 int idx )
{
  cv::Point2f robot_loc( Xr_, Yr_ );
  for( auto i = 0; i < 4; i++ )
    {
      cv::Point2f tmp_pt = Rz_*body_corners_ref_[i]+robot_loc;
      rect_vec[idx][i].x = tmp_pt.x;
      rect_vec[idx][i].y = tmp_pt.y;
    }
}
// End of getRobotBoundaryRect



////////////////////////////////////
// Return the circular body points
////////////////////////////////////
void GazelleRobot::getRobotBoundaryCirc(std::vector<cv::Point2f> &center_vec,
					std::vector<double> &radius_vec,
					int idx )
{
  center_vec[idx].x = Xr_;
  center_vec[idx].y = Yr_;
  radius_vec[idx] = robot_radius_;
}
// End of getRobotBoundaryCirc



///////////////////////////
// Set the location (X,Y)
///////////////////////////
void GazelleRobot::setLocation(double x, double y)
{
  // Set previous location
  Xr_prev_ = Xr_;
  Yr_prev_ = Yr_;

  // Update location
  Xr_ = x;
  Yr_ = y;
}
// End of setLocation



/////////////////////////////////////
// Set the location (X,Y) and Theta
/////////////////////////////////////
void GazelleRobot::setLocation(double x, double y, double theta)
{
  // Set previous location
  Xr_prev_ = Xr_;
  Yr_prev_ = Yr_;
  Theta_prev_ = Theta_;

  // Update location
  Xr_ = x;
  Yr_ = y;
  Theta_ = theta;
}
// End of setLocation



//////////////////////////////////////
// Rotate the robot by a given angle
//////////////////////////////////////
void GazelleRobot::incrementRotation(double inc_theta)
{
  Theta_ = Theta_ + inc_theta;
}
// End of incrementRotation



/*******************************************************************************
*
* Drawing functions
*
*******************************************************************************/

///////////////////
// Draw the robot 
///////////////////
void GazelleRobot::drawRobot(cv::Mat &img_field, int color_flag,
			     double pixels_per_meter)
{
  cv::Point2f robot_loc( Xr_, Yr_ );
  cv::Point2i coords_img[4];

  // Draw robot using reference points
  // ==>> ONLY NEEDED WITH PARAMETER UPDATE <<==
  setRobotRefPoints();
  
  // Draw robot body
  if( robot_type_ == DIFF_DRIVE_RECT || robot_type_ == ACKER_STEER_RECT )
    {
      for( int i = 0; i < 4; i++ )
	{
	  ::real_to_image_coords( Rz_*body_corners_ref_[i]+robot_loc,
				  coords_img[i] );
	}

      drawFilledPoly(img_field, coords_img,
		     (color_flag==0) ? robot_color_ : cv::Scalar(color_flag));
      
    }
  else if( robot_type_ == DIFF_DRIVE_CIRC )
    {
      cv::Point2i center_img, radius_img;
      ::real_to_image_coords( robot_loc, center_img );
      
      int radius = (int) (robot_radius_ * pixels_per_meter);
      cv::circle(img_field, center_img, radius,
		 (color_flag==0) ? robot_color_ : cv::Scalar(color_flag), -1);
    }

  // Draw the drive/rear wheels
  if( wheel_width_ > 0.0 )
    {
      drawTire(img_field, lr_whl_ref_, color_flag);
      drawTire(img_field, rr_whl_ref_, color_flag);
    }
  

  // Draw the front wheels for ackerman steer
  if( robot_type_ == ACKER_STEER_RECT )
    {
      cv::Point2f whl_ref[4];
      cv::Point2f wb( wheelbase_, 0.0 );
      double lf_steer = 0;
      double rf_steer = 0;
      cv::Matx22f Rz_lf(1, 0,
			0, 1);
      cv::Matx22f Rz_rf(1, 0,
			0, 1);

      // Update steer rotation matrices for turning robot
      if( steer_angle_ != 0.0 )
	{
	  double steer_dir = 1.0;
	  if( steer_angle_ < 0.0 ) steer_dir = -1.0;
	  double turn_radius = wheelbase_ / fabs(steer_angle_);
	  lf_steer = steer_dir * wheelbase_ /
	    ( turn_radius - steer_dir*(trackwidth_/2.0));
	  
	  rf_steer = steer_dir * wheelbase_ /
	    ( turn_radius + steer_dir*(trackwidth_/2.0));

	  Rz_lf = cv::Matx22f(cos(lf_steer), -sin(lf_steer),
			      sin(lf_steer), cos(lf_steer));
	  Rz_rf = cv::Matx22f(cos(rf_steer), -sin(rf_steer),
			      sin(rf_steer), cos(rf_steer));
	}

      // Steer and draw lf tire
      for(int i = 0; i < 4; i++ )
	whl_ref[i] = Rz_lf * lf_whl_ref_[i];

      for(int i = 0; i < 4; i++ )
	whl_ref[i] = lf_whl_center_ref_ + whl_ref[i];

      drawTire(img_field, whl_ref, color_flag);

      // Steer and draw rf tire
      for(int i = 0; i < 4; i++ )
	whl_ref[i] = Rz_rf * rf_whl_ref_[i];

      for(int i = 0; i < 4; i++ )
	whl_ref[i] = rf_whl_center_ref_ + whl_ref[i];

      drawTire(img_field, whl_ref, color_flag);

    }

     
  // Camera
  if( camera_width_ > 0.0 && camera_length_ > 0.0 && color_flag == 0)
    {
      for(int i = 0; i < 4; i++ )
	{
	  ::real_to_image_coords( Rz_*camera_pts_ref_[i]+robot_loc,
				  coords_img[i] );
	}
      drawFilledPoly(img_field, coords_img, camera_color_ );
    }

  // Lidar
  if( lidar_radius_ref_ > 0.0  && color_flag == 0)
    {
      cv::Point2i center_img, radius_img;
      ::real_to_image_coords( Rz_*cv::Point2f(lidar_x_ref_, lidar_y_ref_) +
			      robot_loc, center_img );
      
      int radius = (int) (lidar_radius_ref_ * pixels_per_meter);
      cv::circle(img_field, center_img, radius , lidar_color_, -1 );
    }
	     
  // Testing:end

}
// End of drawRobot




//////////////
// Draw tire
//////////////
void GazelleRobot::drawTire(cv::Mat &img_field, cv::Point2f *whl_ref,
			    int color_flag)
{
  cv::Point2f robot_loc(Xr_,Yr_);
  cv::Point2i coords_img[4];
    
  // LR tire points
  for(int i = 0; i < 4; i++ )
    {
      cv::Point2f tmp = Rz_*whl_ref[i] + robot_loc;
      //ROS_INFO("Robot Loc = (%f,%f)", robot_loc.x, robot_loc.y);
      //ROS_INFO("Tire ref %d = (%f,%f)", i, tmp.x, tmp.y );
      ::real_to_image_coords( tmp, coords_img[i] );
      //ROS_INFO("Tire cords %d = (%d,%d)", i, coords_img[i].x, coords_img[i].y );
    }
  drawFilledPoly(img_field, coords_img,
		 (color_flag==0) ? tire_color_ : cv::Scalar(color_flag));
  
  cv::Point2i center = 0.5*(coords_img[0] + coords_img[3]);
  int radius = (int) sqrt( pow(center.x-coords_img[0].x,2) +
			   pow(center.y-coords_img[0].y,2) );
  cv::circle(img_field, center, radius,
	     (color_flag==0) ? tire_color_ : cv::Scalar(color_flag), -1);
  
  center = 0.5*(coords_img[1] + coords_img[2]);
  radius = (int) sqrt( pow(center.x-coords_img[1].x,2) +
		       pow(center.y-coords_img[1].y,2) );
  cv::circle(img_field, center, radius,
	   (color_flag==0) ? tire_color_ : cv::Scalar(color_flag), -1);
 
}
// End of drawTire



/////////////////////////////////////////////////////
// Draw border around carmera view on playing field
/////////////////////////////////////////////////////
void GazelleRobot::drawOutlineCameraView(cv::Mat &img_field)
{
  // Get the corners of the camera view
  getCameraCorners( );

  // Draw camera view frame on field (after camera output image)
  for (int i = 0; i < 4; i++)
    {
      cv::line(img_field, camera_corners_img_[i], camera_corners_img_[(i+1)%4],
	       camera_color_, 4);
    }
}
// End of GazelleRobot::drawOutlineCameraView



/////////////////////////////
// Draw the lidar scan data
/////////////////////////////
void GazelleRobot::drawLidarScan(cv::Mat &img_field,
				 float lidar_max_dist)
{
  // Get the source location
  cv::Point2i src;
  ::real_to_image_coords(cv::Point2f(Xr_, Yr_), src);

  for(int theta = 0; theta < 360; theta++)
    {
      if( scan_data_[theta] > 0.0 && scan_data_[theta] < lidar_max_dist_ )
	{
	  cv::circle(img_field,scan_loc_img_[theta],lidar_marker_size_,
		     lidar_color_, -1);
	}
      
    }
  
}
// End of GazelleRobot::drawLidarScan



//////////////////////
// Draw the pen path
//////////////////////
void GazelleRobot::drawPenPath(cv::Mat &img_field)
{
  if( pen_points_.size() < 2 ) return;

  for( auto i = 1; i < pen_points_.size(); i++ )
    {
      cv::line(img_field, pen_points_[i-1], pen_points_[i],
	       robot_color_, pen_width_);
    }
}
// End of drawPenPath




/*******************************************************************************
*
* Pen functions
*
*******************************************************************************/

//////////////////////////////
// Set up the pen parameters
//////////////////////////////
void GazelleRobot::initializePen()
{
  // Set up pen points vector
  pen_points_.clear();
  
  //cv::Point2f robot_loc( Xr_, Yr_ );
  //cv::Point2i pt;
  //::real_to_image_coords( robot_loc, pt );

  //pen_points_.push_back( pt );

  // Set the pen color
  //pen_color_ = cv::Scalar(255,255,255);
}
// End of initializePen



/////////////////////////
// Add a pen path point
/////////////////////////
void GazelleRobot::addPenPoint()
{
  // Compute current location
  cv::Point2f robot_loc( Xr_, Yr_ );
  cv::Point2i pt;
  ::real_to_image_coords( robot_loc, pt );

  // Add initial point if empty
  if( pen_points_.size() < 1 )
    {
      pen_points_.push_back(pt);
      return;
    }
  
  // Add to pen point list if the robot moved
  if( pt != pen_points_.back() )
    pen_points_.push_back(pt);
}
// End of addPenPoint



/*******************************************************************************
*
* Reference frame functions
*
*******************************************************************************/

//////////////////////////////////////
// Set the global rotation matrix Rz
//////////////////////////////////////
void GazelleRobot::setRotationZ()
{
   Rz_ = cv::Matx22f( cos(Theta_), -sin(Theta_),
		      sin(Theta_),  cos(Theta_));
}
// End of setRotationZ



//////////////////////////////////
// Set the lidar global location
//////////////////////////////////
void GazelleRobot::setLidarGlobalLocation()
{
  cv::Point2f lidar_loc;
  lidar_loc = Rz_*cv::Point2f(lidar_x_ref_,lidar_y_ref_) + cv::Point2f(Xr_,Yr_);
  Xl_ = lidar_loc.x;
  Yl_ = lidar_loc.y;
}
// End of setLidarGlobalLocation



///////////////////////////////////
// Set the robot reference points
///////////////////////////////////
void GazelleRobot::setRobotRefPoints()
{
  // Initiaize signs for finding rectangular endpoints
  double x_signs[] = {-1, 1,  1, -1};
  double y_signs[] = { 1, 1, -1, -1};

  // Set the robot body reference points
  if( robot_type_ == DIFF_DRIVE_RECT || robot_type_ == ACKER_STEER_RECT )
    {
      for( int i = 0; i < 4; i++ )
	{
	  body_corners_ref_[i] = cv::Point2f(x_signs[i] * robot_length_/2.0 +
					     axle_dst_,
					     y_signs[i]*robot_width_/2.0 );
	}
    }

  // Set the drive/rear left and right wheel reference points
  for(int i = 0; i < 4; i++ )
    {
      
      lr_whl_ref_[i] = cv::Point2f(x_signs[i]*(rwheel_-wheel_width_/2.0),
				   trackwidth_/2.0 +
				   y_signs[i]*wheel_width_/2.0);
      
      rr_whl_ref_[i] = cv::Point2f(x_signs[i]*(rwheel_-wheel_width_/2.0),
				   -trackwidth_/2.0 +
				   y_signs[i]*wheel_width_/2.0);
    }

  // Set the front left and right wheel centers
  lf_whl_center_ref_ = cv::Point2f(wheelbase_,trackwidth_/2.0);
  rf_whl_center_ref_ = cv::Point2f(wheelbase_,-trackwidth_/2.0);
  for(int i = 0; i < 4; i++ )
    {
      lf_whl_ref_[i] = -lf_whl_center_ref_ +
	cv::Point2f(wheelbase_ + x_signs[i]*(rwheel_-wheel_width_/2.0),
		    trackwidth_/2.0 +
		    y_signs[i]*wheel_width_/2.0 );

      rf_whl_ref_[i] = -rf_whl_center_ref_ +
	cv::Point2f(wheelbase_ + x_signs[i]*(rwheel_-wheel_width_/2.0),
		    -trackwidth_/2.0 +
		    y_signs[i]*wheel_width_/2.0 );
    }

  double x_vals[] = {-1, 0,  0, -1};
  // Set the camera reference points
  for(int i = 0; i < 4; i++ )
    {
      camera_pts_ref_[i] = cv::Point2f(camera_x_ref_ +
				       x_vals[i]*camera_length_,
				       camera_y_ref_ +
				       y_signs[i]*camera_width_/2.0);
    }
  
}
// End of setRobotRefPoints




/*******************************************************************************
*
* Camera functions
*
*******************************************************************************/
/////////////////////
// Get camera image
/////////////////////
void GazelleRobot::getCameraImage(cv::Mat &img_field,
				  bool exterior_walls,
				  cv::Scalar wall_color)
{
  // Get the corners of the camera view
  getCameraCorners( );

  // Get the camera view image
  cv::Point2f field_cam[4], out_cam[4];
  field_cam[0] = camera_corners_img_[0];
  field_cam[1] = camera_corners_img_[1];
  field_cam[2] = camera_corners_img_[2];
  field_cam[3] = camera_corners_img_[3];
  out_cam[0] = cv::Point2f(   0,   0 );
  out_cam[1] = cv::Point2f( camera_img_width_,   0 );
  out_cam[2] = cv::Point2f( camera_img_width_, camera_img_height_ );
  out_cam[3] = cv::Point2f(                 0, camera_img_height_ );

  // ==> INCOMPLETE <==
  // ==> WORK TO UPDATE CAMERA VIEW <==
  // Determine if any obstructions are in the camera view
  //for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
  //{
  //  for( int j = 0; j < 4; j++ )
  //	{
  //	  if( ::pointInsidePolygon(camera_corners_, 4, rect_obs_corners_[i][j]))
  //	    ;
  //	    //ROS_INFO("Rect %d inside camera view", i );
  //	}
  //}
   
  // Get the Perspective Transform Matrix i.e. lambda
  cv::Mat lambda( 2, 4, CV_32FC1 );
  lambda = cv::getPerspectiveTransform( field_cam, out_cam );
  
  // Apply the Perspective Transform just found to the src image
  cv::Scalar exterior_color(0,0,0);
  if( exterior_walls )
    exterior_color = wall_color;
  
  cv::warpPerspective(img_field,img_cam_,lambda,
		      cv::Size(camera_img_width_, camera_img_height_),
		      cv::INTER_LINEAR, cv::BORDER_CONSTANT,
		      exterior_color );
		      
}
// End of getCameraImage



///////////////////////////
// Get the camera corners
///////////////////////////
void GazelleRobot::getCameraCorners()
{
  // Set up rotation matrices
  cv::Point2f offset(Xr_, Yr_);

  cv::Point2f corners_rot[4];

  for( int i = 0; i < 4; i++ )
    {
      camera_corners_[i] = Rz_*camera_corners_ref_[i] + offset;
      ::real_to_image_coords( camera_corners_[i], camera_corners_img_[i] );
      // ROS_INFO("Corner: %4d, X: %10.4f, Y: %10.4f", i, camera_corners_[i].x,
      //      camera_corners_[i].y);

    }

}
// End of getCameraCorners



/////////////////////////////////////
// Get the reference camera corners
/////////////////////////////////////
void GazelleRobot::getRefCameraCorners()
{
  // Set up rotation matrix for the camera mounting
  cv::Vec3f camera_position(camera_x_ref_, camera_y_ref_, Zc_);
  cv::Matx33f Ry( cos(camera_phi_), 0, sin(camera_phi_),
		         0,         1,         0,
                 -sin(camera_phi_), 0, cos(camera_phi_));

  // Get the viewing corners
  double y_signs[] = {1, -1, -1,  1};
  double z_signs[] = {1,  1, -1, -1};

  // Get vector norm
  double x_length = sqrt(camera_f_ * camera_f_
			 + camera_img_width_/2.0 * camera_img_width_/2.0
			 + camera_img_height_/2.0 * camera_img_height_/2.0);

  // Find corners
  for(int i = 0; i < 4; i++)
    {
      cv::Vec3f xhat;
      xhat[0] = 1.0*camera_f_ / x_length;
      xhat[1] = y_signs[i] * camera_img_width_/2.0 / x_length;
      xhat[2] = z_signs[i] * camera_img_height_/2.0 / x_length;
      cv::Vec3f camera_unit_vec = Ry*xhat;

      // Check to verify that camera is vewing downward
      if( camera_unit_vec[2] > 0.0 )
	{
	  camera_unit_vec[2] = -0.001;
	  ROS_WARN("Camera pointed towards horizon. Camera view adjusted.");
	}

      double d_length = -camera_position[2] / camera_unit_vec[2];
      cv::Vec3f proj_pt = camera_position + camera_unit_vec * d_length;
      camera_corners_ref_[i].x = proj_pt[0];
      camera_corners_ref_[i].y = proj_pt[1];

    }
}
// End of getCameraCorners




/////////////////////////////////////
// Get the reference camera corners
/////////////////////////////////////
void GazelleRobot::getRefCameraCornersORIG()
{
  // Set up rotation matrix for the camera mounting
  cv::Vec3f camera_position(camera_x_ref_, camera_y_ref_, Zc_);
  cv::Matx33f Ry( cos(camera_phi_), 0, sin(camera_phi_),
		         0,         1,         0,
                 -sin(camera_phi_), 0, cos(camera_phi_));

  // Get the viewing corners
  double y_signs[] = {1, -1, -1,  1};
  double z_signs[] = {1,  1, -1, -1};

  // Get vector norm
  double x_length = sqrt(camera_f_*camera_f_ + 32.0*32.0 + 24.0*24.0);

  // Find corners
  for(int i = 0; i < 4; i++)
    {
      cv::Vec3f xhat;
      xhat[0] = 1.0*camera_f_ / x_length;
      xhat[1] = y_signs[i] * 32.0 / x_length;
      xhat[2] = z_signs[i] * 24.0 / x_length;
      cv::Vec3f camera_unit_vec = Ry*xhat;

      // Check to verify that camera is vewing downward
      if( camera_unit_vec[2] > 0.0 )
	{
	  camera_unit_vec[2] = -0.001;
	  ROS_WARN("Camera pointed towards horizon. Camera view adjusted.");
	}

      double d_length = -camera_position[2] / camera_unit_vec[2];
      cv::Vec3f proj_pt = camera_position + camera_unit_vec * d_length;
      camera_corners_ref_[i].x = proj_pt[0];
      camera_corners_ref_[i].y = proj_pt[1];

    }
}
// End of getCameraCornersORIG



/*******************************************************************************
*
* Lidar functions
*
*******************************************************************************/

//////////////////////////////////
// Zero out the lidar scan data
//////////////////////////////////
void GazelleRobot::initializeLidarScan()
{
  // Initialize the scan values
  for(int i = 0; i < 360; i++ )
    {
      scan_data_[i] = 0.0;
      scan_loc_img_[i].x = 0;
      scan_loc_img_[i].y = 0;
    }
}
// End of initializeLidarScan
  


//////////////////////////////////////////////////////////////
// Fill the lidar scan data with distances to exterior walls
//////////////////////////////////////////////////////////////
void GazelleRobot::fillLidarScanWalls( int rows, int cols,
				       double pixels_per_meter,
				       int x_img_cg, int y_img_cg )
{
  // Get field dimensions
  //int rows = img_field_master_.rows;
  //int cols = img_field_master_.cols;

  float xmax = 1.0*(cols - x_img_cg) / pixels_per_meter;
  float xmin = -1.0*x_img_cg / pixels_per_meter;
  float ymax = 1.0*y_img_cg / pixels_per_meter;
  float ymin = -1.0*(rows - y_img_cg) / pixels_per_meter;

  double th_off = Theta_*180.0/CV_PI - floor(Theta_*180.0/CV_PI);

  //ROS_INFO("Theta: %.4f  Offset: %.4f", Theta_*180.0/CV_PI, th_off);
  
  /////////////////////////////////////////////////////////////////////
  // LIDAR SCAN DATA IS FILLED IN GLOBAL COORDINATES FROM (Xl_, Yl_)
  // DATA IS OFFSET BY Theta_ DURING PUBLISHING
  /////////////////////////////////////////////////////////////////////

  int theta_endpts[9];
  theta_endpts[0] = 0;
  theta_endpts[1] = (int) floor(atan2(ymax-Yl_,xmax-Xl_)*180.0/CV_PI);
  theta_endpts[1] = (int) ceil(atan2(ymax-Yl_,xmax-Xl_)*180.0/CV_PI);
  theta_endpts[2] = 90;
  theta_endpts[3] = (int) floor(atan2(Xl_-xmin,ymax-Yl_)*180.0/CV_PI)+90;
  theta_endpts[3] = (int) ceil(atan2(Xl_-xmin,ymax-Yl_)*180.0/CV_PI)+90;
  theta_endpts[4] = 180;
  theta_endpts[5] = (int) floor(atan2(Yl_-ymin,Xl_-xmin)*180.0/CV_PI)+180;
  theta_endpts[5] = (int) ceil(atan2(Yl_-ymin,Xl_-xmin)*180.0/CV_PI)+180;
  theta_endpts[6] = 270;
  theta_endpts[7] = (int) floor(atan2(xmax-Xl_,Yl_-ymin)*180.0/CV_PI)+270;
  theta_endpts[7] = (int) ceil(atan2(xmax-Xl_,Yl_-ymin)*180.0/CV_PI)+270;
  theta_endpts[8] = 360;

  // Fill lidar data - section 0
  for( int theta = theta_endpts[0]; theta < theta_endpts[1]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (xmax - Xl_ ) / cos(CV_PI/180.0*th);
      cv::Point2f scan_loc( xmax,
			    Yl_+scan_data_[theta]*sin(CV_PI/180.0*th));
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 1
  for( int theta = theta_endpts[1]; theta < theta_endpts[2]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (ymax - Yl_ ) / cos(CV_PI/180.0*(90.0-th));
      cv::Point2f scan_loc(Xl_+scan_data_[theta]*sin(CV_PI/180.0*(90.0-th)),
			   ymax);
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 2
  for( int theta = theta_endpts[2]; theta < theta_endpts[3]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (ymax - Yl_ ) / cos(CV_PI/180.0*(th-90.0));
      cv::Point2f scan_loc( Xl_-scan_data_[theta]*sin(CV_PI/180.0*(th-90.0)),
						      ymax);
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 3
  for( int theta = theta_endpts[3]; theta < theta_endpts[4]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Xl_ - xmin ) / cos(CV_PI/180.0*(180.0-th));
      cv::Point2f scan_loc( xmin, Yl_ +
			    scan_data_[theta]*sin(CV_PI/180.0*(180.0-th)));
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }
  
  // Fill lidar data - section 4
  for( int theta = theta_endpts[4]; theta < theta_endpts[5]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Xl_ - xmin ) / cos(CV_PI/180.0*(th-180.0));
      cv::Point2f scan_loc( xmin, Yl_ -
			    scan_data_[theta]*sin(CV_PI/180.0*(th-180.0)));
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 5
  for( int theta = theta_endpts[5]; theta < theta_endpts[6]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Yl_ - ymin ) / cos(CV_PI/180.0*(270.0-th));
      cv::Point2f scan_loc( Xl_-scan_data_[theta]*sin(CV_PI/180.0*(270.0-th)),
			    ymin );
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }


  // Fill lidar data - section 6
  for( int theta = theta_endpts[6]; theta < theta_endpts[7]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Yl_ - ymin ) / cos(CV_PI/180.0*(th-270.0));
      cv::Point2f scan_loc( Xl_+scan_data_[theta]*sin(CV_PI/180.0*(th-270.0)),
			    ymin );
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 7
  for( int theta = theta_endpts[7]; theta < theta_endpts[8]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (xmax - Xl_ ) / cos(CV_PI/180.0*(-th));
      cv::Point2f scan_loc( xmax,
			    Yl_-scan_data_[theta]*sin(CV_PI/180.0*(-th)));
      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }
}
// End GazelleRobot::fillLidarScanWalls



//////////////////////////////////////////
// Fill lidar scan for polygon obstacles
//////////////////////////////////////////
void GazelleRobot::fillLidarScanPolygonObstruction(const std::vector<cv::Point2f> pts)
{
  // Loop through sides and fill lidar scan
  for(int i = 0; i < 4; i++ )
    fillLidarScanLine(pts[i], pts[(i+1)%4]);
}
// End of fillLidarScanPolygonObstruction



///////////////////////////////////////////
// Fill the scan data for a line segement
///////////////////////////////////////////
void GazelleRobot::fillLidarScanLine(const cv::Point2f pta, const cv::Point2f ptb )
{
  // Determine proper 1st and 2nd segment point (right hand rule)
  cv::Point3f pt1(pta.x, pta.y, 0.0);
  cv::Point3f pt2(ptb.x, ptb.y, 0.0);
  cv::Point3f robot(Xl_, Yl_, 0.0);
  cv::Point3f direction = (pt1-robot).cross(pt2-robot);

  // Switch points if needed
  if( direction.z < 0 )
    {
      cv::Point3f tmp;
      tmp = pt1;
      pt1 = pt2;
      pt2 = tmp;
    }
  
  // Find angle to each endpoint
  double theta1 = atan2(pt1.y - Yl_, pt1.x - Xl_)*180.0/CV_PI;
  if(theta1 < 0) theta1 += 360;
  int th1i = (int) ceil(theta1);

  double theta2 = atan2(pt2.y - Yl_, pt2.x - Xl_)*180.0/CV_PI;
  if(theta2 < 0) theta2 += 360;
  int th2i = (int) floor(theta2);

  //ROS_INFO("Theta 1,2 = (%.2f,%.2f) => %d, %d", theta1, theta2, th1i, th2i);


  // Line segment parameters
  double m_l, b_l;
  if( fabs(pt2.x-pt1.x) > 0.0 )
    {
      m_l = (pt2.y-pt1.y)/(pt2.x-pt1.x);
      b_l = pt1.y - m_l*pt1.x;
    }
  else
    {
      m_l = 0.0;  // Undefined - not used in this case
      b_l = 0.0;  // Undefined - not used in this case
    }

  // Get the theta start and theta end
  int npts = th2i - th1i + 1;
  if( npts < 0 )
    {
      npts = 360 - th1i + th2i + 1;
    }

  // Loop through angles and get distances
  for( int ipt = 0; ipt < npts; ipt++ )
    {
      int theta = th1i + ipt;
      if( theta > 359 ) theta -= 360;
      
      // Get theta line
      double m_th = tan(theta*CV_PI/180.0);
      double b_th = robot.y - m_th*robot.x;
      
      // Find intersection
      double x_inter, y_inter;
      if( fabs(pt2.x-pt1.x) > 0.0 )
	{
	  x_inter = (b_th - b_l) / (m_l - m_th);
	  y_inter = m_l * x_inter + b_l;
	}
      else
	{
	  x_inter = pt1.x;
	  y_inter = m_th * x_inter + b_th;
	}
      
      // Update scan
      double dist = sqrt( pow(robot.x-x_inter,2) +
			  pow(robot.y-y_inter,2) );
      if( scan_data_[theta] == 0.0 || scan_data_[theta] > dist )
	{
	  cv::Point2f scan_loc( x_inter, y_inter);
	  ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
	  scan_data_[theta] = dist;
	  //ROS_INFO("TH: %d, Dist: %.2f", theta, dist);
	}
    }
}
// End of fillLidarScanLine



//////////////////////////////////////////////
// Fill lidar scan for circular obstructions
//////////////////////////////////////////////
void GazelleRobot::fillLidarScanCircularObstruction(const cv::Point2f center,
						    const double radius )
{

  // Find vectors to visible range of circular object: v1 & v2
  cv::Point3f robot_to_center( center.x - Xl_, center.y - Yl_, 0.0);
  cv::Point3f z_unit_vec( 0.0, 0.0, 1.0);
  cv::Point3f vec1, vec2;
  
  vec1 = robot_to_center.cross(z_unit_vec);
  vec2 = z_unit_vec.cross(robot_to_center);
  vec1 = vec1/norm(vec1);
  vec2 = vec2/norm(vec2);
  cv::Point2f v1(vec1.x, vec1.y);
  cv::Point2f v2(vec2.x, vec2.y);

  // Find end points coordinates
  cv::Point2f pt1 = center + radius*v1;
  cv::Point2f pt2 = center + radius*v2;


  // Determine proper 1st and 2nd segment point (right hand rule)
  cv::Point3f pt1_tmp(pt1.x, pt1.y, 0.0);
  cv::Point3f pt2_tmp(pt2.x, pt2.y, 0.0);
  cv::Point3f robot3(Xl_, Yl_, 0.0);
  cv::Point3f direction = (pt1_tmp-robot3).cross(pt2_tmp-robot3);

  // Switch points if needed
  if( direction.z < 0 )
    {
      cv::Point2f tmp;
      tmp = pt1;
      pt1 = pt2;
      pt2 = tmp;
    }


  // Get the integer angle end values
  double theta1 = atan2(pt1.y - Yl_, pt1.x - Xl_)*180.0/CV_PI;
  if(theta1 < 0) theta1 += 360;
  int th1i = (int) ceil(theta1);

  double theta2 = atan2(pt2.y - Yl_, pt2.x - Xl_)*180.0/CV_PI;
  if(theta2 < 0) theta2 += 360;
  int th2i = (int) floor(theta2);

  
  // Loop inside the two end values
  int npts = th2i - th1i + 1;
  if( npts < 0 )
    {
      npts = 360 - th1i + th2i + 1;
    }

  cv::Point2f p1(Xl_-center.x, Yl_-center.y);
  for( int ipt = 1; ipt < npts; ipt++ )
    {
      int theta = th1i + ipt;
      if( theta > 359 ) theta -= 360;

      // Get 2nd point on line
      cv::Point2f p2 = p1 + 10.0*cv::Point2f(cos(theta*CV_PI/180.0),
					     sin(theta*CV_PI/180.0));

      cv::Point2i p1i, p2i;
      // Find line and circle intersection
      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      int sign_dy = dy > 0 ? 1: -1;
      double dr = sqrt( dx*dx + dy*dy );
      double D = p1.x*p2.y - p2.x*p1.y;
      double r = radius;
      double delta = r*r*dr*dr - D*D;
      if( delta > 0.0 )
	{
	  double x1 = (D*dy + sign_dy*dx*sqrt(delta))/(dr*dr) + center.x;
	  double x2 = (D*dy - sign_dy*dx*sqrt(delta))/(dr*dr) + center.x;
	  double y1 = (-D*dx + abs(dy)*sqrt(delta))/(dr*dr) + center.y;
	  double y2 = (-D*dx - abs(dy)*sqrt(delta))/(dr*dr) + center.y;
	  double dist1 = sqrt( (Xl_-x1)*(Xl_-x1) + (Yl_-y1)*(Yl_-y1) );
	  double dist2 = sqrt( (Xl_-x2)*(Xl_-x2) + (Yl_-y2)*(Yl_-y2) );
	  double x_inter = x1;
	  double y_inter = y1;
	  double dist = dist1;
	  if( dist2 < dist1 )
	    {
	      x_inter = x2;
	      y_inter = y2;
	      dist = dist2;
	    }
	      
	  if( scan_data_[theta] == 0.0 || scan_data_[theta] > dist )
	    {
	      cv::Point2f scan_loc( x_inter, y_inter);
	      ::real_to_image_coords(scan_loc, scan_loc_img_[theta]);
	      scan_data_[theta] = dist;
	    }

	}
      else
	{
	  ROS_ERROR("Error in lidar to circular object.");
	}
    }
  
}
// End of fillLidarScanCircularObstruction




/*******************************************************************************
*
* GPS functions
*
*******************************************************************************/

////////////////////////////////////////////////////////////////////
// Compute GPS based on the current robot location
// Source: https://www.dcode.fr/geographic-coordinates-calculation
// Cross Ref: https://www.movable-type.co.uk/scripts/latlong.html
//            Used in gazelle_sim.cpp
//
// φ2 = Math.asin( Math.sin(φ1)*Math.cos(d/R) +
//                 Math.cos(φ1)*Math.sin(d/R)*Math.cos(brng) );
// λ2 = λ1 + Math.atan2(Math.sin(brng)*Math.sin(d/R)*Math.cos(φ1),
//                      Math.cos(d/R)-Math.sin(φ1)*Math.sin(φ2));
//
// where φ is latitude, λ is longitude, θ is the bearing (clockwise from north)
// δ is the angular distance d/R; d being the distance travelled,
// R the earth’s radius
////////////////////////////////////////////////////////////////////
void GazelleRobot::computeGPSLocation()
{
  double x1 = north_unit_vec_.x;
  double y1 = north_unit_vec_.y;
  double x2 = Xr_;
  double y2 = Yr_;
  
  double dot = x1*x2 + y1*y2;
  double det = x1*y2 - y1*x2;
  double direction = -atan2(det,dot)*180.0/CV_PI;
  double distance = sqrt(pow(x2,2.0) + pow(y2,2.0));

  latitude_ = asin( sin(latitude_base_*CV_PI/180.0) *
		    cos(distance/earth_radius_) +
		    cos(latitude_base_*CV_PI/180.0) *
		    sin(distance/earth_radius_) *
		    cos(direction*CV_PI/180.0) ) * 180.0/CV_PI;
                                 
  longitude_ = longitude_base_ +
    atan2( sin(direction*CV_PI/180.0) * sin(distance/earth_radius_) *
	   cos(latitude_base_*CV_PI/180.0),
	   cos(distance/earth_radius_) - sin(latitude_base_*CV_PI/180.0) *
	   sin(latitude_*CV_PI/180.0) ) * 180.0/CV_PI;
  
}
// End of computeGPSLocation



////////////////////////////////////////////////////////
// Compute GPS based velociy based on robot velocities
///////////////////////////////////////////////////////
void GazelleRobot::computeGPSVelocity(double vx, double vy)
{
  // Rotate velocity vector and convert to mm/s
  gps_ned_vel_ = Rz_gps_vel_ * cv::Point2f(vx, vy) * 1000.0;
}
// End of computeGPSVelocity



/*******************************************************************************
*
* Solver functions
*
*******************************************************************************/

/////////////////////////////////////////////
// Set the robot state to the initial state
/////////////////////////////////////////////
void GazelleRobot::resetState()
{
  // Reset Position
  Xr_ = Xr_init_;
  Yr_ = Yr_init_;
  Theta_ = Theta_init_;
  Xr_prev_ = Xr_init_;
  Yr_prev_ = Yr_init_;
  Theta_prev_ = Theta_init_;

  // Reset Velocities
  linear_x_ = 0.0;
  angular_z_ = 0.0;
}
// End of resetState



/////////////////////////////////////////////////
// Save the current state as the previous state
/////////////////////////////////////////////////
void GazelleRobot::saveCurrentState()
{
  Xr_prev_ = Xr_;
  Yr_prev_ = Yr_;
  Theta_prev_ = Theta_;
}
// End of saveCurrentState



//////////////////////////////////
// Revet from the previous state
//////////////////////////////////
void GazelleRobot::revertState()
{
  Xr_ = Xr_prev_;
  Yr_ = Yr_prev_;
  Theta_ = Theta_prev_;
  vx_ = 0.0;
  vy_ = 0.0;
}
// End of revertState



/////////////////////////////
// Complete simulation step
/////////////////////////////
void GazelleRobot::computeNextState(double delta_t)
{
  // Compute local velocity and update location
  vx_ = linear_x_ * cos(Theta_);
  vy_ = linear_x_ * sin(Theta_);
  Xr_ = Xr_ + vx_ * delta_t;
  Yr_ = Yr_ + vy_ * delta_t;

  if( robot_type_ == DIFF_DRIVE_RECT || robot_type_ == DIFF_DRIVE_CIRC )
    {
      Theta_ = Theta_ + angular_z_ * delta_t;
      omega_ = angular_z_;
    }
  else
    {
      // Compute steer angle from yaw rate request
      //
      // delta = wheelbase / turn_radius
      // yaw_rate = velocity / turn_radius
      //   ==>> turn_radius = velocity / yaw_rate
      //
      // delta = wheelbase / (velocity / yaw_rate)
      // 
      if( fabs(angular_z_) > 0.0 && abs(linear_x_ > 0.01) ) {
	steer_angle_ = wheelbase_ / (linear_x_ / angular_z_);
      }
      else {
	steer_angle_ = 0.0;
      }

      // Cap steering to max steer
      if( steer_angle_ > max_steer_) {
	ROS_WARN("Steering lock reached");
	steer_angle_ = max_steer_;
      }
      if( steer_angle_ < -max_steer_) {
	ROS_WARN("Steering lock reached");
	steer_angle_ = -max_steer_;
      }

      omega_ = linear_x_ / wheelbase_ * tan(steer_angle_);
      Theta_ = Theta_ +  omega_* delta_t;
    }
}
// End of computeNextState


////////////////////////////////////////////////////////////////////////////////
// Update robot for the new time and publish lidar, trasnform and GPS messages
//
// Camera images are published from the simulator
////////////////////////////////////////////////////////////////////////////////
void GazelleRobot::updateNextState( ros::Time time_now )
{
  setRotationZ();
  setLidarGlobalLocation();
  publishTransform( time_now );
  computeGPSLocation();
  computeGPSVelocity(vx_, vy_);
  publishGPS();
}
// End of updateNextState



//////////////////////////
// Output robot location
//////////////////////////
void GazelleRobot::displayLocation()
{

  ROS_INFO("%s: X: %.2f m; Y: %.2f m, Theta: %.2f deg",
	   robot_name_.c_str(), Xr_, Yr_,
	   Theta_*180.0/CV_PI );
}
// End of displayLocation



