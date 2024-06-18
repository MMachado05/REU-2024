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


#include "simple_sim/simple_sim.h"


SimpleSim::SimpleSim()
  : nh_priv_("~"), it_{nh_}
{
  // Initialize the simple sim node
  ROS_INFO("Simple Sim Node Initialized");
  auto ret = init();
  ROS_ASSERT(ret);

}

SimpleSim::~SimpleSim()
{
  // Stop robot
  linear_x_ = 0.0;
  angular_z_ = 0.0;
  ros::shutdown();
}


/*******************************************************************************
*
* Init function
*
*******************************************************************************/
bool SimpleSim::init()
{
  // Define the robot and camera default locations (in plane)
  setDefaultParameters();
  
  // Read parameter file
  if( !loadParameters() ) exit(0);

  // Check parameter
  if( !checkParameters() ) exit(0);
  
  // Define the time step
  delta_t_target_ = 0.050; // 50 ms
  
  // Define twist inputs and angular velocity
  linear_x_ = 0.0;
  angular_z_ = 0.0;
  omega_ = 0.0;

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
  img_cam_ = cv::Mat::zeros(cv::Size(480,640),CV_8UC1);
  for( int i = 0; i < 4; i++ )
    {
      camera_corners_[i] = cv::Point2f(0.0,0.0);
      camera_corners_img_[i] = cv::Point2i(0,0);
    }


  // Initialize publishers
  camera_view_pub_ = it_.advertise("/camera/image_raw", 50);

  odom_pub_= nh_.advertise<nav_msgs::Odometry>("/odom", 50);
  
  lidar_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 50);

  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps", 50);
  
  // Initialize subscribers
  cmd_vel_sub_  = nh_.subscribe("/simulator/cmd_vel", 50,
				&SimpleSim::cmdVelocityCb, this);
  
  // Dynamic Reconfigure
  server_.setCallback(boost::bind(&SimpleSim::configCallback, this, _1, _2));

  // Load defaults
  server_.getConfigDefault(config_);

  // Initialize robot and sensors
  setRotationZ();
  setRobotRefPoints();
  setLidarGlobalLocation();
  getRefCameraCorners();
  
  return true;
}
// End of SimpleSim::init



/*******************************************************************************
*
* Parameter functions
*
*******************************************************************************/

///////////////////////////
// Set default parameters
//////////////////////////
void SimpleSim::setDefaultParameters()
{
  // Set image scaling
  img_view_scale_ = 1.0;
  
  // Robot location variables
  init_flag_ = false;
  Xr_init_ = 0.0;
  Yr_init_ = 0.0;
  Theta_init_= 0.0;
  Xr_ = 0.0;
  Yr_ = 0.0;
  Theta_ = 0.0;
  cv::Matx22f Rz_(1, 0, 0, 1);

  // GPS variables
  latitude_base_ = 42.474775859702966;    // Use LTU Ockham's Wedge
  longitude_base_ = -83.24920586143432;
  latitude_ = latitude_base_;
  longitude_ = longitude_base_;
  north_angle_ = 90.0;
  north_unit_vec_ = cv::Point2f(0.0,1.0); // North aligned with y-axis (90 deg)

  double ang_rad = (north_angle_-90.0)*CV_PI/180.0;
  Rz_gps_vel_ = cv::Matx22f( cos(ang_rad), -sin(ang_rad),
			     sin(ang_rad),  cos(ang_rad));;
  earth_radius_ = 6371000;                // m
  gps_ned_vel_ = cv::Point2f(0.0,0.0);
  
  // Camera parameters and location - will not draw camera
  camera_width_ = 0.0; 
  camera_length_= 0.0;
  camera_x_ref_ = 0.0;
  camera_y_ref_ = 0.0;

  // Lidar locations and parameters - will not draw lidar
  Xl_ = Xr_;
  Yl_ = Yr_;
  lidar_x_ref_ = 0.0;
  lidar_y_ref_ = 0.0;
  lidar_radius_ref_ = 0.0;

  // Robot parameters
  wheelbase_ = 1.0;
  max_steer_ = 0.25*CV_PI;

  // Colors - BGR - NEED NON-ZERO in EACH BGR FOR COLLISION MODEL
  robot_color_ = cv::Scalar(  1, 125,   1);
  tire_color_  = cv::Scalar( 30,  30,  30);
  camera_color_= cv::Scalar(  1,   1, 255);
  lidar_color_ = cv::Scalar(255,   1,   1);
  wall_color_  = cv::Scalar(150, 150, 150);
  obs_color_   = cv::Scalar(15, 65, 140);

}
// End of set DefaultParameters




////////////////////////////
// Load the ROS parameters
////////////////////////////
bool SimpleSim::loadParameters()
{
  // Get simple_sim package dir
  std::string package_dir;
  ros::param::get("simple_sim_package_dir", package_dir);
  // Get image loading dir
  std::string map_dir;
  ros::param::get("map_dir", map_dir);

  // Check for required parameters
  if(!( nh_.hasParam("robot_type") && nh_.hasParam("field_image_file")
	&& nh_.hasParam("x_scale") && nh_.hasParam("y_scale")
	&& nh_.hasParam("x_center_img") && nh_.hasParam("y_center_img")) )
    {
      ROS_ERROR("The following parameters must be provided:");
      ROS_ERROR("robot_type");
      ROS_ERROR("field_image_file");
      ROS_ERROR("x_scale & y_scale");
      ROS_ERROR("x_center_img & y_center_img");
      return false;
    }

  // Get field file name
  ros::param::get("field_image_file", fname_field_img_);
  if (fname_field_img_.length() == 0)
    {
      ROS_ERROR_STREAM("Missing parameter 'field_image_file'");
      return false;
    }
  else if (fname_field_img_[0] != '/')
    {
      fname_field_img_ = map_dir + "/" + fname_field_img_;
    }


  // Set image scaling
  ros::param::get("x_scale", x_scale_field_);
  ros::param::get("y_scale", y_scale_field_);

  // Place field on coordinate system
  ros::param::get("x_center_img", x_img_cg_);
  ros::param::get("y_center_img", y_img_cg_);

  // Get the view image scale
  if(nh_.hasParam("img_view_scale"))
    ros::param::get("img_view_scale", img_view_scale_);

  // Get the robot type
  std::string robot_str;
  if(nh_.hasParam("robot_type"))
    {
      ros::param::get("robot_type", robot_str);
      if( robot_str.compare("diff_drive_rect") == 0 )
	robot_type_ = DIFF_DRIVE_RECT;
      
      else if( robot_str.compare("diff_drive_circ") == 0 )
	robot_type_ = DIFF_DRIVE_CIRC;
      
      else if( robot_str.compare("acker_steer_rect") == 0 )
	robot_type_ = ACKER_STEER_RECT;
      
      else
	{
	  ROS_ERROR_STREAM("Invalid parameter 'robot_type'");
	  return false;
	}
    }
  else
    {
      ROS_ERROR_STREAM("Missing parameter 'robot_type'");
      return false;
    }       
  
  // Load robot initial location
  if(nh_.hasParam("Xr_init")) ros::param::get("Xr_init", Xr_init_);
  if(nh_.hasParam("Yr_init")) ros::param::get("Yr_init", Yr_init_);
  if(nh_.hasParam("Theta_init"))
    {
      ros::param::get("Theta_init", Theta_init_);
      Theta_init_ = Theta_init_ * CV_PI/180.0;
    }
  Xr_ = Xr_init_;
  Yr_ = Yr_init_;
  Theta_ = Theta_init_;

  // Robot size parameters
  if(nh_.hasParam("robot_width")) ros::param::get("robot_width", robot_width_);
  if(nh_.hasParam("robot_length"))ros::param::get("robot_length",robot_length_);
  if(nh_.hasParam("robot_radius"))ros::param::get("robot_radius",robot_radius_);
  if(nh_.hasParam("rwheel")) ros::param::get("rwheel", rwheel_);
  if(nh_.hasParam("wheel_width")) ros::param::get("wheel_width", wheel_width_);
  if(nh_.hasParam("trackwidth")) ros::param::get("trackwidth", trackwidth_);
  if(nh_.hasParam("axle_dst"))
    ros::param::get("axle_dst", axle_dst_);
  if(nh_.hasParam("wheelbase")) ros::param::get("wheelbase", wheelbase_);
  if(nh_.hasParam("max_steer")) ros::param::get("max_steer", max_steer_);

  // Camera location and dimensions
  if(nh_.hasParam("camera_x")) ros::param::get("camera_x", camera_x_ref_);
  if(nh_.hasParam("camera_y")) ros::param::get("camera_y", camera_y_ref_);
  if(nh_.hasParam("camera_width"))
    ros::param::get("camera_width", camera_width_);
  if(nh_.hasParam("camera_length"))
    ros::param::get("camera_length", camera_length_);

  // Lidar location and dimensions
  if(nh_.hasParam("lidar_x")) ros::param::get("lidar_x", lidar_x_ref_);
  if(nh_.hasParam("lidar_y")) ros::param::get("lidar_y", lidar_y_ref_);
  if(nh_.hasParam("lidar_r")) ros::param::get("lidar_r", lidar_radius_ref_);

  // GPS variables
  if(nh_.hasParam("latitude_base")) ros::param::get("latitude_base",
						    latitude_base_);
  if(nh_.hasParam("longitude_base")) ros::param::get("longitude_base",
						     longitude_base_);
  if(nh_.hasParam("north_angle"))
    {
      ros::param::get("north_angle", north_angle_);
      cv::Point2f xhat{1.0, 0.0};
      double ang_rad = north_angle_*CV_PI/180.0;
      north_unit_vec_ = cv::Matx22f( cos(ang_rad), -sin(ang_rad),
				     sin(ang_rad),  cos(ang_rad)) * xhat;
      
      ang_rad = (north_angle_-90.0)*CV_PI/180.0;
      Rz_gps_vel_ = cv::Matx22f( cos(ang_rad), -sin(ang_rad),
				 sin(ang_rad),  cos(ang_rad));
      
    }

  std::cout << "North Vector = " << north_unit_vec_ << std::endl;

  /*      
  if(nh_.hasParam("north_angle"))
    {
      
      ros::param::get("north_vec", tmp);
      double dist = sqrt(tmp[0]*tmp[0] + tmp[1]*tmp[1]);
      if( abs(dist) < 0.001 )
	{
	  ROS_ERROR("GPS North Vector Norm is Zero");
	  exit(0);
	}
      tmp[0] = tmp[0] / dist;
      tmp[1] = tmp[1] / dist;
      north_unit_vec_ = cv::Point2f(tmp[0],tmp[1]);
    } 
  */
  
  if(nh_.hasParam("earth_radius")) ros::param::get("earth_radius",
						   earth_radius_);
  // Load the obstructions
  loadObstructions(1);

  // Succesful completion
  return true;
}
// End of SimpleSim::loadParameters


////////////////////////////////////
// Load the obstrcution parameters
////////////////////////////////////
bool SimpleSim::loadObstructions(int echo_on)
{
  // Clear the current data
  rect_obs_corners_.clear();
  circle_obs_center_.clear();
  circle_obs_radius_.clear();
    
  // Load obstruction objects
  char str[10];
  for( int i = 0; i < 10; i++ )
    {
      sprintf(str,"rect%d",i);
      if(nh_.hasParam(str))
	{
	  if( echo_on ) ROS_INFO("Loading rect%d", i );
	  std::vector<double> tmp;
	  ros::param::get(str, tmp);
	  std::vector<cv::Point2f> tmpvec;
	  for(int j = 0; j < 4; j++ )
	    tmpvec.push_back(cv::Point2f(tmp[2*j], tmp[2*j+1]));
	  rect_obs_corners_.push_back(tmpvec);
	}
      
      sprintf(str,"circ%d",i);
      if(nh_.hasParam(str))
	{
	  if( echo_on ) ROS_INFO("Loading circ%d", i );
	  std::vector<double> tmp;
	  ros::param::get(str, tmp);
	  circle_obs_center_.push_back(cv::Point2f(tmp[0], tmp[1]));
	  circle_obs_radius_.push_back(tmp[2]);
	}
    }

  return true;
}


////////////////////////////
// Check the ROS parameters - TO DO - NOT IMPLEMENTED
////////////////////////////
bool SimpleSim::checkParameters()
{
  return true;
}
// End of checkParameters




/*******************************************************************************
*
* Callback functions
*
*******************************************************************************/

////////////////////////////////
// Callback: Dynamic Reconfigure
////////////////////////////////
void SimpleSim::configCallback(simple_sim::SimpleSimConfig &config, uint32_t level)
{
    config_ = config;

    // Set configuraion variables
    Zc_ = config_.camera_z;
    camera_phi_ = config_.camera_phi * CV_PI/180.0;
    camera_f_ = config_.camera_f;
    camera_border_ = config_.camera_border;
    lidar_active_ = config_.lidar_active;
    lidar_max_dist_ = config_.lidar_max_dst;
    lidar_marker_size_ = config_.lidar_mkr_size;
    exterior_walls_ = config_.exterior_walls;
    obstructions_ = config_.obstructions;
    collision_model_ = config_.collision_type;
    init_flag_ = config_.initialize;

    // Update camera reference corners for new Zc_
    getRefCameraCorners();

}
// End of SimpleSim::configCallback


///////////////////////////////
// Callback: Command Velocity
//////////////////////////////
void SimpleSim::cmdVelocityCb(const geometry_msgs::Twist::ConstPtr &msg)
{
  linear_x_ = msg->linear.x;
  angular_z_ = msg->angular.z;
}
// End of SimpleSim::cmdVelocityCb



/*******************************************************************************
*
* Publish functions
*
*******************************************************************************/

///////////////////////////
// Publish the lidar scan
///////////////////////////
void SimpleSim::publishLidarScan()
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
      if( scan_data_[idx] > 0.0 && scan_data_[idx] < lidar_max_dist_ )
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
void SimpleSim::publishGPS()
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
void SimpleSim::publishTransform( const ros::Time time_now )
{

  // Set robot location
  transformStamped_.header.stamp = time_now;
  transformStamped_.header.frame_id = "map";
  transformStamped_.child_frame_id = "base_link";
  transformStamped_.transform.translation.x = Xr_;
  transformStamped_.transform.translation.y = Yr_;
  transformStamped_.transform.translation.z = 0.0;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(Theta_);
  transformStamped_.transform.rotation.x = q.x;
  transformStamped_.transform.rotation.y = q.y;
  transformStamped_.transform.rotation.z = q.z;
  transformStamped_.transform.rotation.w = q.w;

  // Publish the base_link transform
  tf_broadcaster_.sendTransform(transformStamped_); 


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
// End of SimpleSim::publishTransform



/*******************************************************************************
*
* Coordinate mapping functions
*
*******************************************************************************/

///////////////////////////////////////////////////////
// Mapping from real coordinates to image coordinates
///////////////////////////////////////////////////////
void SimpleSim::real_to_image_coords( const cv::Point2f &point,
				      cv::Point2i &img_point )
{
  img_point.x = x_img_cg_ + (int) (point.x * x_scale_field_);
  img_point.y = y_img_cg_ - (int) (point.y * y_scale_field_);
}
// End of SimpleSim::real_to_image_coords



/*******************************************************************************
*
* Geometric support functions
*
*******************************************************************************/

double SimpleSim::angle2D( const cv::Point2f p1, const cv::Point2f p2)
{
  {
   double dtheta,theta1,theta2;

   theta1 = atan2(p1.y,p1.x);
   theta2 = atan2(p2.y,p2.x);
   dtheta = theta2 - theta1;
   while (dtheta > CV_PI)
      dtheta -= 2.0*CV_PI;
   while (dtheta < -CV_PI)
      dtheta += 2.0*CV_PI;

   return(dtheta);
}

}
// End of SimpleSim::angle2D


///////////////////////////////////////
// Determine if a point is a trapeziod
///////////////////////////////////////
bool SimpleSim::pointInsidePolygon(const cv::Point2f *polygon,
				   const int n, const cv::Point2f p)
{
   int i;
   double angle=0;
   cv::Point2f p1,p2;

   for (i=0;i<n;i++)
     {
       p1 = polygon[i] - p;
       p2 = polygon[(i+1)%n] - p;
       angle += angle2D(p1, p2);
     }

   if (fabs(angle) < CV_PI)
      return(false);
   else
      return(true);
}
// End of SimpleSim::piontInsidePolygon



/////////////////////////////////////////////////////////////////////////////
// Determine if the robot is in contact with exterior walls or obstructions
////////////////////////////////////////////////////////////////////////////
bool SimpleSim::robotCollision( )
{
  // Create blank images
  cv::Mat img_robot(img_field_master_.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat img_obs = img_robot.clone();

  // Draw the robot 
  drawRobot( img_robot );

  // Draw exterior walls
  if( exterior_walls_ )
    cv::rectangle(img_obs, cv::Point2i(0,0),
		  cv::Point( img_obs.cols-1, img_obs.rows-1 ),
		  cv::Scalar(255), 1 );
  
  // Draw the obstructions
  if( obstructions_ )
    {
      // Draw polygon obstructions
      for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
	{
	  drawPolygonObstruction(img_obs, rect_obs_corners_[i]);
	}

      
      // Draw circular obstructions
      for(int i = 0; i < (int) circle_obs_center_.size(); i++ )
	{
	  drawCircularObstruction(img_obs, circle_obs_center_[i],
				  circle_obs_radius_[i]);
	}
    }

  // Check for collisions
  cv::Mat img_coll;
  cv::bitwise_and( img_robot, img_obs, img_coll);
  
  if( cv::countNonZero(img_coll) )
    {
      return true;
    }

  return false;
}
// End of SimpleSim::robotCollision




/*******************************************************************************
*
* Drawing functions
*
*******************************************************************************/

///////////////////////
// Update field image
///////////////////////
void SimpleSim::updateFieldImage(const bool update_time_disp,
				 const double delta_t)
{
  static char time_str[100] = "";
 
  // Copy over master image
  cv::Mat img_field = img_field_master_.clone();

  // Draw the coordinates
  drawFieldCoord( img_field );

  // Draw the obstructions
  if( obstructions_ )
    {
      // Draw polygon obstructions
      for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
	{
	  drawPolygonObstruction(img_field, rect_obs_corners_[i]);
	}

      
      // Draw circular obstructions
      for(int i = 0; i < (int) circle_obs_center_.size(); i++ )
	{
	  drawCircularObstruction(img_field, circle_obs_center_[i],
				  circle_obs_radius_[i]);
	}
   }
  
  // Draw the robot
  drawRobot( img_field );

  // Get the camera view window
  getCameraImage( img_field );

  // Get the lidar data
  if( lidar_active_ )
    {
      // Initialize the scan data
      initializeLidarScan();
      
      // Check for exterior walls
      if( exterior_walls_ )
	fillLidarScanWalls();

      // Check for obstructions
      if( obstructions_ )
	{
	  // Process polygon obstructions
	  for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
	    {
	      fillLidarScanPolygonObstruction(rect_obs_corners_[i]);
	    }

	  // Process circular obstructions
	  for(int i = 0; i < (int) circle_obs_center_.size(); i++ )
	    {
	      fillLidarScanCircularObstruction(circle_obs_center_[i],
					       circle_obs_radius_[i] );
	    }
	}

      // Draw lidar scan results
      drawLidarScan( img_field );

      // Publish the lidar scan
      publishLidarScan();
    }
  
  // Draw camera bounding box
  if( camera_border_ )
    drawOutlineCameraView( img_field );
 

  // Create viewing size image
  cv::Mat img_view;
  cv::resize(img_field, img_view, cv::Size(), img_view_scale_,
	     img_view_scale_, cv::INTER_LINEAR);

  if( update_time_disp )
    {
      // Place execution deltas
      sprintf(time_str,"Target/Act: %.3fs/%.3fs = %.2f RT",
	      delta_t_target_, delta_t, 1.0/(delta_t/delta_t_target_) );
    }
      
  cv::putText(img_view,
	      time_str,
	      cv::Point(10, img_view.rows-10 ),
	      cv::FONT_HERSHEY_COMPLEX_SMALL,
	      1,
	      CV_RGB(0, 0, 255),
	      2);

  // Notify user if simulation is frozen
  if( init_flag_ )
    {
      int baseline = 0;
      cv::Size sz;
      sz = cv::getTextSize("Simulation Stopped",
			   cv::FONT_HERSHEY_COMPLEX_SMALL,
			   1,
			   2,
			   &baseline);
      
      cv::putText(img_view,
		  "Simulation Stopped",
		  cv::Point(int(img_view.cols/2 -sz.width/2),
			    int(img_view.rows/2) ),
		  cv::FONT_HERSHEY_COMPLEX_SMALL,
		  1,
		  CV_RGB(255, 0, 0),
		  2);
     
    }
  // Publish camera view
  // Create message wrapper
  sensor_msgs::ImagePtr msg_pub;
  msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
			       img_cam_).toImageMsg();
  
  // Publish the preview image
  camera_view_pub_.publish(msg_pub);


  // Test for collision
  if( collision_model_ && (robotCollision() || obs_sim_stopped_) )
    {
      cv::Mat img_red(img_view.size(), CV_8UC3, cv::Scalar(0,0,255));
      cv::addWeighted(img_view, 0.8, img_red, 0.2, 0.0, img_view);
    }

  // Display playing field
  cv::imshow("Playing Field", img_view);
  cv::waitKey((int) (1.0/(2*delta_t_target_)));

}
// End of SimpleSim::updateFieldImage


////////////////////////////
// Draw field coordindates
////////////////////////////
void SimpleSim::drawFieldCoord(cv::Mat &img_field )
{
  cv::Point2f coord_center = cv::Point2f( x_img_cg_, y_img_cg_ );
  cv::arrowedLine( img_field, cv::Point2f( x_img_cg_, y_img_cg_ ),
		   cv::Point2f( x_img_cg_+150, y_img_cg_ ), cv::Scalar(225,0,0),
		   6, 8, 0, 0.2 );
  cv::putText(img_field,
	      "X",
	      cv::Point2f( x_img_cg_+150, y_img_cg_ ),
	      cv::FONT_HERSHEY_DUPLEX,
	      1.5,
	      CV_RGB(0, 0, 255),
	      2);
  
  cv::arrowedLine( img_field, cv::Point2f( x_img_cg_, y_img_cg_ ),
		   cv::Point2f( x_img_cg_, y_img_cg_-150 ), cv::Scalar(0,0,255),
		   6, 8, 0, 0.2 );
  cv::putText(img_field,
	      "Y",
	      cv::Point2f( x_img_cg_, y_img_cg_-150 ),
	      cv::FONT_HERSHEY_DUPLEX,
	      1.5,
	      CV_RGB(255, 0, 0),
	      2);
}
// End of SimpleSim::drawFieldCoord



///////////////////
// Draw the robot 
///////////////////
void SimpleSim::drawRobot(cv::Mat &img_field)
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
	  real_to_image_coords( Rz_*body_corners_ref_[i]+robot_loc,
				coords_img[i] );
	}
      drawFilledPoly(img_field, coords_img, robot_color_ );
    }
  else if( robot_type_ == DIFF_DRIVE_CIRC )
    {
      cv::Point2i center_img, radius_img;
      real_to_image_coords( robot_loc, center_img );
      
      int radius = (int) (robot_radius_ * x_scale_field_);
      cv::circle(img_field, center_img, radius , robot_color_, -1 );
    }

  // Draw the drive/rear wheels
  if( wheel_width_ > 0.0 )
    {
      drawTire(img_field, lr_whl_ref_);
      drawTire(img_field, rr_whl_ref_);
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

      drawTire(img_field, whl_ref);

      // Steer and draw rf tire
      for(int i = 0; i < 4; i++ )
	whl_ref[i] = Rz_rf * rf_whl_ref_[i];

      for(int i = 0; i < 4; i++ )
	whl_ref[i] = rf_whl_center_ref_ + whl_ref[i];

      drawTire(img_field, whl_ref);

    }

     
  // Camera
  if( camera_width_ > 0.0 && camera_length_ > 0.0 )
    {
      for(int i = 0; i < 4; i++ )
	{
	  real_to_image_coords( Rz_*camera_pts_ref_[i]+robot_loc,
				coords_img[i] );
	}
      drawFilledPoly(img_field, coords_img, camera_color_ );
    }

  // Lidar
  if( lidar_radius_ref_ > 0.0 )
    {
      cv::Point2i center_img, radius_img;
      real_to_image_coords( Rz_*cv::Point2f(lidar_x_ref_, lidar_y_ref_) +
			    robot_loc, center_img );
      
      int radius = (int) (lidar_radius_ref_ * x_scale_field_);
      cv::circle(img_field, center_img, radius , lidar_color_, -1 );
    }
	     
  // Testing:end

}
// End of drawRobot




void SimpleSim::drawRobotOLD(cv::Mat &img_field)
{
  
  // Define robot key coorindates - THIS CAN BE DONE ONCE
  double x_signs[] = {-1, 1,  1, -1};
  double y_signs[] = { 1, 1, -1, -1};

  cv::Point2f robot_loc( Xr_, Yr_ );
  
  // Body corner points
  cv::Point2f body_pts_[4];
  cv::Point2i body_pts_img[4];
  for(int i = 0; i < 4; i++ )
    {
      body_pts_[i] = cv::Point2f( x_signs[i]*robot_length_/2.0 - axle_dst_,
				  - y_signs[i]*robot_width_/2.0 );
      real_to_image_coords( Rz_*body_pts_[i]+robot_loc, body_pts_img[i] );
    }
  // Need non-zero values in each channel for collision algorithm
  drawFilledPoly(img_field, body_pts_img, cv::Scalar(1,125,1) );

  // LF tire points
  cv::Point2f lf_tire_pts_[4];
  cv::Point2i lf_tire_pts_img[4];
  for(int i = 0; i < 4; i++ )
    {
      lf_tire_pts_[i] = cv::Point2f(x_signs[i]*rwheel_, trackwidth_/2.0 +
				    y_signs[i]*wheel_width_/2.0);
      real_to_image_coords( Rz_*lf_tire_pts_[i]+robot_loc, lf_tire_pts_img[i] );
    }
  drawFilledPoly(img_field, lf_tire_pts_img, cv::Scalar(0,0,0) );     


  // RF tire points
  cv::Point2f rf_tire_pts_[4];
  cv::Point2i rf_tire_pts_img[4];
  for(int i = 0; i < 4; i++ )
    {
      rf_tire_pts_[i] = cv::Point2f(x_signs[i]*rwheel_, -trackwidth_/2.0 +
				    y_signs[i]*wheel_width_/2.0 );
      real_to_image_coords( Rz_*rf_tire_pts_[i]+robot_loc, rf_tire_pts_img[i] );
    }
  drawFilledPoly(img_field, rf_tire_pts_img, cv::Scalar(0,0,0) );

  //
  // TO DO: Caster size control
  //
  // Caster point
  cv::Point2f caster_pt_( -(robot_length_/2+axle_dst_)*0.9, 0);
  cv::Point2i caster_pt_img;
  real_to_image_coords( Rz_*caster_pt_+robot_loc, caster_pt_img );
  cv::circle(img_field, caster_pt_img, 15, cv::Scalar(0,0,0), -1, cv::LINE_8 );

  //
  // TO DO: Draw camera after camera capture
  //
  // Camera points
  cv::Point2f camera_pts_[4];
  cv::Point2i camera_pts_img[4];
  for(int i = 0; i < 4; i++ )
    {
      camera_pts_[i] = cv::Point2f( robot_length_/2.0 - axle_dst_ +
				    x_signs[i]*0.04, 
				    y_signs[i]*0.02 );
      real_to_image_coords( Rz_*camera_pts_[i]+robot_loc, camera_pts_img[i] );
    }

  //drawFilledPoly(img_field, camera_pts_img, cv::Scalar(0,0,255) );
  
}
// End of SimpleSim::drawRobotOLD



void SimpleSim::drawTire(cv::Mat &img_field, cv::Point2f *whl_ref)
{
  cv::Point2f robot_loc(Xr_,Yr_);
  cv::Point2i coords_img[4];
    
  // LR tire points
  for(int i = 0; i < 4; i++ )
    {
      real_to_image_coords( Rz_*whl_ref[i] + robot_loc, coords_img[i] );
    }
  drawFilledPoly(img_field, coords_img, tire_color_ );
  
  cv::Point2i center = 0.5*(coords_img[0] + coords_img[3]);
  int radius = (int) sqrt( pow(center.x-coords_img[0].x,2) +
			   pow(center.y-coords_img[0].y,2) );
  cv::circle(img_field, center, radius, tire_color_, -1 );
  
  center = 0.5*(coords_img[1] + coords_img[2]);
  radius = (int) sqrt( pow(center.x-coords_img[1].x,2) +
		       pow(center.y-coords_img[1].y,2) );
  cv::circle(img_field, center, radius, tire_color_, -1 );
 
}
// End of drawTire



/////////////////////////////////////////////////////
// Draw border around carmera view on playing field
/////////////////////////////////////////////////////
void SimpleSim::drawOutlineCameraView(cv::Mat &img_field)
{
  // Draw camera view frame on field (after camera output image)
  for (int i = 0; i < 4; i++)
    cv::line(img_field, camera_corners_img_[i], camera_corners_img_[(i+1)%4],
	     camera_color_, 4);
}
// End of SimpleSim::drawOutlineCameraView




////////////////////////////
// Draw filled polynominal
////////////////////////////
void SimpleSim::drawFilledPoly(cv::Mat &img, const cv::Point2i vertices[],
			       const cv::Scalar color )
{
  // Find vertices and draw filled poly
  cv::Point corners[1][4];
  for( int i = 0; i < 4; i++ )
    {
      corners[0][i] = vertices[i];
    }
  const cv::Point* ppt[1] = { corners[0] };
  int npt[] = { 4 };
  cv::fillPoly( img, ppt, npt, 1, color, 8 );
}
// End of SimpleSim::drawFilledPoly


//////////////////////////////////////////////
// Draw polygon obstruction on playing field
//////////////////////////////////////////////
void SimpleSim::drawPolygonObstruction(cv::Mat &img_field,
				       const std::vector<cv::Point2f> pts)
{
  cv::Point2i draw_pts[4];
  
  // Draw filled polygon
  for(int i = 0; i < 4; i++ )
    {
      // Get points
      cv::Point2f pt1 = pts[i];
      cv::Point2f pt2 = pts[(i+1)%4];

      cv::Point pt1i, pt2i;
      real_to_image_coords(pt1, pt1i);
      real_to_image_coords(pt2, pt2i);
      fillLidarScanLine(pt1, pt2 );
      draw_pts[i] = pt1i;
    }

  // Draw with proper depth
  if( img_field.channels() == 3 )
    drawFilledPoly(img_field, draw_pts, wall_color_);
  else
    drawFilledPoly(img_field, draw_pts, cv::Scalar(255));
    
  

}
// End of SimpleSim::drawPolygonObstruction



////////////////////////////////////////////////
// Draw circular obstructions on playing field
///////////////////////////////////////////////
void SimpleSim::drawCircularObstruction(cv::Mat &img_field,
					const cv::Point2f center,
					const double radius )
{
  // Convert data to image coordates
  cv::Point2i center_img;
  real_to_image_coords( center, center_img );
  cv::Point2i rad_img;
  real_to_image_coords( center + cv::Point2f(radius,0.0), rad_img );


  // Draw with proper depth
  if( img_field.channels() == 3 )
    cv::circle(img_field, center_img, rad_img.x-center_img.x, obs_color_, -1 );
  else
    cv::circle(img_field,center_img,rad_img.x-center_img.x,cv::Scalar(255),-1);
    
  
  
}
// End of SimpleSim::drawCircularObstruction



/////////////////////////////
// Draw the lidar scan data
/////////////////////////////
void SimpleSim::drawLidarScan(cv::Mat &img_field)
{
  // Get the source location
  cv::Point2i src;
  real_to_image_coords(cv::Point2f(Xr_, Yr_), src);
  
  for(int theta = 0; theta < 360; theta++)
    {
      if( scan_data_[theta] > 0.0 && scan_data_[theta] < lidar_max_dist_ )
	{
	  cv::circle(img_field,scan_loc_img_[theta],lidar_marker_size_,
		     lidar_color_, -1);
	}
      
    }
  
}
// End of SimpleSim::drawLidarScan



/*******************************************************************************
*
* Reference frame functions
*
*******************************************************************************/

//////////////////////////////////////
// Set the global rotation matrix Rz
//////////////////////////////////////
void SimpleSim::setRotationZ()
{
   Rz_ = cv::Matx22f( cos(Theta_), -sin(Theta_),
		      sin(Theta_),  cos(Theta_));
}
// End of setRotationZ



//////////////////////////////////
// Set the lidar global location
//////////////////////////////////
void SimpleSim::setLidarGlobalLocation()
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
void SimpleSim::setRobotRefPoints()
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
void SimpleSim::getCameraImage(cv::Mat &img_field)
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
  out_cam[1] = cv::Point2f( 640,   0 );
  out_cam[2] = cv::Point2f( 640, 480 );
  out_cam[3] = cv::Point2f(   0, 480 );

  // ==> INCOMPLETE <==
  // ==> WORK TO UPDATE CAMERA VIEW <==
  // Determine if any obstructions are in the camera view
  for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
    {
      for( int j = 0; j < 4; j++ )
	{
	  if( pointInsidePolygon( camera_corners_, 4, rect_obs_corners_[i][j]))
	    ;
	    //ROS_INFO("Rect %d inside camera view", i );
	}
    }
   
  // Get the Perspective Transform Matrix i.e. lambda
  cv::Mat lambda( 2, 4, CV_32FC1 );
  lambda = cv::getPerspectiveTransform( field_cam, out_cam );
  
  // Apply the Perspective Transform just found to the src image
  cv::Scalar ext_wall_color(0,0,0);
  if( exterior_walls_ )
    ext_wall_color = wall_color_;
  
  cv::warpPerspective(img_field,img_cam_,lambda,cv::Size(640,480),
		      cv::INTER_LINEAR, cv::BORDER_CONSTANT,
		      ext_wall_color );
		      
}
// End of SimpleSim::getCameraImage



//////////////////////////////////////
// Get the camera corners - Not Used
/////////////////////////////////////
void SimpleSim::getCameraCornersOLD(cv::Mat &img_field)
{
  // Set up rotation matrices
  cv::Vec3f camera_position(0.0, 0.0, Zc_);
  cv::Matx33f Ry( cos(camera_phi_), 0, sin(camera_phi_),
		         0,         1,         0,
                 -sin(camera_phi_), 0, cos(camera_phi_));
  
  cv::Matx33f Rz33( cos(Theta_), -sin(Theta_), 0,
		    sin(Theta_),  cos(Theta_), 0,
		           0,           0,     1);
  
  // Get the viewing corners
  double y_signs[] = {1, -1, -1,  1};
  double z_signs[] = {1,  1, -1, -1};
  for(int i = 0; i < 4; i++)
    {
      double x_length = sqrt(camera_f_*camera_f_ + 32.0*32.0 + 24.0*24.0);
      cv::Vec3f xhat;
      xhat[0] = 1.0*camera_f_ / x_length;
      xhat[1] = y_signs[i] * 32.0 / x_length;
      xhat[2] = z_signs[i] * 24.0 / x_length;
      cv::Vec3f camera_unit_vec = Rz33*Ry*xhat;
      double d_length = -camera_position[2] / camera_unit_vec[2];
      cv::Vec3f proj_pt = camera_position + camera_unit_vec * d_length;
      cv::Point2i proj_img;
      real_to_image_coords( cv::Point2f(proj_pt[0],proj_pt[1]), proj_img );
      camera_corners_[i].x = proj_pt[0];
      camera_corners_[i].y = proj_pt[1];
      camera_corners_img_[i] = proj_img;
    }

}
// End of SimpleSim::getCameraCornersOLD



///////////////////////////
// Get the camera corners
///////////////////////////
void SimpleSim::getCameraCorners()
{
  // Set up rotation matrices
  cv::Point2f offset(Xr_, Yr_);

  cv::Point2f corners_rot[4];

  for( int i = 0; i < 4; i++ )
    {
      camera_corners_[i] = Rz_*camera_corners_ref_[i] + offset;
      real_to_image_coords( camera_corners_[i], camera_corners_img_[i] );
    }

}
// End of SimpleSim::getCameraCorners




/////////////////////////////////////
// Get the reference camera corners
/////////////////////////////////////
void SimpleSim::getRefCameraCorners()
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
// End of SimpleSim::getCameraCorners



/*******************************************************************************
*
* Lidar functions
*
*******************************************************************************/

//////////////////////////////////
// Zero out the lidar scan data
//////////////////////////////////
void SimpleSim::initializeLidarScan()
{
  // Initialize the scan values
  for(int i = 0; i < 360; i++ )
    {
      scan_data_[i] = 0.0;
      scan_loc_img_[i].x = 0;
      scan_loc_img_[i].y = 0;
    }
}
// End of SimpleSim::initializeLidarScan
  

//////////////////////////////////////////////////////////////
// Fill the lidar scan data with distances to exterior walls
//////////////////////////////////////////////////////////////
void SimpleSim::fillLidarScanWalls()
{
  // Get field dimensions
  int rows = img_field_master_.rows;
  int cols = img_field_master_.cols;

  float xmax = 1.0*(cols - x_img_cg_) / x_scale_field_;
  float xmin = -1.0*x_img_cg_ / x_scale_field_;
  float ymax = 1.0*y_img_cg_ / y_scale_field_;
  float ymin = -1.0*(rows - y_img_cg_) / y_scale_field_;

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
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 1
  for( int theta = theta_endpts[1]; theta < theta_endpts[2]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (ymax - Yl_ ) / cos(CV_PI/180.0*(90.0-th));
      cv::Point2f scan_loc(Xl_+scan_data_[theta]*sin(CV_PI/180.0*(90.0-th)),
			   ymax);
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 2
  for( int theta = theta_endpts[2]; theta < theta_endpts[3]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (ymax - Yl_ ) / cos(CV_PI/180.0*(th-90.0));
      cv::Point2f scan_loc( Xl_-scan_data_[theta]*sin(CV_PI/180.0*(th-90.0)),
						      ymax);
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 3
  for( int theta = theta_endpts[3]; theta < theta_endpts[4]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Xl_ - xmin ) / cos(CV_PI/180.0*(180.0-th));
      cv::Point2f scan_loc( xmin, Yl_ +
			    scan_data_[theta]*sin(CV_PI/180.0*(180.0-th)));
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }
  
  // Fill lidar data - section 4
  for( int theta = theta_endpts[4]; theta < theta_endpts[5]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Xl_ - xmin ) / cos(CV_PI/180.0*(th-180.0));
      cv::Point2f scan_loc( xmin, Yl_ -
			    scan_data_[theta]*sin(CV_PI/180.0*(th-180.0)));
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 5
  for( int theta = theta_endpts[5]; theta < theta_endpts[6]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Yl_ - ymin ) / cos(CV_PI/180.0*(270.0-th));
      cv::Point2f scan_loc( Xl_-scan_data_[theta]*sin(CV_PI/180.0*(270.0-th)),
			    ymin );
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }


  // Fill lidar data - section 6
  for( int theta = theta_endpts[6]; theta < theta_endpts[7]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (Yl_ - ymin ) / cos(CV_PI/180.0*(th-270.0));
      cv::Point2f scan_loc( Xl_+scan_data_[theta]*sin(CV_PI/180.0*(th-270.0)),
			    ymin );
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }

  // Fill lidar data - section 7
  for( int theta = theta_endpts[7]; theta < theta_endpts[8]; theta++ )
    {
      double th = theta-th_off;
      scan_data_[theta] = (xmax - Xl_ ) / cos(CV_PI/180.0*(-th));
      cv::Point2f scan_loc( xmax,
			    Yl_-scan_data_[theta]*sin(CV_PI/180.0*(-th)));
      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
    }
}
// End SimpleSim::fillLidarScanWalls



//////////////////////////////////////////
// Fill lidar scan for polygon obstacles
//////////////////////////////////////////
void SimpleSim::fillLidarScanPolygonObstruction(const std::vector<cv::Point2f> pts)
{
  // Loop through sides and fill lidar scan
  for(int i = 0; i < 4; i++ )
    fillLidarScanLine(pts[i], pts[(i+1)%4]);
}
// End of SimpleSim::fillLidarScanPolygonObstruction



///////////////////////////////////////////
// Fill the scan data for a line segement
///////////////////////////////////////////
void SimpleSim::fillLidarScanLine(const cv::Point2f pta, const cv::Point2f ptb )
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
	  real_to_image_coords(scan_loc, scan_loc_img_[theta]);
	  scan_data_[theta] = dist;
	  //ROS_INFO("TH: %d, Dist: %.2f", theta, dist);
	}
    }
}
// End of SimpleSim::fillLidarScanLine



//////////////////////////////////////////////
// Fill lidar scan for circular obstructions
//////////////////////////////////////////////
void SimpleSim::fillLidarScanCircularObstruction(const cv::Point2f center,
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
	      real_to_image_coords(scan_loc, scan_loc_img_[theta]);
	      scan_data_[theta] = dist;
	    }

	}
      else
	{
	  ROS_ERROR("Error in lidar to circular object.");
	}
    }
  
}
// End of SimpleSim::fillLidarScanCircularObstruction



/*******************************************************************************
*
* GPS functions
*
*******************************************************************************/

////////////////////////////////////////////////////////////////////
// Compute GPS based on the current robot location
// Source: https://www.dcode.fr/geographic-coordinates-calculation
////////////////////////////////////////////////////////////////////
void SimpleSim::computeGPSLocation()
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
void SimpleSim::computeGPSVelocity(double vx, double vy)
{
  // Rotate velocity vector and convert to mm/s
  gps_ned_vel_ = Rz_gps_vel_ * cv::Point2f(vx, vy) * 1000.0;

  // N = gps_ned_vel_.y
  // E = gps_ned_vel_.x
  // D = 0.0
  
  //ROS_INFO("NED N: %8.3e  E: %8.3e (%.2f,%.2f)", gps_ned_vel_.y,
  //	   gps_ned_vel_.x, vx, vy);
}
// End of computeGPSVelocity



/*******************************************************************************
*
* Public functions
*
*******************************************************************************/

////////////////////
// Load field file
////////////////////
void SimpleSim::loadFieldImage()
{
  // Load field
  ROS_INFO("Loading Field: %s",fname_field_img_.c_str());
  img_field_master_ = cv::imread(fname_field_img_.c_str());
  
  if( img_field_master_.empty() )
    {
      ROS_ERROR("Cound not open field image file");
      exit(0);
    }

  // Report results
  ROS_INFO("Playing Field Size: (%.2f m x %.2f m)",
	   img_field_master_.cols / x_scale_field_,
	   img_field_master_.rows / y_scale_field_ );
	 
}
// End of SimpleSim::loadFieldImage


///////////////////////////////////////////////////////////////
// Update robot location and sensor outputs for one time step
///////////////////////////////////////////////////////////////
void SimpleSim::timeStep()
{
  static bool initialize_prev_time = true;
  static ros::Time time_prev;
  static int display_counter = 0;

  /////////
  // 0) Load the parameters
  /////////
  loadObstructions(0);
    
  /////////
  // 1a) Compute time step
  /////////
  // Get timestep
  ros::Time time_now = ros::Time::now();
  if( initialize_prev_time )
    {
      time_prev = time_now;
      initialize_prev_time = false;
      return;
    }

  // Get time step and check time step
  ros::Duration time_delta = time_now - time_prev;
  double delta_t = time_delta.toSec();

  // Update previous time
  time_prev = time_now;

  /////////
  // 2) Save the current state / initialize simulation stopped flag
  /////////
  double Xr_prev = Xr_;
  double Yr_prev = Yr_;
  double Theta_prev = Theta_;
  obs_sim_stopped_ = false;
  
  /////////
  // 3) Compute the next state
  /////////
  //Xr_ = Xr_ + linear_x_ * cos(Theta_) * delta_t;
  //Yr_ = Yr_ + linear_x_ * sin(Theta_) * delta_t;

  double vx = linear_x_ * cos(Theta_);
  double vy = linear_x_ * sin(Theta_);
  Xr_ = Xr_ + vx * delta_t;
  Yr_ = Yr_ + vy * delta_t; 

  if( robot_type_ == DIFF_DRIVE_RECT || robot_type_ == DIFF_DRIVE_CIRC )
    {
      Theta_ = Theta_ + angular_z_ * delta_t;
      omega_ = angular_z_;
      //double v_r = linear_x_ + trackwidth_/2.0*angular_z_;
      //double v_l = linear_x_ - trackwidth_/2.0*angular_z_;
      //omega_ = (v_r - v_l)/trackwidth_;
    }
  else
    {
      // Compute steer angle from yaw rate request
      // OLD MEHTOD: steer_angle_ = angular_z_;
      if( fabs(angular_z_) > 0.0 ) {
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

  
  /////////
  // 4) Check if updated state is collision free
  /////////
  if( collision_model_ == COLLISION_HARD)
    {
      // Revert to previous state (freeze robot) if next state causes collision
      if( robotCollision() )
	{
	  // Get old state
	  Xr_ = Xr_prev;
	  Yr_ = Yr_prev;
	  Theta_ = Theta_prev;

	  // Set simulation stop flag
	  obs_sim_stopped_ = true;

	  // Set velocities for GPS
	  vx = 0.0;
	  vy = 0.0;
	}
    }

  /////////
  // 5) Reset if initialize flag is set
  /////////
  if( init_flag_ )
    {
      Xr_ = Xr_init_;
      Yr_ = Yr_init_;
      Theta_ = Theta_init_;
    }

  /////////
  // 6) Update global locations, odometry and GPS
  /////////
  setRotationZ();
  setLidarGlobalLocation();
  publishTransform( time_now );
  computeGPSLocation();
  computeGPSVelocity(vx, vy);
  publishGPS();

  
  /////////
  // 7) Update images
  /////////
  display_counter++;
  bool update_time_disp = false;
  if( display_counter == (int) 1.0/delta_t_target_ )
    {
      update_time_disp = true;
      display_counter = 0;
    }
  updateFieldImage(update_time_disp, delta_t);
  
}
// End of SimpleSim::timeStep



//////////////////////////
// Output robot location
//////////////////////////
void SimpleSim::displayLocation()
{

  ROS_INFO("X: %.2f m; Y: %.2f m, Z: %.2f m, Theta: %.2f deg", Xr_, Yr_,
	   0.0, Theta_*180.0/CV_PI );
  
}
// End of SimpleSim::displayLocation



/*******************************************************************************
*
* Main function
*
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_sim");
  SimpleSim simple_sim;

  ros::Time time_now = ros::Time::now();
  ros::Time time_prev = time_now;
  
  ros::Rate loop_rate((int) (1.0/simple_sim.delta_t_target_) );

  // Load the field image
  simple_sim.loadFieldImage();
  
  while (ros::ok())
  {
    // Ros spin
    ros::spinOnce();

    // Update the robot location
    simple_sim.timeStep();
      
    // Display the location
    //simple_sim.displayLocation();
    
    loop_rate.sleep();
  }

  return 0;
}
// End of main
