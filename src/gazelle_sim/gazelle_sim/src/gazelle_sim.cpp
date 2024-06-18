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

#include "gazelle_sim/gazelle_sim.h"

///////////////////////////
// ROS NodeHandle Pointer
///////////////////////////
ros::NodeHandle* nh_ptr_;

////////////////////
// Robot Variables
////////////////////
std::vector<GazelleRobot *> robots_;
std::vector<std::vector<cv::Point2f>> rbt_rect_obs_corners_;
std::vector<cv::Point2f> rbt_circle_obs_center_;
std::vector<double> rbt_circle_obs_radius_;

/////////////////////
// Solver Variables
/////////////////////
// Solver time
double delta_t_target_;

////////////////////////////
// Playing Field Variables
////////////////////////////
// Source image filename
std::string fname_field_img_;

// Field Image
cv::Mat img_field_master_;

// Set image scaling
double pixels_per_meter_;
double img_view_scale_;
  
// Coordinate system location on field image
int x_img_cg_;
int y_img_cg_;

// GPS variables
double latitude_base_; // Used by gazelle_robot.cpp
double longitude_base_; // Used by gazelle_robot.cpp
double north_angle_; // Used by gazelle_robot.cpp
cv::Point2f north_unit_vec_; // Used by gazelle_robot.cpp
double earth_radius_; // Used by gazelle_robot.cpp

// GPS waypoints
cv::Mat img_gps_waypoint_;
std::vector<cv::Point2d> gps_waypoints_;
std::vector<cv::Point2i> gps_waypoints_img_loc_;
std::vector<bool> gps_waypoints_visible_;

// Dynamic reconfigure variables
gazelle_sim::GazelleSimConfig config_;
bool camera_border_ = false;
bool lidar_active_ = false;
float lidar_max_dist_ = 5;
int lidar_marker_size_ = 4;
bool display_gps_waypoints_ = false;
bool exterior_walls_ = false;
bool obstructions_ = false;
bool dyn_load_obs_ = false;
int collision_model_ = COLLISION_NONE;
bool draw_origin_ = true;
bool pen_down_ = false;
bool reset_sim_ = false;

// Obstruction information
bool obs_sim_stopped_ = false;
std::vector<std::vector<cv::Point2f>> rect_obs_corners_;
std::vector<cv::Point2f> circle_obs_center_;
std::vector<double> circle_obs_radius_;

// Colors - stored in BGR format
cv::Scalar wall_color_;
cv::Scalar obs_color_;
cv::Scalar wall_color_collision_;
cv::Scalar obs_color_collision_;


/*******************************************************************************
*
* Init function
*
*******************************************************************************/

/////////////////////////////////////////
// Initialize th Set default parameters
/////////////////////////////////////////
bool gazelleSimInit(ros::NodeHandle *nh)
{
  // Define the robot and camera default locations (in plane)
  setDefaultParameters();
  
  // Read parameter file
  if( !loadROSParameters(nh) ) exit(0);

  // Check parameter
  // if( !checkROSParameters() ) exit(0); -- NEED TO CREATE --
    
  return true;
}
// End of gazelleSimInit



/*******************************************************************************
*
* Parameter functions
*
*******************************************************************************/

///////////////////////////
// Set default parameters
//////////////////////////
void setDefaultParameters()
{
  // Set image scaling
  img_view_scale_ = 1.0;
  
  // Set simulation reset flag
  reset_sim_ = false;

  // GPS variables
  latitude_base_ = 42.474775859702966;    // Use LTU Ockham's Wedge
  longitude_base_ = -83.24920586143432;
  north_angle_ = CV_PI/2.0;
  north_unit_vec_ = cv::Point2f(0.0,1.0); // North aligned with y-axis (90 deg)
  earth_radius_ = 6371000;                // m
  
  // Colors - BGR - NEED NON-ZERO in EACH BGR FOR COLLISION MODEL
  wall_color_  = cv::Scalar(150, 150, 150);
  obs_color_   = cv::Scalar(175, 175, 175);
  wall_color_collision_ = cv::Scalar(255);
  obs_color_collision_  = cv::Scalar(255);

  // Define the time step
  delta_t_target_ = 0.050; // 50 ms

}
// End of setDefaultParameters




////////////////////////////
// Load the ROS parameters
////////////////////////////
bool loadROSParameters(ros::NodeHandle *nh)
{
  // Get gazelle_sim package dir
  std::string package_dir;
  ros::param::get("gazelle_sim_package_dir", package_dir);

  // Check for required parameters
  if(!( nh->hasParam("map_dir") && nh->hasParam("field_image_file")
	&& nh->hasParam("pixels_per_meter")
	&& nh->hasParam("x_center_img") && nh->hasParam("y_center_img")) )
    {
      ROS_ERROR("The following parameters must be provided:");
      ROS_ERROR("map_dir");
      ROS_ERROR("field_image_file");
      ROS_ERROR("pixels_per_meter");
      ROS_ERROR("x_center_img & y_center_img");
      return false;
    }

  // Get image loading dir
  std::string map_dir;
  ros::param::get("map_dir", map_dir);

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
  ros::param::get("pixels_per_meter", pixels_per_meter_);
  
  // Place field on coordinate system
  ros::param::get("x_center_img", x_img_cg_);
  ros::param::get("y_center_img", y_img_cg_);

  // Get the view image scale
  if(nh->hasParam("img_view_scale"))
    ros::param::get("img_view_scale", img_view_scale_);

  // GPS variables
  if(nh->hasParam("latitude_base")) ros::param::get("latitude_base",
						    latitude_base_);
  if(nh->hasParam("longitude_base")) ros::param::get("longitude_base",
						     longitude_base_);
  if(nh->hasParam("north_angle"))
    {
      ros::param::get("north_angle", north_angle_);
      cv::Point2f xhat{1.0, 0.0};
      double ang_rad = north_angle_;
      north_unit_vec_ = cv::Matx22f( cos(ang_rad), -sin(ang_rad),
				     sin(ang_rad),  cos(ang_rad)) * xhat;
      
    }

  std::cout << "North Vector = " << north_unit_vec_ << std::endl;

  if(nh->hasParam("earth_radius")) ros::param::get("earth_radius",
						   earth_radius_);

  // Get the time step
  if(nh->hasParam("delta_t")) ros::param::get("delta_t",
					      delta_t_target_);

  // Load GPS waypoint icon
  std::string fname_gps_img;
  fname_gps_img = package_dir + "/map/" + "gps_sym.png";
  img_gps_waypoint_ = cv::imread(fname_gps_img.c_str());
  
  if( img_gps_waypoint_.empty() )
    {
      ROS_ERROR("Cound not open GPS waypoint image file");
      exit(0);
    }

  ROS_INFO("Loading GPS Icon: %s",fname_gps_img.c_str());
  
  // Load robots
  loadRobots(nh, true);
  
  // Load the obstructions
  //loadObstructions(nh, true);
  loadObstructionsList(nh, true);

  // Load the GPS waypoint list
  loadGPSWaypointList(nh, true);
  
  // Succesful completion
  return true;
}
// End of loadROSParameters


////////////////
// Load robots
////////////////
bool loadRobots(ros::NodeHandle *nh, bool echo_on)
{
  // Parse the list or robots
  if( nh->hasParam("robot_list") )
    {
      std::vector<std::string> robot_name_list;
      nh->getParam("robot_list", robot_name_list);
      ROS_INFO("===>>> ROBOT LIST");
      for(auto i=0; i < robot_name_list.size(); i++)
	{
	  if( findRobot(robot_name_list[i]) < 0)
	    {
	      generateRobot(robot_name_list[i], nh);
	    }
	  else
	    {
	      ROS_ERROR("Must supply unique robot name [%s].",
			robot_name_list[i].c_str());
	    }
	}
    }

  // Return
  return true;
}
// End of loadRobots




////////////////
// Spawn robot
////////////////
bool spawnRobot(ros::NodeHandle *nh, bool echo_on,
		std::string robot_name )
{
  ROS_INFO("===>>> Spawn Robot: %s", robot_name.c_str() );
  generateRobot(robot_name, nh);
  
  return true;
}



////////////////////////////////////
// Load the obstrcution parameters
////////////////////////////////////
bool loadObstructions(ros::NodeHandle *nh, bool echo_on)
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
      if(nh->hasParam(str))
	{
	  if( echo_on ) ROS_INFO("Loading rect%d", i );
	  std::vector<double> tmp;
	  ros::param::get(str, tmp);

	  // Check vector length
	  if( tmp.size() == 8 )
	    {
	      // Parse points
	      std::vector<cv::Point2f> tmpvec;
	      for(int j = 0; j < 4; j++ )
		tmpvec.push_back(cv::Point2f(tmp[2*j], tmp[2*j+1]));

	      // Store rectangle
	      rect_obs_corners_.push_back(tmpvec);
	    }
	  else
	    {
	      ROS_WARN("Rectangle obstruction %d has %d entries, expecting 8.",
		       i, int(tmp.size()) );
	    }
	}
      
      sprintf(str,"circ%d",i);
      if(nh->hasParam(str))
	{
	  if( echo_on ) ROS_INFO("Loading circ%d", i );
	  std::vector<double> tmp;
	  ros::param::get(str, tmp);

	  // Check vector length
	  if( tmp.size() == 3 )
	    {
	      // Check radius
	      if( tmp[2] < 0.0 )
		{
		  ROS_WARN("Circular obstruction %d must have a positive radius.",
			   i );
		  
		}
	      else
		{
		  // Store circle
		  circle_obs_center_.push_back(cv::Point2f(tmp[0], tmp[1]));
		  circle_obs_radius_.push_back(tmp[2]);
		}
	    }
	  else
	    {
	      ROS_WARN("Circular obstruction %d has %d entries, expecting 3.",
		       i, int(tmp.size()) );
	    }

	}
    }

  return true;
}
// End of loadObstructions



///////////////////////////////////////
// Check for numeric XmlRpc parameter
///////////////////////////////////////
bool isXmlRpcParamNumeric( XmlRpc::XmlRpcValue xml_param )
{
  if( xml_param.getType() == XmlRpc::XmlRpcValue::TypeArray )
    {
      bool return_val = true;
      for (int i = 0; i < xml_param.size(); i++)
	{
	  return_val = return_val &&
	    ( xml_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt ||
	      xml_param[i].getType() == XmlRpc::XmlRpcValue::TypeDouble );
	}
      return return_val;
    }
  else
    {
      if( xml_param.getType() == XmlRpc::XmlRpcValue::TypeInt ||
	  xml_param.getType() == XmlRpc::XmlRpcValue::TypeDouble )
	return true;
    }
  return false;
}



//////////////////////////////////////
// Convert XML parameter to a double
//////////////////////////////////////
double getXmlRpcDouble( XmlRpc::XmlRpcValue xml_param )
{
  double val=0;

  if (xml_param.getType() == XmlRpc::XmlRpcValue::TypeInt )
      val = (double) int(xml_param);
  else if( xml_param.getType() == XmlRpc::XmlRpcValue::TypeDouble )
    val =  double(xml_param);
  else
    {
      ROS_ERROR("Error converting xml value to floating point.");
      exit(0);
    }

  // Return the converted value
  return val;
}



			  
//////////////////////////////////////////////////////////
// Load the obstrcution parameters in nested list format
//////////////////////////////////////////////////////////
bool loadObstructionsList(ros::NodeHandle *nh, bool echo_on)
{
  // Clear the current data
  rect_obs_corners_.clear();
  circle_obs_center_.clear();
  circle_obs_radius_.clear();


  // Check for rectangular obstruction list parameter
  if(nh->hasParam("rectlist"))
    {
      XmlRpc::XmlRpcValue rect_list;
      nh->getParam("rectlist", rect_list);

      // Loop and add rectangular obstructions
      if (rect_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
	  if( echo_on ) ROS_INFO("Loading %d rectangular obstructions", rect_list.size() );
	  for (int i = 0; i < rect_list.size(); i++)
	    {
	      if( echo_on ) ROS_INFO("Loading rectangular obstruction %d", i );
	      
	      // Check vector length
	      XmlRpc::XmlRpcValue rect = rect_list[i];
	      if( rect.size() == 8 && isXmlRpcParamNumeric(rect) )
		{
		  // Parse points
		  std::vector<cv::Point2f> tmpvec;
		  for(int j = 0; j < 4; j++ )
		    tmpvec.push_back(cv::Point2f(getXmlRpcDouble(rect[2*j]),
						 getXmlRpcDouble(rect[2*j+1])));
		  
		  // Store rectangle
		  rect_obs_corners_.push_back(tmpvec);
		}
	      else
		{
		  ROS_WARN("Rectangular obstruction %d requires 8 numerical values.", i);
		}
	    }
	}      
    }

  
  // Check for circular obstruction list parameter
  if(nh->hasParam("circlist"))
    {
      XmlRpc::XmlRpcValue circ_list;
      nh->getParam("circlist", circ_list);

      // Loop and add circular obstructions
      if (circ_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
	  if( echo_on ) ROS_INFO("Loading %d circular obstructions", circ_list.size() );
	  for (int i = 0; i < circ_list.size(); i++)
	    {
	      if( echo_on ) ROS_INFO("Loading circular obstruction %d", i );
	      
	      // Check vector length
	      XmlRpc::XmlRpcValue circ = circ_list[i];
	      if( circ.size() == 3 && isXmlRpcParamNumeric(circ) )
		{
		  // Check radius
		  double radius = getXmlRpcDouble( circ[2] );
		  if( radius < 0.0 )
		    {
		      ROS_WARN("Circular obstruction %d must have a positive radius.",
			       i );
		    }
		  else
		    {
		      // Store circle
		      circle_obs_center_.push_back(cv::Point2f(getXmlRpcDouble(circ[0]),
							       getXmlRpcDouble(circ[1])));
		      circle_obs_radius_.push_back(radius);
		    }
		}
	      else
		{
		  ROS_WARN("Circular obstruction %d requires 3 numerical values.", i);
		}
	    }
	}      
    }
  
  return true;
}
// End of loadObstructionsList



///////////////////////////////////////////////////////////
// Load the GPS waypoint parameters in nested list format
///////////////////////////////////////////////////////////
bool loadGPSWaypointList(ros::NodeHandle *nh, bool echo_on)
{
  // Clear the current data
  gps_waypoints_.clear();
  
  // Check for GPS waypoints list parameter
  if(nh->hasParam("gps_waypoints"))
    {
      XmlRpc::XmlRpcValue gps_list;
      nh->getParam("gps_waypoints", gps_list);

      // Loop and add GPS waypoints
      if (gps_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
	  for (int i = 0; i < gps_list.size(); i++)
	    {
	      if( echo_on ) ROS_INFO("Loading GPS waypoint %d", i );
	      
	      // Check vector length
	      XmlRpc::XmlRpcValue gps = gps_list[i];
	      if( gps.size() == 2 && isXmlRpcParamNumeric(gps) )
		{
		  // Store GPS waypoint
		  gps_waypoints_.push_back(cv::Point2d(getXmlRpcDouble(gps[0]),
						       getXmlRpcDouble(gps[1])));
		}
	      else
		{
		  ROS_WARN("GPS waypoint %d requires 2 numerical values.", i);
		}
	    }
	}      
    }  

  return true;
}
// End of loadGPSWaypointList




////////////////////
// Load field file
////////////////////
void loadFieldImage()
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
	   img_field_master_.cols / pixels_per_meter_,
	   img_field_master_.rows / pixels_per_meter_);
	 
}
// End of loadFieldImage



/*******************************************************************************
*
* ROS callback functions
*
*******************************************************************************/

////////////////////////
// Dynamic Reconfigure
////////////////////////
void configCallback(gazelle_sim::GazelleSimConfig &config, uint32_t level)
{
  config_ = config;

  // Set configuraion variables
  camera_border_ = config_.camera_border;
  lidar_active_ = config_.lidar_active;
  lidar_max_dist_ = config_.lidar_max_dst;
  lidar_marker_size_ = config_.lidar_mkr_size;
  display_gps_waypoints_ = config_.gps_waypoints;
  exterior_walls_ = config_.exterior_walls;
  obstructions_ = config_.obstructions;
  dyn_load_obs_ = config_.dyn_load_obs;
  collision_model_ = config_.collision_type;
  draw_origin_ = config_.draw_origin;
  pen_down_ = config_.pen_down;
  reset_sim_ = config_.reset_sim;
}
// End of configCallback


/*******************************************************************************
*
* Robot utility functions
*
*******************************************************************************/

///////////////////////////////////
// Add a robot to the robot lists 
///////////////////////////////////
void generateRobot(std::string robot_name, ros::NodeHandle *nh)
{
  // Parse the list or robots
  ROS_INFO("===>>> Generate Robot: %s", robot_name.c_str() );
  GazelleRobot * rbt_ptr = new GazelleRobot(robot_name, nh);
  robots_.push_back( rbt_ptr );

  // Build robot rectangle obstruction vector
  std::vector<cv::Point2f> tmpvec;
  tmpvec.clear();
  for(auto j = 0; j < 4; j++ )
    tmpvec.push_back(cv::Point2f(0.0, 0.0));
  // Store rectangle
  rbt_rect_obs_corners_.push_back(tmpvec);

  // Bulid robot circle obstruction vector
  rbt_circle_obs_center_.push_back(cv::Point2f(0.0, 0.0));
  rbt_circle_obs_radius_.push_back(0.0);
}
// End of generateRobot



/////////////////////////////////////////
// Find the index of a given robot name
/////////////////////////////////////////
int findRobot( std::string name )
{
  // Search for robot and return index
  for( int irbt = 0; irbt < robots_.size(); irbt++ )
    {
      if( robots_[irbt]->getName() == name )
	{
	  return irbt;
	}
    }
  

  // Robot not found, return error flag
  return -1;
}


///////////////////////////////////////
// Delete a robot from the robot list
///////////////////////////////////////
bool deleteRobot( std::string name )
{
  int idx_del = -1;
  // Search for robot and return index
  for( int irbt = 0; irbt < robots_.size(); irbt++ )
    {
      if( robots_[irbt]->getName() == name )
	{
	  idx_del = irbt;
	  break;
	}
    }

  // Robot not found, return error flag
  if( idx_del < 0 )
    {
      return false;
    }

  // Delete the robot and associated obstructions
  if( robots_.size() == 1 )
    {
      robots_.clear();
      rbt_rect_obs_corners_.clear();
      rbt_circle_obs_center_.clear();
      rbt_circle_obs_radius_.clear();
    }
  else
    {
      robots_.erase(std::next(robots_.begin(), idx_del));
      rbt_rect_obs_corners_.erase(std::next(rbt_rect_obs_corners_.begin(),
					    idx_del));
      rbt_circle_obs_center_.erase(std::next(rbt_circle_obs_center_.begin(),
					     idx_del));
      rbt_circle_obs_radius_.erase(std::next(rbt_circle_obs_radius_.begin(),
					     idx_del));
    }
  
  return true;
}




/*******************************************************************************
*
* ROS service functions
*
*******************************************************************************/

///////////////////////////////
// Service: Spawn a new robot
///////////////////////////////
bool serviceSpawn( gazelle_sim::Spawn::Request &req,
		   gazelle_sim::Spawn::Response &resp )
{
  // Check the robot name for length and uniqueness
  if( req.name.length() < 1 )
    {
      ROS_ERROR("Must supply robot name of on-zero length");
      resp.success = false;
      return false;
    }

  int irbt = findRobot( req.name );
  if( irbt >= 0 )
    {
      ROS_ERROR("Must supply unique robot name [%s].", req.name.c_str());
      resp.success = false;
      return false;
    }
  

  ROS_INFO("Spawn");
  // Add robot
  spawnRobot(nh_ptr_, false,
	     req.name );

  irbt = findRobot( req.name );
  robots_[irbt]->setLocation(req.x, req.y, req.theta);

  resp.success = true;
  return true;
}
// End of serviceSpawn



////////////////////////////
// Service: Delete a robot
////////////////////////////
bool serviceDelete( gazelle_sim::Delete::Request &req,
		    gazelle_sim::Delete::Response &resp )
{
  // Check to see if robot exists
  int irbt = findRobot( req.name );
  if( irbt < 0 )
    {
      ROS_WARN("No robot with name [%s].", req.name.c_str());
    }

  // Delete the robot
  resp.success = deleteRobot(req.name);
  return true;
}
// End of serviceDelete



/////////////////////////////////////////////
// Service: Reload all the robot parameters
/////////////////////////////////////////////
bool serviceRefreshRobots( gazelle_sim::Refresh::Request &req,
			   gazelle_sim::Refresh::Response &resp )
{
  if( req.name.length() == 0 )
    {
      ROS_INFO("Refreshing all robot parameters");
      for( int irbt = 0; irbt < robots_.size(); irbt++ )
	{
	  // Load ROS parameters and initalize robot reference points
	  robots_[irbt]->loadROSParameters(nh_ptr_, false);
	  robots_[irbt]->initializeRobotRefPoints();
	}

      // Return success
      resp.success = true;
      return true;
    }
  else
    {
      int irbt = findRobot( req.name );
      if( irbt >= 0 )
	{
	  // Refresh a single robot
	  ROS_INFO("Refreshing [%s] robot parameters", req.name.c_str() );
	  robots_[irbt]->loadROSParameters(nh_ptr_, false);
	  robots_[irbt]->initializeRobotRefPoints();


	  // Return success
	  resp.success = true;
	  return true;
	}
    }
  
  ROS_WARN("Could not refresh robot [%s].  Robot now found", req.name.c_str());
  return false;
}
// End of serviceRefreshRobots




////////////////////////////////////////////////
// Service: Generate list of the active robots
////////////////////////////////////////////////
bool serviceListRobots( gazelle_sim::ListRobots::Request &req,
			gazelle_sim::ListRobots::Response &resp )
{
  //std::vector<std::string> robot_names;
  for( int irbt = 0; irbt < robots_.size(); irbt++ )
    {
      resp.robot_names.push_back(robots_[irbt]->getName());
    }

  return true;
}
// End of serviceListRobots



//////////////////////////////////////////////////////////
// Service: Move robot a new location and rotation angle
//////////////////////////////////////////////////////////
bool teleportRobotAbsolute( gazelle_sim::TeleportAbsolute::Request &req,
			    gazelle_sim::TeleportAbsolute::Response &resp )
{
  // Find the robot
  int irbt = findRobot( req.name );
  if( irbt >= 0 )
    {
      // Set position
      robots_[irbt]->setLocation(req.x, req.y, req.theta);

      resp.success = true;
      return true;
    }


  // Return error state
  ROS_ERROR("No robot with name [%s] exists", req.name.c_str());
  resp.success = false;
  return false;
}
// End of teleportRobotAbsolute




/*******************************************************************************
*
* OpenCV callback functions
*
*******************************************************************************/

///////////////////////////////////////////
// Callback: Mouse click on playing field
///////////////////////////////////////////
void mouseClickCb(int event, int x, int y, int flags, void *userdata)
{
  cv::Point2f coords_real(0.0,0.0);
  cv::Point2i coords_img(x,y);

  // Exit if no robots are defined
  if( robots_.size() == 0 )
    return;
  
  // Move robot
  if( event == cv::EVENT_LBUTTONDOWN )
    {
      // Get coordinates
      image_to_real_coords( coords_img, coords_real );

      // Set position
      robots_[0]->setLocation(coords_real.x, coords_real.y);
    }

  // Rotation 15 degs
  if( event == cv::EVENT_RBUTTONDOWN )
    {
      robots_[0]->incrementRotation(15*CV_PI/360.0);
    }

  // Get point location
  if( event == cv::EVENT_MBUTTONDOWN )
    {
      // Get coordinates
      image_to_real_coords( coords_img, coords_real );
      ROS_INFO("Location (%.2f,%.2f)", coords_real.x, coords_real.y);
    }
}
// End of mouseClickCb





/*******************************************************************************
*
* Coordinate mapping functions
*
*******************************************************************************/

///////////////////////////////////////////////////////
// Mapping from real coordinates to image coordinates
///////////////////////////////////////////////////////
void real_to_image_coords( const cv::Point2f &point,
			   cv::Point2i &img_point )
{
  img_point.x = x_img_cg_ + (int) (point.x * pixels_per_meter_);
  img_point.y = y_img_cg_ - (int) (point.y * pixels_per_meter_);
}
// End of real_to_image_coords


///////////////////////////////////////////////////////
// Mapping from image coordinates to real coordinates
///////////////////////////////////////////////////////
void image_to_real_coords( const cv::Point2i &img_point,
			   cv::Point2f &point )
{
  point.x =  (img_point.x/img_view_scale_ - x_img_cg_) / pixels_per_meter_;
  point.y = -(img_point.y/img_view_scale_ - y_img_cg_) / pixels_per_meter_;
}
// End of GazelleSim::real_to_image_coords



/*******************************************************************************
*
* Geometric support functions
*
*******************************************************************************/

////////////////////////////////////////////////////////////////////////
// Determine the angle between two vectors (points relative to origin)
////////////////////////////////////////////////////////////////////////
double angle2D( const cv::Point2f p1, const cv::Point2f p2)
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
// End of angle2D


/////////////////////////////////////////
// Determine if a point is in a polygon
/////////////////////////////////////////
bool pointInsidePolygon(const cv::Point2f *polygon,
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
// End of pointInsidePolygon



/////////////////////////////////////////////////////////////////////////////
// Determine if the robot is in contact with exterior walls or obstructions
/////////////////////////////////////////////////////////////////////////////
bool robotCollision( )
{
  // Create blank images
  cv::Mat img_robot(img_field_master_.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat img_obs = img_robot.clone();

  // Draw first robot
  robots_[0]->drawRobot( img_robot, 1, pixels_per_meter_);
  
  // Check for collision with other robots
  for( auto i = 1; i < robots_.size(); i++ )
    {
      // Check next robot
      cv::Mat img_robot_next(img_field_master_.size(), CV_8UC1, cv::Scalar(0));
      robots_[i]->drawRobot( img_robot_next, 1, pixels_per_meter_ );

      cv::Mat img_coll;
      cv::bitwise_and( img_robot, img_robot_next, img_coll);
      if( cv::countNonZero(img_coll) )
	{
	  return true;
	}

      // No collision, add robot to robot image
      robots_[i]->drawRobot( img_robot, 1, pixels_per_meter_ );
      
    }

  // Draw exterior walls
  if( exterior_walls_ )
    cv::rectangle(img_obs, cv::Point2i(0,0),
		  cv::Point( img_obs.cols-1, img_obs.rows-1 ),
		  wall_color_collision_, 1 );
  
  // Draw the obstructions
  if( obstructions_ )
    {
      // Draw polygon obstructions
      for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
	{
	  drawPolygonObstruction(img_obs, rect_obs_corners_[i],
				 obs_color_collision_);
	}

      
      // Draw circular obstructions
      for(int i = 0; i < (int) circle_obs_center_.size(); i++ )
	{
	  drawCircularObstruction(img_obs, circle_obs_center_[i],
				  circle_obs_radius_[i],
				  obs_color_collision_);
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
// End of robotCollision




/*******************************************************************************
*
* GPS functions
*
*******************************************************************************/

///////////////////////
// initializeGPSImage
////////////////////////
void initalizeGPSImage()
{
  // Clear the current data
  gps_waypoints_img_loc_.clear();

  // Store gps information ==>> MOVE TO NEW FUNCTION!!!
  for( auto i = 0; i < gps_waypoints_.size(); i++ )
    {
      cv::Point2i img_loc;
      computeGPSImgLocation(gps_waypoints_[i].x, gps_waypoints_[i].y,
			    img_loc);

      gps_waypoints_img_loc_.push_back(img_loc);

      if( img_loc.y > img_gps_waypoint_.rows &&
	  img_loc.y < img_field_master_.rows &&
	  img_loc.x > int(img_gps_waypoint_.cols/2) &&
	  img_loc.x< (img_field_master_.cols - int(img_gps_waypoint_.cols/2)) )
	{
	  gps_waypoints_visible_.push_back(true);
	}
      else
	{
	  gps_waypoints_visible_.push_back(false);
	  ROS_WARN("Waypoint %d: (%f,%f) is outside of the playing field.",
		   i, gps_waypoints_[i].x, gps_waypoints_[i].y );
	}
    }
}
// End of initializeGPSImage



//////////////////////////
// computeGPSImgLocation
//////////////////////////
void computeGPSImgLocation(const double lat, const double lon,
			   cv::Point2i &gps_loci)
{

  // Get the distance and bearing to the gps location
  double distance, bearing;
  computeGPSLocation(lat, lon, distance, bearing);

  // Rotation by north vector location
  bearing = bearing + (CV_PI/2 - north_angle_);

  // Get location
  cv::Point2f gps_loc;
  gps_loc.x = cos(-(bearing-CV_PI/2.0))*distance;
  gps_loc.y = sin(-(bearing-CV_PI/2.0))*distance;

  // Convert to image coordinates
  real_to_image_coords(gps_loc, gps_loci);

}
// End of computeGPSImgLocation



////////////////////////////////////////////////////////////////
// Get GPS location: distance and bearing
// Source: https://www.movable-type.co.uk/scripts/latlong.html
//
// Distance
// R = 6371e3; // metres
// φ1 = lat1 * Math.PI/180; // φ, λ in radians
// φ2 = lat2 * Math.PI/180;
// Δφ = (lat2-lat1) * Math.PI/180;
// Δλ = (lon2-lon1) * Math.PI/180;
//
// a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
//   Math.cos(φ1) * Math.cos(φ2) *
//   Math.sin(Δλ/2) * Math.sin(Δλ/2);
// c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
// d = R * c; // in metres
//
// Bearing
// y = Math.sin(λ2-λ1) * Math.cos(φ2);
// x = Math.cos(φ1)*Math.sin(φ2) -
//     Math.sin(φ1)*Math.cos(φ2)*Math.cos(λ2-λ1);
// θ = Math.atan2(y, x);
// brng = (θ*180/Math.PI + 360) % 360; // in degrees
//
// where φ1, λ1 is the start point and φ2, λ2 the end point
////////////////////////////////////////////////////////////////
void computeGPSLocation(const double lat, const double lon,
			double &distance, double &bearing)
{

  // Compute distance and bearing
  // From: (φ1, λ1) = (latitude_base_, longitude_base_ )
  // To:   (φ2, λ2) = (lat, lon)

  // Distance
  double lat1 = latitude_base_ * CV_PI/180.0;
  double lon1 = longitude_base_ * CV_PI/180.0;
  double lat2 = lat * CV_PI/180.0;
  double lon2 = lon * CV_PI/180.0;
  double delta_lat = lat2 - lat1;  //  Δφ 
  double delta_lon = lon2 - lon1; //  Δλ

  double a = sin(delta_lat/2.0) * sin(delta_lat/2.0) +
    cos(lat1) * cos(lat2) * sin(delta_lon/2.0) * sin(delta_lon/2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1-a));

  distance = earth_radius_ * c; // in metres

  // Alternative approach - NOT USED 
  //d = acos( sin φ1 ⋅ sin φ2 + cos φ1 ⋅ cos φ2 ⋅ cos Δλ ) ⋅ R
  //distance = acos( sin(latitude_base_) * sin(lat) +
  //		   cos(latitude_base_) * cos(lat) * cos(delta_lon) ) *
  //  earth_radius_;
  
  // Bearing
  double y = sin(delta_lon) * cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(delta_lon);
  bearing = atan2(y, x);
  //theta = (theta + 2.0*CV_PI) % (2.0*CV_PI);

  //ROS_INFO("GPS=(%.8f, %.8f) (Th,d)=(%.8f, %.8f)", lat, lon,
  //	   theta*180.0/CV_PI, distance );


  // Compute location relative 
  // Error Check: Compute GPS (lat,lon) based on distance and bearing
  // φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
  // λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )
  // where φ is latitude, λ is longitude,
  // θ is the bearing (clockwise from north), δ is the angular distance d/R;
  // d being the distance travelled, R the earth’s radius
  // double lat2v = asin(sin(lat1)*cos(distance/earth_radius_) +
  // 		      cos(lat1)*sin(distance/earth_radius_)*cos(theta));
  // lat2v = lat2v * 180.0/CV_PI;
  //
  // double lon2v = lon1 + atan2( sin(theta)*sin(distance/earth_radius_)
  //			       *cos(lat1),
  //			       cos(distance/earth_radius_) - 
  //			       sin(distance/earth_radius_)*sin(lat));
  // lon2v = lon2v * 180.0/CV_PI;
  //
  // ROS_INFO("GPS=(%.8f, %.8f)", lat2v, lon2v );

		      
}
// End of computeGPSLocation





/*******************************************************************************
*
* Drawing functions
*
*******************************************************************************/

///////////////////////
// Update field image
///////////////////////
void updateFieldImage(const bool update_time_disp,
		      const double delta_t)
{
  static char time_str[100] = "";
  static bool callback_reg = false;
  
  // Copy over master image
  cv::Mat img_field = img_field_master_.clone();

  // Draw the coordinates
  if( draw_origin_ )
    drawFieldCoord( img_field );

  // Draw pen points
  for( auto i = 0; i < robots_.size(); i++ )
    {
      if( pen_down_ )
	{
	  robots_[i]->addPenPoint();
	  robots_[i]->drawPenPath(img_field);
	}
      else
	{
	  robots_[i]->initializePen();
	}
    }
  
  // Draw the obstructions
  if( obstructions_ )
    {
      // Draw polygon obstructions
      for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
	{
	  drawPolygonObstruction(img_field, rect_obs_corners_[i],
				 obs_color_ );
	}

      
      // Draw circular obstructions
      for(int i = 0; i < (int) circle_obs_center_.size(); i++ )
	{
	  drawCircularObstruction(img_field, circle_obs_center_[i],
				  circle_obs_radius_[i],
				  obs_color_);
	}
   }
  
  // Draw the robot
  for( auto i = 0; i < robots_.size(); i++ )
    {
      robots_[i]->drawRobot( img_field, 0, pixels_per_meter_ );
    }
  
  // Get the camera view window
  for( auto i = 0; i < robots_.size(); i++ )
    {
      robots_[i]->getCameraImage( img_field,
				  exterior_walls_,
				  wall_color_);
    }

  // Draw GPS waypoints
  if( display_gps_waypoints_ )
    {
      for( auto i = 0; i < gps_waypoints_.size(); i++ )
	{
	  if( gps_waypoints_visible_[i] )
	    drawGPSWaypoint(img_field, gps_waypoints_img_loc_[i], i);
	}
    }
  
  // Get the lidar data
  if( lidar_active_ )
    {
      // Populate the dynamic robot obstruction data
      for( auto irbt = 0; irbt < robots_.size(); irbt++ )
	{
	  if( robots_[irbt]->isRectBody() )
	    {
	      robots_[irbt]->getRobotBoundaryRect( rbt_rect_obs_corners_,
						   irbt );
	    }
	  else
	    {
	      robots_[irbt]->getRobotBoundaryCirc( rbt_circle_obs_center_,
						   rbt_circle_obs_radius_,
						   irbt );
	    }
	}

      // Fill the lidar data for exterior walls and obstructions
      for( auto irbt = 0; irbt < robots_.size(); irbt++ )
	{
	  // Initialize the scan data
	  robots_[irbt]->initializeLidarScan();
      
	  // Check for exterior walls
	  if( exterior_walls_ )
	    robots_[irbt]->fillLidarScanWalls( img_field_master_.rows,
					       img_field_master_.cols,
					       pixels_per_meter_,
					       x_img_cg_, y_img_cg_ );

	  // Check for obstructions
	  if( obstructions_ )
	    {
	      // Process polygon obstructions
	      for(int i = 0; i < (int) rect_obs_corners_.size(); i++ )
		{
		  robots_[irbt]->fillLidarScanPolygonObstruction(rect_obs_corners_[i]);
		}
	      
	      // Process circular obstructions
	      for(int i = 0; i < (int) circle_obs_center_.size(); i++ )
		{
		  robots_[irbt]->fillLidarScanCircularObstruction(circle_obs_center_[i],
							 circle_obs_radius_[i]);
		}
	    }

	}
      
      // Fill for other robots
      for( auto irbt = 0; irbt < robots_.size(); irbt++ )
	{
	  // Check for other robots
	  for( auto jrbt = 0; jrbt < robots_.size(); jrbt++ )
	    {
	      if( irbt != jrbt )
		{
		  if( robots_[jrbt]->isRectBody() )
		    {
		      // Process polygon obstructions
		      robots_[irbt]->fillLidarScanPolygonObstruction(rbt_rect_obs_corners_[jrbt]);
		    }
		  else
		    {
		      // Process circular obstructions
		      robots_[irbt]->fillLidarScanCircularObstruction(rbt_circle_obs_center_[jrbt],
								      rbt_circle_obs_radius_[jrbt]);
		    }
		}
	    }
	}
      
      // Draw the results
      for( auto irbt = 0; irbt < robots_.size(); irbt++ )
	{
	  // Draw lidar scan results
	  robots_[irbt]->drawLidarScan( img_field,
					lidar_max_dist_);
	  
	  // Publish the lidar scan
	  robots_[irbt]->publishLidarScan(lidar_max_dist_);
	}
      
    }
  
  // Draw camera bounding box
  if( camera_border_ )
    {
      for( auto i = 0; i < robots_.size(); i++ )
	{
	  robots_[i]->drawOutlineCameraView( img_field );
	}
    }
 

  
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
  if( reset_sim_ )
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

      // Clear the pen
      pen_down_ = false;
    }

  
  // Publish camera views
  for( auto i = 0; i < robots_.size(); i++ )
    {
      // Create message wrapper
      sensor_msgs::ImagePtr msg_pub;
      msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
      				   robots_[i]->img_cam_).toImageMsg();
  
      // Publish the preview image
      robots_[i]->camera_view_pub_.publish(msg_pub);
    }

  
  // Test for collision
  if( collision_model_ && (robotCollision() || obs_sim_stopped_) )
    {
      cv::Mat img_red(img_view.size(), CV_8UC3, cv::Scalar(0,0,255));
      cv::addWeighted(img_view, 0.8, img_red, 0.2, 0.0, img_view);
    }
  
  // Display playing field
  cv::imshow("Playing Field", img_view);
  cv::waitKey((int) (1.0/(2*delta_t_target_)));

  if( !callback_reg )
    {
      cv::setMouseCallback("Playing Field", mouseClickCb);
      callback_reg = true;
    }
}
// End of GazelleSim::updateFieldImage



////////////////////////////
// Draw field coordindates
////////////////////////////
void drawFieldCoord(cv::Mat &img_field )
{
  int axis_len = 150;
  int north_len = 75;
    
  // Get center of coordinate system
  cv::Point2f coord_center = cv::Point2f( x_img_cg_, y_img_cg_ );

  // Draw the X axis
  cv::arrowedLine( img_field, cv::Point2f( x_img_cg_, y_img_cg_ ),
		   cv::Point2f( x_img_cg_+axis_len, y_img_cg_ ),
		   cv::Scalar(225,0,0), 6, 8, 0, 0.2 );
  cv::putText(img_field,
	      "X",
	      cv::Point2f( x_img_cg_+axis_len, y_img_cg_ ),
	      cv::FONT_HERSHEY_DUPLEX,
	      1.5,
	      CV_RGB(0, 0, 255),
	      2);

  // Draw the Y axis
  cv::arrowedLine( img_field, cv::Point2f( x_img_cg_, y_img_cg_ ),
		   cv::Point2f( x_img_cg_, y_img_cg_-axis_len ),
		   cv::Scalar(0,0,255), 6, 8, 0, 0.2 );
  cv::putText(img_field,
	      "Y",
	      cv::Point2f( x_img_cg_, y_img_cg_-axis_len ),
	      cv::FONT_HERSHEY_DUPLEX,
	      1.5,
	      CV_RGB(255, 0, 0),
	      2);

  // Draw the north vector
  cv::Point2f end_pnt;
  end_pnt.x = x_img_cg_ + int(north_unit_vec_.x * north_len);
  end_pnt.y = y_img_cg_ - int(north_unit_vec_.y * north_len);
  
  cv::arrowedLine( img_field, cv::Point2f( x_img_cg_, y_img_cg_ ),
		   end_pnt,
		   cv::Scalar(0,255,0), 6, 8, 0, 0.2 );
  cv::putText(img_field,
	      " N",
	      end_pnt,
	      cv::FONT_HERSHEY_DUPLEX,
	      1.5,
	      CV_RGB(0, 255, 0),
	      2);
  
}
// End of drawFieldCoord



////////////////////////////
// Draw filled polynominal
////////////////////////////
void drawFilledPoly(cv::Mat &img, const cv::Point2i vertices[],
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
// End of drawFilledPoly



//////////////////////////////////////////////
// Draw polygon obstruction on playing field
//////////////////////////////////////////////
void drawPolygonObstruction(cv::Mat &img_field,
			    const std::vector<cv::Point2f> pts,
			    const cv::Scalar color )
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
      // fillLidarScanLine(pt1, pt2 ); -- NEEDS UPDATE --
      draw_pts[i] = pt1i;
    }

  // Draw with proper depth
  drawFilledPoly(img_field, draw_pts, color);
}
// End of drawPolygonObstruction



////////////////////////////////////////////////
// Draw circular obstructions on playing field
///////////////////////////////////////////////
void drawCircularObstruction(cv::Mat &img_field,
			     const cv::Point2f center,
			     const double radius,
			     const cv::Scalar color)
{
  // Convert data to image coordates
  cv::Point2i center_img;
  real_to_image_coords( center, center_img );
  cv::Point2i rad_img;
  real_to_image_coords( center + cv::Point2f(radius,0.0), rad_img );


  // Draw with proper depth
  cv::circle(img_field, center_img, rad_img.x-center_img.x, color, -1 );
}
// End of drawCircularObstruction





////////////////////////
// Draw a GPS Waypoint
///////////////////////
void drawGPSWaypoint(cv::Mat &img_field, cv::Point2i point, int id )
{
  static bool mask_built = false;
  static cv::Mat mask_inner, mask_outer;

  // Map variables
  int row = point.y;
  int col = point.x;
  
  // Build masks if needed
  if( !mask_built )
    {
      cv::inRange(img_gps_waypoint_,cv::Scalar(0,0,0),cv::Scalar(255,24,255),
		  mask_inner);
      cv::inRange(img_gps_waypoint_,cv::Scalar(0,24,0),cv::Scalar(255,255,255),
		  mask_outer);
      mask_built = true;
    }

  // Get roi
  cv::Mat img_background = img_field(cv::Rect(int(col-img_gps_waypoint_.cols/2),
					      int(row-img_gps_waypoint_.rows),
					      img_gps_waypoint_.cols,
					      img_gps_waypoint_.rows));

  // Draw GPS waypoint
  cv::Mat img_overlay;
  cv::bitwise_and(img_background, img_background, img_overlay, mask_inner);
  cv::bitwise_or(img_overlay, img_gps_waypoint_, img_overlay, mask_outer);
  img_overlay.copyTo(img_background);

  // Draw id
  std::string str = std::to_string(id);
      
  cv::putText(img_field,
	      str.c_str(),
	      cv::Point(col+5,row),
	      cv::FONT_HERSHEY_COMPLEX_SMALL,
	      1,
	      CV_RGB(0, 255, 0),
	      2);


}
// End of drawGPSWaypoint




/*******************************************************************************
*
* Solver function
*
*******************************************************************************/


///////////////////////////////////////////////////////////////
// Update robot location and sensor outputs for one time step
///////////////////////////////////////////////////////////////
void timeStep(ros::NodeHandle *nh)
{
  static bool initialize_prev_time = true;
  static ros::Time time_prev;
  static int display_counter = 0;

  
  //for( auto i = 0; i < robots_.size(); i++ )
  // {
  //  ROS_INFO("Robot %d: %s", (int) i, robots_[i]->getName().c_str());
  //}
  obs_sim_stopped_ = false;

  /////////
  // 0) Load the parameters
  /////////
  //ROS_INFO("Time Step 0");
  if( obstructions_ and dyn_load_obs_ )
    {
      //loadObstructions(nh, false);
      loadObstructionsList(nh, false);
    }
    
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
  for( auto i = 0; i < robots_.size(); i++ )
    {
      robots_[i]->saveCurrentState();
    }
  obs_sim_stopped_ = false;
  
  /////////
  // 3) Compute the next state
  /////////
  for( auto i = 0; i < robots_.size(); i++ )
    {
      robots_[i]->computeNextState(delta_t);
    }
      
  /////////
  // 4) Check if updated state is collision free
  /////////
  if( collision_model_ == COLLISION_HARD)
    {
      // Revert to previous state (freeze robot) if next state causes collision
      if( robotCollision() )
	{
	  // Set simulation stop flag
	  obs_sim_stopped_ = true;

	  // Revert robots
	  for( auto i = 0; i < robots_.size(); i++ )
	    {
	      robots_[i]->revertState();
	    }

	}
    }
  
  /////////
  // 5) Reset if initialize flag is set
  /////////
  if( reset_sim_ )
    {
      for( auto i = 0; i < robots_.size(); i++ )
      {
	robots_[i]->resetState();
      }
    }

  /////////
  // 6) Update global locations, odometry and GPS
  /////////
  for( auto i = 0; i < robots_.size(); i++ )
    {
      robots_[i]->updateNextState(time_now);
    }

  
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
  //ROS_INFO("Time Step 7b");
  updateFieldImage(update_time_disp, delta_t);
  
}
// End of GazelleSim::timeStep



/*******************************************************************************
*
* Main function
*
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gazelle_sim");

  // Initialize the gazelle sim node
  ROS_INFO("GazelleSim Node Initialized");

  // ROS NodeHandle
  ros::NodeHandle nh;
  nh_ptr_ = &nh;
  
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<gazelle_sim::GazelleSimConfig> server_;
  server_.setCallback(boost::bind(&configCallback, _1, _2));

  // Services
  ros::ServiceServer srv_spawn = nh.advertiseService("spawn",
						     serviceSpawn);

  ros::ServiceServer srv_delete = nh.advertiseService("delete",
						     serviceDelete);

  ros::ServiceServer srv_refresh = nh.advertiseService("refresh",
						     serviceRefreshRobots);

  ros::ServiceServer srv_list = nh.advertiseService("list",
						     serviceListRobots);

  ros::ServiceServer srv_tele_abs = nh.advertiseService("teleport_absolute",
							teleportRobotAbsolute);


  // Load defaults
  server_.getConfigDefault(config_);

  // Initialize GazelleSim
  gazelleSimInit(&nh);

  // Load the field image
  loadFieldImage();

  // Initialize GPS waypoint image
  initalizeGPSImage();
  
  // Set timing
  ros::Time time_now = ros::Time::now();
  ros::Time time_prev = time_now;
  ros::Rate loop_rate((int) (1.0/delta_t_target_) );

  while (ros::ok())
  {
    // Ros spin
    ros::spinOnce();

    // Update the robot location
    timeStep(&nh);
      
    loop_rate.sleep();
  }

  return 0;
}
// End of main
