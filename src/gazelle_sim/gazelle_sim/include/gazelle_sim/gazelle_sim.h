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


#ifndef GAZELLE_SIM_H_
#define GAZELLE_SIM_H_

#include <ros/ros.h>
#include <ros/package.h>

// Include sensor messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

// Include CV headers
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include math headers
#include <vector>
#include <cmath>
#include <math.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <gazelle_sim/GazelleSimConfig.h>

// Include Gazelle Robot class
#include "gazelle_sim/gazelle_robot.h"

#include "gazelle_sim/Spawn.h"
#include "gazelle_sim/Delete.h"
#include "gazelle_sim/Refresh.h"
#include "gazelle_sim/ListRobots.h"
#include "gazelle_sim/TeleportAbsolute.h"

// Constants
#define DIFF_DRIVE_RECT 0
#define DIFF_DRIVE_CIRC 1
#define ACKER_STEER_RECT 2

#define COLLISION_NONE 0
#define COLLISION_SOFT 1
#define COLLISION_HARD 2

////////////////////
// Robot Variables
////////////////////
extern std::vector<GazelleRobot *> robots_;
extern std::vector<std::vector<cv::Point2f>> rbt_rect_obs_corners_;
extern std::vector<cv::Point2f> rbt_circle_obs_center_;
extern std::vector<double> rbt_circle_obs_radius_;

/////////////////////
// Solver Variables
/////////////////////
// Solver time set
extern double delta_t_target_;  // The targetted time step in milliseconds

////////////////////////////////////////////////////////
// Playing Field Variables - Used in gazelle_robot.cpp
////////////////////////////////////////////////////////
// Source image filename
extern std::string fname_field_img_;

// Field Image
extern cv::Mat img_field_master_;

// Set image scaling
extern double pixels_per_meter_;
extern double img_view_scale_;
  
// Coordinate system location on field image
extern int x_img_cg_;
extern int y_img_cg_;

// GPS variables
extern double latitude_base_;
extern double longitude_base_;
extern double north_angle_;
extern cv::Point2f north_unit_vec_;
extern double earth_radius_;

// GPS waypoints
extern cv::Mat img_gps_waypoint_;
extern std::vector<cv::Point2d> gps_waypoints_;
extern std::vector<cv::Point2i> gps_waypoints_img_loc_;
extern std::vector<bool> gps_waypoints_visible_;

// Dynamic reconfigure variables
extern bool camera_border_;
extern bool lidar_active_;
extern float lidar_max_dist_;
extern int lidar_marker_size_;
extern bool display_gps_waypoints_;
extern bool exterior_walls_;
extern bool obstructions_;
extern bool dyn_load_obs_;
extern int collision_model_;
extern bool draw_origin_;
extern bool pen_down_;
extern bool reset_sim_;

// Obstruction information
extern bool obs_sim_stopped_;
extern std::vector<std::vector<cv::Point2f>> rect_obs_corners_;
extern std::vector<cv::Point2f> circle_obs_center_;
extern std::vector<double> circle_obs_radius_;

// Colors - BGR
extern cv::Scalar wall_color_;
extern cv::Scalar obs_color_;
extern cv::Scalar wall_color_collision_;
extern cv::Scalar obs_color_collision_;


////////////////////////
// Function Prototypes
////////////////////////

// Initialization function
bool gazelleSimInit(ros::NodeHandle *nh);

// Parameter functions
void setDefaultParameters();
bool loadROSParameters(ros::NodeHandle *nh);
bool loadRobots(ros::NodeHandle *nh, bool echo_on);
bool spawnRobot(ros::NodeHandle *nh, bool echo_on,
		std::string robot_name);

bool loadObstructions(ros::NodeHandle *nh, bool echo_on);
bool isXmlRpcParamNumeric( XmlRpc::XmlRpcValue xml_param );
double getXmlRpcDouble( XmlRpc::XmlRpcValue xml_param );
bool loadObstructionsList(ros::NodeHandle *nh, bool echo_on);
bool loadGPSWaypointList(ros::NodeHandle *nh, bool echo_on);

// ROS dynamic reconfigure callback function
void configCallback(gazelle_sim::GazelleSimConfig &config, uint32_t level);

// Utility functions
void generateRobot(std::string robot_name, ros::NodeHandle *nh);
int findRobot( std::string name );
bool deleteRobot( std::string name );
  
//  ROS service functions
bool serviceSpawn( gazelle_sim::Spawn::Request &req,
		   gazelle_sim::Spawn::Response &resp );

bool serviceDelete( gazelle_sim::Delete::Request &req,
		    gazelle_sim::Delete::Response &resp );

bool serviceRefreshRobots( gazelle_sim::Refresh::Request &req,
			   gazelle_sim::Refresh::Response &resp );

bool serviceListRobots( gazelle_sim::ListRobots::Request &req,
			gazelle_sim::ListRobots::Response &resp );

bool teleportRobotAbsolute(gazelle_sim::TeleportAbsolute::Request &req,
			   gazelle_sim::TeleportAbsolute::Response &resp );

// OpenCV callback functions
static void mouseClickCbStatic(int event, int x, int y, int flags,
			       void *userdata);

void mouseClickCb(int event, int x, int y, int flags);

// Coordinate mapping functions
void real_to_image_coords( const cv::Point2f &point,
			   cv::Point2i &img_point );
void image_to_real_coords( const cv::Point2i &img_point,
			   cv::Point2f &point );

// Geometric support functions
double angle2D(const cv::Point2f p1, const cv::Point2f p2);
bool pointInsidePolygon(const cv::Point2f *polygon,
			const int n, const cv::Point2f p);
bool robotCollision( );


// GPS functions
void initalizeGPSImage( );
void computeGPSImgLocation(const double lat, const double lon,
			   cv::Point2i &gps_loc);

void computeGPSLocation(const double lat, const double lon,
			double &distance, double &bearing);


// Drawing functions
void updateFieldImage(const bool update_time_disp,
		      const double delta_t);
void drawFieldCoord(cv::Mat &img_field );
void drawFilledPoly(cv::Mat &img, const cv::Point2i vertices[],
		    const cv::Scalar color );
void drawPolygonObstruction(cv::Mat &img_field,
			    const std::vector<cv::Point2f> pts,
			    const cv::Scalar color);
void drawCircularObstruction(cv::Mat &img_field,
			     const cv::Point2f center,
			     const double radius,
			     const cv::Scalar color);
void drawGPSWaypoint(cv::Mat &img_field, cv::Point2i point, int id );
//void drawLidarScan(cv::Mat &img_field);


// Main solver functions
void loadFieldImage();
void timeStep(ros::NodeHandle *nh);

#endif // GAZELLE_SIM_H_
