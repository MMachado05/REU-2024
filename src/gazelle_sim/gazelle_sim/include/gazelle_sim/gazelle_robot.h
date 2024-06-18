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


#ifndef GAZELLE_ROBOT_H_
#define GAZELLE_ROBOT_H_

#include <ros/ros.h>

// Include sensor messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
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

// Constants
#define DIFF_DRIVE_RECT 0
#define DIFF_DRIVE_CIRC 1
#define ACKER_STEER_RECT 2

class GazelleRobot
{
 public:
  GazelleRobot(std::string robot_name, ros::NodeHandle *nh);

  GazelleRobot(std::string robot_name, ros::NodeHandle *nh,
	       int tmp );

  ~GazelleRobot();
  bool loadROSParameters(ros::NodeHandle *nh, bool init_flag);
  bool initializeROS(ros::NodeHandle *nh);
  void initializeRobotRefPoints( void );
  
  // Camera publisher
  cv::Mat img_cam_;
  image_transport::Publisher camera_view_pub_;  

  // Utility functions
  std::string getName( void );
  bool isRectBody( void );
  void getRobotBoundaryRect( std::vector<std::vector<cv::Point2f>> &rect_vec,
			     int idx );
  void getRobotBoundaryCirc(std::vector<cv::Point2f> &center_vec,
			    std::vector<double> &radius_vec,
			    int idx );
  void setLocation(double x, double y);
  void setLocation(double x, double y, double theta);
  void incrementRotation(double inc_theta);
  bool checkParameters(std::string rbt_name, std::string err_str);
    
  // Public draw functions
  void drawRobot(cv::Mat &img_field, int color_flag,
		 double pixels_per_meter);
  void getCameraImage(cv::Mat &img_field,
		      bool exterior_walls,
		      cv::Scalar wall_color);
  void drawOutlineCameraView(cv::Mat &img_field);

  // Public lidar functions
  void initializeLidarScan();
  void fillLidarScanWalls( int rows, int cols,
			   double pixels_per_meter,
			   int x_img_cg, int y_img_cg );
  void fillLidarScanPolygonObstruction(const std::vector<cv::Point2f> pts);
  void fillLidarScanLine(const cv::Point2f pta, const cv::Point2f ptb );
  void fillLidarScanCircularObstruction(const cv::Point2f center,
					const double radius );
  void drawLidarScan(cv::Mat &img_field,
		     float lidar_max_dist);
  void drawPenPath(cv::Mat &img_field);
  
  // Pen fucntions
  void initializePen();
  void addPenPoint();
    
  // Publish functions
  void publishLidarScan(float lidar_max_dist);
  void publishGPS();
  void publishTransform(const ros::Time time_now);

  // Solver functions
  void resetState();
  void saveCurrentState();
  void revertState();
  void computeNextState( double delta_t );
  void updateNextState( ros::Time time_now );
  
 private:

  // ROS node handle
  ros::NodeHandle *nh_;

  // ROS Topic Publishers
  ros::Publisher odom_pub_;
  ros::Publisher lidar_scan_pub_;
  ros::Publisher gps_pub_;
  
  // ROS Topic Subscribers
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber cmd_pos_sub_;

  // ROS topics
  nav_msgs::Odometry odom_;

  // Transport 
  std::vector<image_transport::ImageTransport> it_vec_;
  
  //////////////
  // Variables
  //////////////

  // Robot type
  std::string robot_name_;
  int robot_type_;
  
  // Robot location variables
  double Xr_init_;
  double Yr_init_;
  double Theta_init_;
  double Xr_;
  double Yr_;
  double Theta_;
  double vx_;
  double vy_;
  cv::Matx22f Rz_;
  double Xr_prev_;
  double Yr_prev_;
  double Theta_prev_;

  // GPS variables
  double latitude_;
  double longitude_;
  // double north_angle_;
  // cv::Point2f north_unit_vec_;
  cv::Matx22f Rz_gps_vel_;
  // double earth_radius_;
  cv::Point2f gps_ned_vel_;
  
  // Camera parameters and location
  double Zc_;                           // Height
  double camera_phi_;                   // Rotation about local y-axis
  int camera_f_;                        // Focal length (pixels)
  int camera_img_width_;                // Camera image size (pixels)
  int camera_img_height_;               // Camera image size (pixels)
  bool camera_border_;                  // Flag for camera view border
  cv::Point2f camera_corners_ref_[4];   // View corners @Xr_=Yr_=Theta_=0
  double camera_width_;                 // Physical dimension
  double camera_length_;                // Physical dimension
  double camera_x_ref_;                 // Postion reference
  double camera_y_ref_;                 // Postion reference
  cv::Point2f camera_pts_ref_[4];       // Drawing reference

  // Lidar locations and parameters
  double Xl_;
  double Yl_;
  double lidar_x_ref_;
  double lidar_y_ref_;
  double lidar_radius_ref_;

  // Define twist inputs and angular velocity
  double linear_x_;
  double angular_z_;
  double omega_;
  double steer_angle_;

  // Define robot dimensions
  double robot_width_;
  double robot_length_;
  double robot_radius_;
  int robot_width_img_;
  int robot_length_img_;
  double rwheel_;
  double wheel_width_;
  double trackwidth_;
  double axle_dst_ = 0;
  double wheelbase_;
  double max_steer_;
  cv::Point2f body_corners_ref_[4];
  cv::Point2f lr_whl_ref_[4];
  cv::Point2f rr_whl_ref_[4];
  cv::Point2f lf_whl_center_ref_;
  cv::Point2f rf_whl_center_ref_;
  cv::Point2f lf_whl_ref_[4];
  cv::Point2f rf_whl_ref_[4];

  // Define camera view
  cv::Point2f camera_corners_[4];
  cv::Point2i camera_corners_img_[4];

  // Lidar scan
  float scan_data_[360] = { 0.0 };
  cv::Point2i scan_loc_img_[360];

  // Odometry variables
  double pose_cov_[36];

  // Pen variables
  std::vector<cv::Point2i> pen_points_;
  int pen_width_;
  
  // Colors - BGR
  cv::Scalar robot_color_;
  cv::Scalar tire_color_;
  cv::Scalar camera_color_;
  cv::Scalar lidar_color_;
  cv::Scalar pen_color_;


  ////////////////////////
  // Function Prototypes
  ////////////////////////

  // Parameter functions
  bool loadROSIntParameter(std::string topic, int &var );
  bool loadROSDoubleParameter(std::string topic, double &var );
  bool loadROSStringParameter(std::string topic, std::string &var );
  void setDefaultParameters();
  bool loadParameters();

  void setParameters( std::string robot_type,
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
		      double lidar_r );

  // ROS callback function prototypes
  void cmdVelocityCb(const geometry_msgs::Twist::ConstPtr &msg);
  void cmdPositionCb(const geometry_msgs::Pose2D::ConstPtr &msg);
  
  // Drawing functions
  void drawTire(cv::Mat &img_field, cv::Point2f *whl_ref,
		int color_flag);

  // Reference frame functions
  void setRotationZ();
  void setRobotRefPoints();
  void setLidarGlobalLocation();
  
  // Camera functions
  void getCameraCorners();
  void getRefCameraCorners();
  void getRefCameraCornersORIG();
  
  // GPS functions
  void computeGPSLocation();
  void computeGPSVelocity(double vx, double vy);
 
  // Utility functions
  void displayLocation();
  
};
#endif // GAZELLE_ROBOT_H_
