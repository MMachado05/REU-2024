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


#include <ros/ros.h>

// Node handle pointer
ros::NodeHandle *nh_ptr;

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <gazelle_sim/GazelleRobotConfig.h>

// Spawn service
#include "gazelle_sim/Spawn.h"
#include "gazelle_sim/Delete.h"

// Spawn client
ros::ServiceClient client_spawn;
ros::ServiceClient client_delete;

// Dynamic reconfigure variables
gazelle_sim::GazelleRobotConfig config_;

std::string robot_name_;
double robot_type_;

double length_;
double width_;
double radius_;
double wheelbase_;
double max_steer_;
double trackwidth_;
double axle_dist_;
double rwheel_;

double camera_x_;
double camera_y_;
double Zc_;
double camera_phi_;
double camera_width_;
double camera_length_;

double lidar_x_;
double lidar_y_;
double lidar_r_;

double X_init_;
double Y_init_;
double Theta_init_;


////////////////////////////////
// Callback: Dynamic Reconfigure
////////////////////////////////
void configCallback(gazelle_sim::GazelleRobotConfig &config, uint32_t level)
{
    config_ = config;

    // Set configuraion variables
    robot_name_ = config_.robot_name;
    robot_type_ = config_.robot_type;

    length_ = config_.length;
    width_ = config_.width;
    radius_ = config_.radius;
    wheelbase_ = config_.wheelbase;
    max_steer_ = config_.max_steer;
    trackwidth_ = config_.trackwidth;
    axle_dist_ = config_.axle_dist;
    rwheel_ = config_.rwheel;

    camera_x_ = config_.camera_x;
    camera_y_ = config_.camera_y;
    Zc_ = config_.camera_z;
    camera_phi_ = config_.camera_phi;
    camera_width_ = config_.camera_width;
    camera_length_ = config_.camera_length;

    lidar_x_ = config_.lidar_x;
    lidar_y_ = config_.lidar_y;
    lidar_r_ = config_.lidar_r;

    X_init_ = config_.x_init;
    Y_init_ = config_.y_init;
    Theta_init_ = config_.theta_init;

    // Write parameters
    if( config_.robot_type == 0 )
      {
	nh_ptr->setParam(config_.robot_name+"/robot_type/","diff_drive_circ");
      }
    else if( config_.robot_type == 1 )
      {
	nh_ptr->setParam(config_.robot_name+"/robot_type/","diff_drive_rect");
      }
    else if( config_.robot_type == 2 )
      {
	nh_ptr->setParam(config_.robot_name+"/robot_type/","acker_steer_rect");
      }


    nh_ptr->setParam(config_.robot_name+"/robot_length",config_.length);
    nh_ptr->setParam(config_.robot_name+"/robot_width",config_.width);
    nh_ptr->setParam(config_.robot_name+"/robot_radius",config_.radius);
    nh_ptr->setParam(config_.robot_name+"/wheelbase",config_.wheelbase);
    nh_ptr->setParam(config_.robot_name+"/max_steer",config_.max_steer);
    nh_ptr->setParam(config_.robot_name+"/trackwidth",config_.trackwidth);
    nh_ptr->setParam(config_.robot_name+"/axle_dist",config_.axle_dist);
    nh_ptr->setParam(config_.robot_name+"/rwheel",config_.rwheel);
    nh_ptr->setParam(config_.robot_name+"/wheel_width",config_.wheel_width);

    nh_ptr->setParam(config_.robot_name+"/camera_x",config_.camera_x);
    nh_ptr->setParam(config_.robot_name+"/camera_y",config_.camera_y);
    nh_ptr->setParam(config_.robot_name+"/camera_z",config_.camera_z);
    nh_ptr->setParam(config_.robot_name+"/camera_phi",config_.camera_phi);
    nh_ptr->setParam(config_.robot_name+"/camera_f",config_.camera_f);
    nh_ptr->setParam(config_.robot_name+"/camera_width",config_.camera_width);
    nh_ptr->setParam(config_.robot_name+"/camera_length",config_.camera_length);

    nh_ptr->setParam(config_.robot_name+"/lidar_x",config_.lidar_x);
    nh_ptr->setParam(config_.robot_name+"/lidar_y",config_.lidar_y);
    nh_ptr->setParam(config_.robot_name+"/lidar_r",config_.lidar_r);

    // Fill the service request
    gazelle_sim::Spawn::Request req;
    gazelle_sim::Spawn::Response resp;
    
    req.name = config_.robot_name;

    req.x =  config_.x_init;
    req.y = config_.y_init;
    req.theta = config_.theta_init;

    // Call service
    ROS_INFO("Ready to spawn robot...");
    gazelle_sim::Delete srv_del;
    srv_del.request.name = config_.robot_name;
    client_delete.call(srv_del);
    
    gazelle_sim::Spawn srv_spawn;
    srv_spawn.request = req;
    client_spawn.call(srv_spawn);
    
    return;

}
// End of configCallback





/*******************************************************************************
*
* Main function
*
*******************************************************************************/
int main(int argc, char* argv[])
{
  // Initialize node
  ros::init(argc, argv, "gazelle_robot_node");
  ros::NodeHandle nh;
  nh_ptr = &nh;

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<gazelle_sim::GazelleRobotConfig> server_;
  server_.setCallback(boost::bind(&configCallback, _1, _2));

  // Load defaults
  server_.getConfigDefault(config_);

  // Set up service clients
  client_spawn = nh.serviceClient<gazelle_sim::Spawn>("spawn");
  client_delete = nh.serviceClient<gazelle_sim::Delete>("delete");
  
  ros::spin();
  
  return 0;
}
// End of main
