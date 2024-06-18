Introduction
------
GazelleSim is a ROS based multi-robot simulation environment that supports the simulation of differential drive and ackerman steer robots.  GazelleSim was created to support the development of autonomous robots in virtual classroom environments.  GazelleSim is implemented to be light, quick, efficient and easily integrated into a ROS network.

The goal of GazelleSim is to provide a purpose-driven efficient simulation environment as an alternative to using physical robots in an academic setting.  The simulation environments are targeted to be roughly the size of typical classroom with autonomous mobile robots intended to operate in such environments.  GazelleSim is a "two-dimensional+" simulation environment.  The ground plane is defined by an image that the robots drive on.  However, robots have cameras that are oriented to view the ground plane.  In addition, obstructions may be modeled that are visible to both the cameras and modeled lidar sensors.

The complete features of GazelleSim are:
* Multi-robot simulation
* Kinematic models of differential drive and ackerman steer robots
* Pinhole camera model
* 360 degree lidar model
* GPS model
* Circular and rectangular obstructions

Next: [Installation](../installation/installation.md)
