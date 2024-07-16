^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_polaris_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2024-01-02)
------------------
* Warn when the unsupported DBW2 system is detected at runtime and suggest the correct package
* Contributors: Kevin Hallenbeck

1.1.2 (2023-09-11)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

1.1.1 (2023-05-10)
------------------
* Bump firmware versions
* Specify Python3 for scripts (no more ROS Melodic with Python2)
* Add P702 platform (2021+ F-150)
* Contributors: Kevin Hallenbeck

1.1.0 (2022-11-30)
------------------
* Bump firmware versions
* Change unsigned vehicle speed to signed vehicle velocity
* Periodically publish DBW enabled status in addition to latched and on change
* Contributors: Kevin Hallenbeck

1.0.2 (2022-03-08)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

1.0.1 (2021-09-20)
------------------
* Bump firmware versions
* Add reserved bits
* Improve printing of license info
* Add user control of alert
* Contributors: Kevin Hallenbeck

1.0.0 (2021-05-12)
------------------
* Bump firmware versions
* C++17 and std::clamp()
* Remove ROS Kinetic support
* Populate brake/throttle/steering command values even if enable is false
* Use 'steering_cmd' topic as preferred method of steering calibration
* Fix socketcan error frame lock up
* Contributors: Kevin Hallenbeck, Robert Maupin

0.0.8 (2021-03-09)
------------------
* Bump firmware versions
* License multiple features individually
* Add option to use socketcan hardware
* Contributors: Kevin Hallenbeck

0.0.7 (2021-01-14)
------------------
* Bump firmware versions
* Update brake type field to match firmware change
* Add Ford Ranger platform
* Contributors: Kevin Hallenbeck

0.0.6 (2020-11-16)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

0.0.5 (2020-11-04)
------------------
* Bump firmware versions
* Add reserved bit to prevent warnings
* Contributors: Kevin Hallenbeck

0.0.4 (2020-10-12)
------------------
* Add Accel and Gyro reports
* Contributors: Sun Hwang

0.0.3 (2020-08-17)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

0.0.2 (2020-08-06)
------------------
* Add shift control
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

0.0.1 (2020-02-28)
------------------
* Initial release (fork of dbw_fca_ros)
* Contributors: Kevin Hallenbeck
