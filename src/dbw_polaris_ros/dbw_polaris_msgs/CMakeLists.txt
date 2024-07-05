cmake_minimum_required(VERSION 3.0.2)
project(dbw_polaris_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  BrakeCmd.msg
  BrakeReport.msg
  Gear.msg
  GearCmd.msg
  GearReject.msg
  GearReport.msg
  SteeringCmd.msg
  SteeringReport.msg
  ThrottleCmd.msg
  ThrottleReport.msg
  WatchdogCounter.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)

install(DIRECTORY bmr
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bmr"
)
