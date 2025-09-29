cmake_minimum_required(VERSION 3.8)
project(codebotler_amrl_impl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# External dependencies
find_package(amrl_msgs REQUIRED)
find_package(cobot_codebotler_actions REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## ROS2 Python setup (no equivalent needed for ament_cmake)

################################################
## Declare ROS messages, services and actions ##
################################################

## ROS2 message, service, and action generation
## Actions are automatically generated from .action files

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Actions are now provided by cobot_codebotler_actions package

################################################
## ROS2 dynamic reconfigure parameters ##
################################################

## ROS2 uses rclcpp parameters instead of dynamic reconfigure
## Parameters are handled through the node's parameter interface

###########
## Build ##
###########

## Install Python scripts
install(PROGRAMS
  src/actions.py
  src/gui.py
  src/zero_shot_object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

## Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

## Install data files
install(FILES
  data.yaml
  DESTINATION share/${PROJECT_NAME}/
)

###################################
## ament specific configuration ##
###################################
ament_package()

#############
## Testing ##
#############

## ROS2 testing setup
## Use ament_cmake testing framework for C++ tests
## Use pytest for Python tests
