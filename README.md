# Implementation of the CodeBotler Action Server on UT AMRL Jackal
## Pre-requisites
You should have the following built and on your `ROS_PACKAGE_PATH`:
1. [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
2. [amrl_maps](https://github.com/ut-amrl/amrl_maps)
3. zed_wrapper
4. ouster_ros
5. pointcloud_to_laserscan
6. [ros actions](action/Ask.action): use catkin_make to create

## Setup
1. Clone this repository
1. Add it to your `ROS_PACKAGE_PATH` in `.bashrc`
1. Run `setup.sh` to install dependencies

## Usage
1. Run `start_all.sh` (which launches `start_all.launch`) to launch all the nodes