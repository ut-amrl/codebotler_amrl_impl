# Implementation of the CodeBotler Action Server on UT AMRL Jackal

## ROS2 Version (Current)

### Pre-requisites
You should have the following built and available in your ROS2 workspace:
1. [amrl_msgs](https://github.com/ut-amrl/amrl_msgs) - ROS2 version
2. [amrl_maps](https://github.com/ut-amrl/amrl_maps) - ROS2 version
3. zed_wrapper - ROS2 version
4. ouster_ros - ROS2 version
5. pointcloud_to_laserscan - ROS2 version
6. [cobot_codebotler_actions](https://github.com/ut-amrl/cobot/cobot_codebotler_actions) - ROS2 version (contains action definitions)
7. ut_jackal autonomy stack - launched separately

### Setup
1. Clone this repository
2. Run `./setup.sh` to install dependencies and build the workspace
3. Source the workspace: `source install/setup.bash`

### Usage
1. Launch ut_jackal autonomy stack separately
2. Run `ros2 launch codebotler_amrl_impl start_all.py` to launch the codebotler nodes
3. Or use the convenience script: `./start_all.sh`

---

## ROS1 Version (Legacy)

### Pre-requisites
You should have the following built and on your `ROS_PACKAGE_PATH`:
1. [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
2. [amrl_maps](https://github.com/ut-amrl/amrl_maps)
3. zed_wrapper
4. ouster_ros
5. pointcloud_to_laserscan
6. [cobot_codebotler_actions](https://github.com/ut-amrl/cobot/cobot_codebotler_actions): contains action definitions

### Setup
1. Clone this repository
1. Add it to your `ROS_PACKAGE_PATH` in `.bashrc`
1. Run `setup.sh` to install dependencies

### Usage
1. Run `start_all.sh` (which launches `start_all.launch`) to launch all the nodes