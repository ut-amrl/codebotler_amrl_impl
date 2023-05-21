## Pre-requisites
You should have the following built and on your `ROS_PACKAGE_PATH`:
1. [amrl_msgs](git@github.com:ut-amrl/amrl_msgs.git)
1. zed_wrapper
1. ouster_ros
1. pointcloud_to_laserscan

## Setup
1. Clone this repository
1. Add it to your `ROS_PACKAGE_PATH` in `.bashrc`
1. Run `setup.sh` to install dependencies
1. Go to `third_party/robot_commands/code_generator` and create a file named `.openai_api_key` with your OpenAI API key in it

## Usage
1. Run `start_all.sh` (which launches `start_all.launch`) to launch all the nodes
1. Open `third_party/robot_commands/code_generator/interface.html` in your browser (change the ip to `10.0.0.123` if you're using the robot)
