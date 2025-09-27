#!/bin/bash

# ROS2 setup script for codebotler_amrl_impl

echo "Setting up ROS2 environment for codebotler_amrl_impl..."

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS2 Humble environment"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo "Sourced ROS2 Galactic environment"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "Sourced ROS2 Foxy environment"
else
    echo "Warning: No ROS2 environment found. Please install ROS2 first."
    exit 1
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Setup GroundingDINO
echo "Setting up GroundingDINO..."
ut_jackal_path=$(realpath third_party/ut_jackal)
echo $ut_jackal_path
graph_nav_path=$(realpath third_party/ut_jackal/graph_navigation)
echo $graph_nav_path

cd third_party/GroundingDINO
pip install -q -e .

mkdir weights/
cd weights/
if test -f groundingdino_swint_ogc.pth; then
  echo "model weights already exists, skipping installation"
else
  echo "downloading model weights"
  wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
fi
cd ../../

# TODO: take care of transitioning this part properly yourselves
# External ROS1 dependencies that need ROS2 equivalents
echo "Setting up external dependencies..."
echo "Note: ut_jackal and graph_navigation need to be converted to ROS2"

# Build the workspace with colcon
echo "Building workspace with colcon..."
colcon build --symlink-install

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Give execute permissions to all scripts
echo "Setting execute permissions..."
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd src/
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../

echo "ROS2 setup complete!"
echo "To run the system, use: ros2 launch codebotler_amrl_impl start_all.py"
