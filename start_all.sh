#!/bin/bash

# TODO: take care of transitioning this part properly yourselves
# External ROS1 dependencies that need ROS2 equivalents
ut_jackal_path=$(realpath third_party/ut_jackal)
echo $ut_jackal_path
graph_nav_path=$(realpath third_party/ut_jackal/graph_navigation)
echo $graph_nav_path

# ROS2 equivalent of ROS_PACKAGE_PATH manipulation
# This would typically be handled by colcon build and sourcing the workspace
if [[ $AMENT_PREFIX_PATH == *"ut_jackal"* ]]; then
    echo "Removing ut_jackal from AMENT_PREFIX_PATH..."
    export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -v "ut_jackal" | paste -sd: -)
fi
# Add the new path to AMENT_PREFIX_PATH
if [[ $AMENT_PREFIX_PATH != *"$ut_jackal_path"* ]]; then
    echo "Adding $ut_jackal_path to AMENT_PREFIX_PATH..."
    export AMENT_PREFIX_PATH=$ut_jackal_path:$AMENT_PREFIX_PATH
fi

if [[ $AMENT_PREFIX_PATH == *"graph_navigation"* ]]; then
    echo "Removing graph_navigation from AMENT_PREFIX_PATH..."
    export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -v "graph_navigation" | paste -sd: -)
fi
# Add the new path to AMENT_PREFIX_PATH
if [[ $AMENT_PREFIX_PATH != *"$graph_nav_path"* ]]; then
    echo "Adding $graph_nav_path to AMENT_PREFIX_PATH..."
    export AMENT_PREFIX_PATH=$graph_nav_path:$AMENT_PREFIX_PATH
fi

# Use ROS2 launch command instead of roslaunch
ros2 launch codebotler_amrl_impl start_all.py
