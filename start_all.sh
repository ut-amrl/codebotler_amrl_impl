#!/bin/bash

codebotler_amrl_impl_path=$(pwd)
echo $codebotler_amrl_impl_path
spot_autonomy_path=$(realpath third_party/spot_autonomy)
echo $spot_autonomy_path
graph_nav_path=$(realpath third_party/spot_autonomy/graph_navigation)
echo $graph_nav_path

# Add the new path to ROS_PACKAGE_PATH
if [[ $ROS_PACKAGE_PATH != *"$codebotler_amrl_impl_path"* ]]; then
    echo "Adding $s to ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$codebotler_amrl_impl_path:$ROS_PACKAGE_PATH
fi

if [[ $ROS_PACKAGE_PATH == *"spot_autonomy"* ]]; then
    echo "Removing spot_autonomy from ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | tr ':' '\n' | grep -v "spot_autonomy" | paste -sd: -)
fi
# Add the new path to ROS_PACKAGE_PATH
if [[ $ROS_PACKAGE_PATH != *"$spot_autonomy_path"* ]]; then
    echo "Adding $spot_autonomy_path to ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$spot_autonomy_path:$ROS_PACKAGE_PATH
fi

if [[ $ROS_PACKAGE_PATH == *"graph_navigation"* ]]; then
    echo "Removing graph_navigation from ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | tr ':' '\n' | grep -v "graph_navigation" | paste -sd: -)
fi
# Add the new path to ROS_PACKAGE_PATH
if [[ $ROS_PACKAGE_PATH != *"$graph_nav_path"* ]]; then
    echo "Adding $graph_nav_path to ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$graph_nav_path:$ROS_PACKAGE_PATH
fi

roslaunch codebotler_amrl_impl start_all.launch
