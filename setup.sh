#!/bin/bash 

git submodule update --init --recursive
pip install -r requirements.txt

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
cd ../

# TODO: take care of transitioning this part properly yourselves
# External ROS1 dependencies that need ROS2 equivalents
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

# TODO: take care of transitioning this part properly yourselves
# External ROS1 build system that needs ROS2 equivalent
cd ../ut_jackal
# ROS2 equivalent would be: colcon build
make -j$(nproc)
cd ../../

# Give execute permissions to all scripts
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd src/
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../
