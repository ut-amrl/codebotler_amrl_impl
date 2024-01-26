#!/bin/bash 

git submodule update --init --recursive
pip install -r requirements.txt

spot_autonomy_path=$(realpath third_party/spot_autonomy)
echo $spot_autonomy_path
graph_nav_path=$(realpath third_party/spot_autonomy/graph_navigation)
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

if [[ $ROS_PACKAGE_PATH == *"spot_autonomy"* ]]; then
    echo "Removing spot_autonomy from ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | tr ':' '\n' | grep -v "spot_autonomy" | paste -sd: -)
fi
# Add the new path to ROS_PACKAGE_PATH
if [[ $ROS_PACKAGE_PATH != *"$ut_jackal_path"* ]]; then
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

cd ../spot_autonomy
make -j$(nproc)
cd ../../

# Give execute permissions to all scripts
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd src/
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../
