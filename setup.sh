#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

RED='\033[0;31m'
NC='\033[0m' # No Color

git submodule update --init --recursive
pip install -r requirements.txt

spot_autonomy_path=$(realpath third_party/spot_autonomy)
echo $spot_autonomy_path
graph_nav_path=$(realpath third_party/spot_autonomy/graph_navigation)
echo $graph_nav_path

cd third_party/GroundingDINO
pip install -q -e .

mkdir weights/ || true
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

# Initialize Conda in this script
eval "$(/opt/miniconda3/bin/conda shell.bash hook)"  # This is the recommended way to initialize Conda in scripts
# deactivate conda env for building spot_autonomy
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    echo "No Conda environment is currently active."
    CURRENT_ENV=""
else
    CURRENT_ENV="$CONDA_DEFAULT_ENV"
    echo "Deactivating Conda environment: $CURRENT_ENV"
    conda deactivate
fi
cd ../spot_autonomy
make clean && make -j$(nproc)
cp launch/start_clearpath_spot.launch.example launch/start_clearpath_spot.launch
echo -e "${RED}Add Spot credentials to the start_clearpath_spot.launch file in spot_autonomy of codebotler_amrl_impl${NC}"
cd ../../
# Reactivate the previous Conda environment if it was active
if [ -z "$CURRENT_ENV" ]; then
    echo "No Conda environment was previously active. Skipping activation."
else
    echo "Reactivating Conda environment: $CURRENT_ENV"
    conda activate "$CURRENT_ENV"
fi

# Give execute permissions to all scripts
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd src/
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../
