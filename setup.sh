#!/bin/bash 

git submodule update --init --recursive
pip install -r requirements.txt

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

# Build the workspace with colcon
echo "Building workspace with colcon..."
colcon build --symlink-install

# Give execute permissions to all scripts
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd src/
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../