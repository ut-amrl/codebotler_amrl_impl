#!/bin/bash

pip install -r requirements.txt

# Build the workspace with colcon
echo "Building workspace with colcon..."
colcon build --symlink-install

# Give execute permissions to all scripts
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd src/
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../
