#!/bin/bash 

cd nl_to_robot_code_deploy/run
chmod +x *.sh
cd -

git submodule update --recursive --init

cd third_party/Grounded-Segment-Anything

export AM_I_DOCKER=False
export BUILD_WITH_CUDA=True
export CUDA_HOME=/usr/local/cuda-11.7/

python -m pip install -e GroundingDINO
if test -f groundingdino_swint_ogc.pth; then
  echo "model already exists, skipping installation"
else
  echo "wget model"
  wget https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
fi

cd ../amrl_msgs
make -j12 

cd ../enml
make -j12

cd ../graph_navigation
mkdir build
cd build 
cmake ..
make -j12

cd ../../webviz
mkdir build
cd build 
cmake ..
make -j12
