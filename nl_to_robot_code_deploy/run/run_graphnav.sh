#!/bin/bash

echo $(dirname $(realpath $0))
cd $(dirname $(realpath $0))

cd ../../third_party/graph_navigation
echo `pwd`

./bin/navigation velodyne_2dscan:=scan
# ./bin/navigation --dw 10.0 --cw 0.2 --fw 0.1 velodyne_2dscan:=scan