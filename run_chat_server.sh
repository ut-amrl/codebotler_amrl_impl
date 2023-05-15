#!/bin/bash
echo $(dirname $(realpath $0))
cd $(dirname $(realpath $0))

cd third_party/robot_commands
python interface-server.py