#!/bin/bash

cd `dirname $0`
cd ..

source ./install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=./config/fastdds.xml

ros2 launch game_controller launch.py
