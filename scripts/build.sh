#!/bin/bash

cd `dirname $0`
cd ..

colcon build  --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --continue-on-error $@