#!/bin/bash -xve

#sync and make
rsync -av ./ ~/catkin_ws/src/pimouse_run_corridor/

#clone pimouse_ros repository
cd ~/catkin_ws/src/
git clone --depth=1 https://github.com/blupat/pimouse_ros.git

cd ~/catkin_ws
catkin_make
