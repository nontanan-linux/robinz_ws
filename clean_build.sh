#!/bin/sh
sudo rm -rf build/ install/ log/
colcon build --symlink-install
# . ~/robinz_ws/install/setup.bash