#!/bin/sh
git submodule update --init --recursive
sudo apt-get install python-catkin-tools
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic