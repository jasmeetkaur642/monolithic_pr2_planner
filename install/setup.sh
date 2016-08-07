#!/bin/bash

echo "Installing dependencies from github"
wstool init ../.. .rosinstall;
cd ../../;
wstool update;

echo "Indigo version of octomap_server does not work with this package"
echo "Installing debian packages using apt-get"
rosdep install --from-paths . --ignore-src --rosdistro=indigo

# orocos_kdl (Not a catkin package, so add it as a separate library in cmakelist)
# Go to every cloned package and run rosdep.
# ompl needs to be separately installed.

# Eigen
# livgsl0-dev
# ompl
# libsdl1.2-dev 
# pcl_conversions

