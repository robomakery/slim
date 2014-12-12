
# ROS Setup

All commands are done from the ros subdirectory in the repo, something like:

    $ cd ~/Code/slim/ros

## [Install Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)

## Install Dependencies

    $ rosdep install --from-paths . --ignore-src --rosdistro indigo -y

## Build Packages

    $ catkin_make

You might have to do ```catkin_make``` a few times.

## Setup shell

    $ source sourceme.sh

## Run examples

    $ roslaunch clam_bringup simulation.launch
    $ roslaunch slim_gazebo warehouse_world.launch
