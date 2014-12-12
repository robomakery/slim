
# ROS Setup

All commands are done from the ros subdirectory in the repo, something like:

    $ cd ~/Code/slim/ros

## [Install Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)

## Install Dependencies

    $ rosdep install --from-paths . --ignore-src --rosdistro indigo -y

## Build Packages

    $ catkin_make

You might have to do ```catkin_make``` a few times.  This is what you should see at the end:

    [100%] Built target block_pick_place_moveit_server
    Linking CXX executable /home/dylan/Code/slim/ros/devel/lib/clam_pick_place/simple_pick_place
    [100%] Built target simple_pick_place
    dylan@ubuntu-trusty:~/Code/slim/ros$ 

## Setup shell

    $ source sourceme.bash

## Run examples

    $ roslaunch clam_bringup simulation.launch
    $ roslaunch slim_gazebo warehouse_world.launch
