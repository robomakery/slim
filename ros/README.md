
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
    $ roslaunch slim_bringup rviz.launch

## Using MoveIt in Rviz

From a clean environment:
    $ roslaunch slim_bringup rviz_simulation.launch"

You now should see the arm in rviz and be able to do planning via the tabs in the bottom left of rviz.

If there's any failure in the above command, try running the following commands individual in separate terminal windows in order to isolate any problems.
    $ roslaunch clam_bringup lowlevel_simulator.launch
    (new initialized terminal)
    $ roslaunch clam_moveit_config move_group.launch
    (new initialized terminal)
    $ roslaunch clam_moveit_config moveit_rviz.launch


## Gazebo

From a clean environment:

    $ roslaunch clam_gazebo clam_world.launch
    (new initialized terminal)
    $ roscd clam_controller
    (new initialized terminal)
    (have gazebo on your screen before you run the next command - it happens pretty fast)
    $ python scripts/pose_cobra.py
    
Current Issues:

* arm keeps moving after being directed to go to a pose.  Issue with PID settings?
* can't use moveit - topic namespaces for controllers don't match between lowlevel_simulator.launch (no namespace) and gazebo controllers (all in /clam namespace)
* can we use the lowlevel_simulator controllers for gazebo?
