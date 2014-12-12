
# ROS Setup

## [Install Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)

## Install Dependencies

   $ cd ~/Code/slim/ros
   $ rosdep install --from-paths . --ignore-src --rosdistro indigo -y

## Build Packages

   $ cd ~/Code/slim/ros
   $ catkin_make

You might have to do ```catkin_make``` twice.
