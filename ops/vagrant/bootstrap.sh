#!/usr/bin/env bash

# break on first error
set -e
# show all executing commands
set -x

echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -

echo "set grub-pc/install_devices /dev/sda" | debconf-communicate
apt-get update
apt-get upgrade -y
apt-get install -y ros-indigo-desktop-full python-rosinstall

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
if grep -Fxq "source /opt/ros/indigo/setup.bash" /home/vagrant/.bashrc
then
    echo "already added setup.bash to .bashrc"
else
    echo "source /opt/ros/indigo/setup.bash" >> /home/vagrant/.bashrc
fi
su - vagrant <<EOF
  mkdir -p ~/catkin_ws/src;
  ln -s /vagrant/slim_description /home/vagrant/catkin_ws/src/slim_description
  ln -s /vagrant/slim_gazebo /home/vagrant/catkin_ws/src/slim_gazebo
  source /opt/ros/indigo/setup.bash;
  /opt/ros/indigo/bin/catkin_init_workspace /home/vagrant/catkin_ws/src;
  /opt/ros/indigo/bin/catkin_make -C /home/vagrant/catkin_ws;
  rosdep update;
EOF
