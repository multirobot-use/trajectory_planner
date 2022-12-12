#!/bin/bash

## Get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
WORKSPACE_PATH="$( cd .. ; pwd -P )"


## ROS Melodic installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update


## Catkin tools
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools


## Install necessary packages
sudo apt-get install sudo apt-get install libeigen3-dev ros-$(rosversion -d)-geodesy ros-$(rosversion -d)-joy


## Clone packages
echo "Cloning necessary packages"
cd $WORKSPACE_PATH/src

git clone https://github.com/alfalcmar/acado.git
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/grvcTeam/grvc-utils
git clone https://github.com/alfalcmar/safe_corridor_generator
git clone https://github.com/alfalcmar/trajectory_follower.git
## Install acado
echo "Installing acado"
cd $WORKSPACE_PATH/src/acado
mkdir -p build
cd build
cmake ..
make

# set bashrc
num=`cat ~/.bashrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
source $WORKSPACE_PATH/src/acado/build/acado_env.sh" >> ~/.bashrc
fi

# set zshrc
num=`cat ~/.zshrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
source $WORKSPACE_PATH/inspection_trajectory_planning/packages/acado/build/acado_env.sh" >> ~/.zshrc
fi


## Install safe_corridor (uncomment if the package is downloaded)
echo "Installing safe_corridor"
cd $WORKSPACE_PATH/src/safe_corridor_generator/thirdparty/jps3d 
mkdir -p build
cd build
cmake ..
make
cd ../..
cd DecompROS/DecompUtil
mkdir -p build
cd build
cmake ..
make


## Install and configure UAL. Only MAVROS needed. Dependencies
echo "Installing and configuring UAL. Only MAVROS needed. Install dependencies"
cd $WORKSPACE_PATH/src/grvc-ual
./configure.py


## Install MAVROS packages
echo "Installing MAVROS necessary packages"
sudo apt install -y ros-$(rosversion -d)-mavros ros-$(rosversion -d)-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager


## Set the ROS environment variables
# set bashrc
num=`cat ~/.bashrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  # set bashrc
  echo "
source $WORKSPACE_PATH/devel/setup.bash" >> ~/.bashrc
  
fi

# set zshrc
num=`cat ~/.zshrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
source $WORKSPACE_PATH/devel/setup.zsh" >> ~/.zshrc

fi


## Install PX4 for SITL simulations
echo "Installing PX4 for SITL simulations"
sudo apt install libgstreamer1.0-dev python-jinja2 python-pip
pip install numpy toml
cd $WORKSPACE_PATH/src/
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo

catkin build
