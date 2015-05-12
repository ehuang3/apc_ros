Amazon Picking Challenge
========================
APC ROS software for team Georgia Tech, lead by Eric Huang

# Install

In your repos directory

git@github.com:miloyip/rapidjson.git
cd rapidjson
mkdir build
cd build
cmake ..
make
sudo make install


In your catkin_workspace

git clone git@github.gatech.edu:ehuang3/apc_ros.git
cd apc_ros
git submodule update --init

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

sudo apt-get install ros-indigo-moveit-full
sudo apt-get install ros-indigo-openni2-camera ros-indigo-openni2-launch
sudo apt-get install binutils-dev

### ROS packages

cd catkin_ws
git repositories (These belong in your catkin_ws/src)
git clone git@github.gatech.edu:ehuang3/moveit_core.git
git clone git@github.gatech.edu:ehuang3/moveit_ros.git
cd moveit_ros
git checkout apc
git pull
cd ..
git clone git@github.gatech.edu:ehuang3/geometric_shapes.git
git clone git@github.com:ehuang3/robot_calibration.git
git clone git@github.com:ros/urdfdom.git
cd urdfdom
wget https://raw.github.com/ros-gbp/urdfdom-release/debian/ROS_DISTRO/UBUNTU_DISTRO/urdfdom/package.xml
cd urdf_parser_py
sudo python setup.py install
cd ../..

Then catkin_make

roslaunch apc_moveit_config setup_assistant.launch
click "load files" in the bottom right
go to configure
add the .setup thing at the bottom of configure
generate, then you are done

### Ceres

sudo apt-get install libgoogle-glog-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev

Do this in your repos directory

wget http://ceres-solver.org/ceres-solver-1.10.0.tar.gz
tar zxf ceres-solver-1.10.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.10.0
make -j3
make test
make install

# Matlab stuff
To install python matlab engine

in matlab, type "matlabroot"
cd to `matlabroot`/extern/engines/python
sudo python setup.py install

sudo apt-get install csh
sudo pip install pymatlab
echo "export PATH=/usr/local/MATLAB/R02014b/bin:$PATH" >> ~/.bashrc

source ~/.bashrc


## Fixing time issues
sudo apt-get install chrony
sudo ntpdate patlabor
(patlabor must have chrony installed)

## Running Stuff
roslaunch apc_moveit_config demo_movegroup.launch
roslaunch apc_moveit_config demo_rviz.launch
roslaunch apc_launch apc_kinect2.launch
