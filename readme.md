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

sudo apt-get install libpcl-all ros-indigo-moveit-full



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

Then catkin_make

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

