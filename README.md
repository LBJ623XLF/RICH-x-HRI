# RICH HRC

## Hardware Requirements

### Robot
- **Model:** UR5e

### Camera
- **Model:** Realsense D435

## Software Requirements

### Operating System
- **Ubuntu:** 20.04

### Robotics Middleware
- **ROS1:** Noetic

### Camera Drivers
- **Realsense SDK**

## Installation

## 1. ROS

### 1.1 Sources.list

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```


### 1.2 Keys

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 1.3 Install

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-PACKAGE
```

### 1.4 Env

```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5 Dep

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
rosdep init
rosdep update
```

## 2. UR

### 2.1 Driver

```bash
sudo apt install ros-${ROS_DISTRO}-ur-robot-driver
```


### 2.2 Build

```bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel branch.
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# activate the workspace (ie: source it)
source devel/setup.bash
```

### 2.3 Setup

**externalcontrol-x.x.x.urcap** on your UR robot

```bash
roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101
```

## 3. Moveit

### 3.1 Install

```bash
#install moveit

sudo apt install ros-noetic-moveit

#install ROS

rosdep update
sudo apt update
sudo apt dist-upgrade

#install catkin

sudo apt install ros-noetic-catkin python3-catkin-tools

#install wstool

sudo apt install python3-wstool
```


### 3.2 WS

```bash

# create
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src

# source
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials  # this is cloned in the next section
wstool update -t .

# code
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

# build
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update

cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build

source ~/ws_moveit/devel/setup.bash

echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
```

### 3.3 Rivz

```bash
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
```

### 3.4 Move

```bash
roslaunch panda_moveit_config demo.launch

rosrun moveit_tutorials move_group_python_interface_tutorial.py

roslaunch moveit_tutorials move_group_interface_tutorial.launch
```

## 4. RS

### 4.1 Dis

```bash
ssudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```


### 4.2 SDK

```bash
#dependency
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

sudo apt-get install git wget cmake build-essential

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

#librealsense2
git clone https://github.com/IntelRealSense/librealsense.git

./scripts/setup_udev_rules.sh

#build
mkdir build && cd build

cmake ../

cmake ../ -DBUILD_EXAMPLES=true

sudo make uninstall && make clean && make && sudo make install
```

### 4.3 RS

```bash
#ws
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/

git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..

#build
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.4 Launch

```bash
roslaunch realsense2_camera rs_camera.launch

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```


## 5. YOLO

### 5.1 v8

```bash
git clone https://github.com/ultralytics/ultralytics
cd ultralytics
pip3 install -r requirements.txt
```

### 5.2 Py

```bash
sudo apt update
sudo apt install python3-pip
pip3 install numpy opencv-python torch torchvision
pip3 install pyrealsense2
```
