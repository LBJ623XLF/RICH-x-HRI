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

