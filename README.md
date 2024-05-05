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
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel branch.
$ git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```
# Setting up a UR Robot for `ur_robot_driver`

## Prepare the Robot

1. **Install the External Control URCap:**
   Install the `externalcontrol-x.x.x.urcap` on your UR robot. You can find the URCap and installation instructions [here](URL-to-externalcontrol).

2. **URCap and Program Setup:**
   For instructions on creating a program with URCap and setting up specific models like e-Series robots, refer to the individual tutorials:
   - [Setup a CB3 robot](URL-to-CB3-setup)
   - [Setup an e-Series robot](URL-to-e-Series-setup)
   - For setting up tool communication on e-Series robots, see the [tool communication setup guide](URL-to-tool-communication-setup).

## Prepare the ROS PC

Ensure that the `ur_robot_driver` is installed on your ROS PC. This can be done either by installing the Debian package or by building from source inside a catkin workspace.

## Extract Calibration Information

To utilize the factory calibration for precise control:

1. **Extract Calibration:**
   Even though not strictly necessary, extracting calibration data from your robot is recommended to avoid end-effector positional errors. Use the following script to extract this information:
   ```bash
   roslaunch ur_calibration calibration_correction.launch \
     robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"


