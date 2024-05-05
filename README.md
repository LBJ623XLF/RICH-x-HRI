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


