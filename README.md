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
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
roslaunch realsense2_camera rs_rgbd.launch
roslaunch easy_handeye ur5_realsense_handeyecalibration.launch
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
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
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
### 5.3 Yolo-ros

```bash
#ws
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git
pip3 install -r yolov8_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build

#launch
ros2 launch yolov8_bringup yolov8.launch.py

ros2 launch yolov8_bringup yolov8_3d.launch.py

ros2 launch yolov8_bringup yolov8.launch.py model:=yolov8m-seg.pt

ros2 launch yolov8_bringup yolov8.launch.py model:=yolov8m-pose.pt

ros2 launch yolov8_bringup yolov8_3d.launch.py model:=yolov8m-seg.pt

ros2 launch yolov8_bringup yolov8_3d.launch.py model:=yolov8m-pose.pt
```

## 6. Lidar

### 6.1 Install

```bash
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git 
cd rslidar_sdk 
git submodule init 
git submodule update


sudo apt-get update 
sudo apt-get install -y libyaml-cpp-dev

sudo apt-get install -y  libpcap-dev

cd rslidar_sdk 
mkdir build && cd build 
cmake .. && make -j4 
./rslidar_sdk_node

catkin_make 
source devel/setup.bash 
roslaunch rslidar_sdk start.launch
```


### 6.2 Run

```bash
roscore 
rosbag record /topic  
./rslidar_sdk_node 
rviz 
rosbag play filename.bag 
rviz 
```

## 7. LIO-SAM

### 7.1 Install

```bash
sudo apt-get install -y ros-noetic-navigation --allow-unauthenticated 
sudo apt-get install -y ros-noetic-robot-localization 
sudo apt-get install -y ros-noetic-robot-state-publisher 

wget -0 ~/ros1_slam_3d_ws/gtsam.zip \ 
https://github.com/borglab/gtsam/archive/4.0.2.zip  
cd ~/ros1_slam_3d_ws/ && unzip gtsam.zip -d ~/ros1_slam_3d_ws/ 
cd gtsam-4.0.2 
mkdir build && cd build 
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \ 
-DGTSAM_USE_SYSTEM_EIGEN=ON .. 
sudo make install -j4 

cd /usr/local/lib/ 
sudo mv libmetis-gtsam.so /opt/ros/noetic/lib/ 

cd ~/ros1_slam_3d_ws/src 
git clone https://github.com/TixiaoShan/LIO-SAM.git 
cd lio-sam 
cd .. 
catkin_make 

cd ~/ros1_slam_3d_ws 
gedit src/LIO-SAM/config/params.yaml 
source devel/setup.bash 
roscore 
roslaunch lio_sam run.launch  
rosbag play casual_walk.bag 
```

### 7.2 Run

```bash
cd wit/wit_ros_ws 
sudo chmod 777 /dev/ttyUSB0 
source devel/setup.bash 
roslaunch wit_ros_imu rviz_and_imu.launch 


cd ~/rslidar_ws 
source devel/setup.bash 
roslaunch rslidar_sdk start.launch 
rosbag record -O 701_v.bag /wit/imu /velodyne_points  

cd ~/catkin_ws 
source devel/setup.bash 
roslaunch lidar_imu_calib calib_exR_lidar2imu.launch 

cd ~/ros1_slam_3d_ws 
source devel/setup.bash 
roslaunch lio_sam run.launch 

rosbag play 701_v.bag /imu/data:=/imu_raw /velodyne_points:=/points_raw 
```

## 8. ViSP

### 4.1 Dep

```bash
sudo apt-get install ros-humble-visp ros-humble-vision_visp

sudo apt-get install libopencv-dev libx11-dev liblapack-dev libeigen3-dev libv4l-dev \
                      libzbar-dev libpthread-stubs0-dev libdc1394-dev nlohmann-json3-dev
```


### 4.2 Install

```bash
#dependency
$echo "export VISP_WS=$HOME/visp-ws" >> ~/.bashrc
source ~/.bashrc
mkdir -p $VISP_WS
git clone https://github.com/lagadic/visp.git
mkdir visp-build; cd visp-build
cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO ../visp
make -j4; sudo make install

mkdir -p $HOME/colcon_ws/src
cd $HOME/colcon_ws/src
source /opt/ros/<version>/setup.bash
git clone https://github.com/lagadic/vision_visp.git -b rolling
```

### 4.3 Ros

```bash
cd $HOME/colcon_ws/src
git clone https://github.com/lagadic/visp_ros.git

cd $HOME/colcon_ws
colcon build --symlink-install --cmake-args -DVISP_DIR=$VISP_WS/visp-build -DCMAKE_BUILD_TYPE=Release
```

