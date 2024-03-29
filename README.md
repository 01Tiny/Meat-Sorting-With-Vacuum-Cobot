# Meat Sorting With Vacuum Cobot
2023 RMIT Capstone Project Repository

### Team:
-Michael Tynan  
-Andrew Felici  
-Harrison Martin  

### Supervisor:
-Ehsan Asadi

# Components:  
-Expand on all of these
## UR5e Cobot  
-Images and info
## OnRobot VG10 end effector  
https://onrobot.com/sites/default/files/documents/VG10_Vacuun_Gripper_User_Manual_V1.1.1.pdf  
### Turning on VG10 through digital IO port
https://forum.universal-robots.com/t/set-tool-i-o-with-ros2/24585
## Intel Realsense D435 Depth and RGB Camera  
## 3D printed camera mount  
-Longer Bolts  
-Offset in programming  
-Image of it installed and 3d model  

# System Setup:  
## Ubuntu 22.04.2 (Jammy Jellyfish)  
https://releases.ubuntu.com/jammy/
## ROS2 Humble Hawksbill  
https://docs.ros.org/en/humble/index.html
## YoloV5  
https://pytorch.org/hub/ultralytics_yolov5/
## Moveit 2  
https://moveit.ros.org/moveit/ros/humble/2022/06/02/MoveIt-Humble-Release.html    
https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html  
https://github.com/ros-planning/moveit2_tutorials/issues/719
https://github.com/ros-planning/moveit2_tutorials/tree/main  
https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html  
### Installation (from source):  
https://moveit.ros.org/install-moveit2/source/  
#### Make sure you have the latest versions of packages installed:
```
sudo apt update
sudo apt dist-upgrade
rosdep update
```
#### Source installation requires various ROS2 build tools:
```
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget && \
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
```
#### Uninstall Any Pre-existing MoveIt Debians:
```
sudo apt remove ros-humble-moveit*
```
#### Create Workspace and Source:
```
export COLCON_WS=~/ws_moveit/
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src
```
#### Download Source Code for Foxy, Galactic, Humble - (stable) :
```
git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
#### Build MoveIt (will take an hour):
```
cd ws_moveit
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```
##### If running into RAM errors when building, use this command instead (stops packages building in parallel):
```
cd ws_moveit
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```
#### Source the Colcon Workspace:
```
source ws_moveit/install/setup.bash
```
## Intel Realsense SDK 2.0  
https://www.intelrealsense.com/sdk-2/  
https://github.com/IntelRealSense/realsense-ros  
## ROS2 UR Driver
**Make sure to build from source and not use binary install**
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble#build-from-source
## Custom vacuum program
Needs to be created if time allows.
# Installation Steps
### 1. Linux OS Distribution
#### 1.1 -  Ensure systemd and udev-related packages are updated before installing ROS 2  
***Manually perform this utilizing the Software Updater that is native to the OS installation.***
## 2. ROS2 Distribution
### 2.1 - Ensure that the Ubuntu Universe repository is enabled
```
sudo apt install software-properties-common  
sudo add-apt-repository universe
```
### 2.2 - Add the ROS 2 GPG key with apt
```
sudo apt update && sudo apt install curl -y  
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
### 2.3 - Add the repository to your sources list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### 2.4 - Update the apt repository caches after setting up the repositories
```
sudo apt update
```
### 2.5 - Ensure your system is up to date before installing new packages
```
sudo apt upgrade
```
### 2.6 - Perform ROS2 Humble Distribution Desktop Package Install - Includes: ROS, RViz, demos, tutorials
```
sudo apt install ros-humble-desktop
```
### 2.7 - To begin developing, source the setup script
```
source /opt/ros/humble/setup.bash
```
#### This can be done automatically upon startup of a new shell by modifying the bash sript using this command:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
## 3. Intel RealSense D435 SDK 2.0
### 3.1 - Register the server's public key
```
sudo mkdir -p /etc/apt/keyrings  
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```
### 3.2 - Ensure apt HTTPS support is installed
```
sudo apt-get install apt-transport-https
```
### 3.3 - Add the server to the list of repositories
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```
### 3.4 - Install dependent libraries
```
sudo apt-get install librealsense2-dkms  
sudo apt-get install librealsense2-utils
```
### 3.5 - Optionally install dev and debug libraries
```
sudo apt-get install librealsense2-dev  
sudo apt-get install librealsense2-dbg
```
### 3.6 - Verify Installation 
```
realsense-viewer
```
### 3.7 - Verify that the kernel is updated
```
modinfo uvcvideo | grep "version:"
```
***If the output includes realsense in the string it is successfully updated***  
### 3.8 - Calibrate the camera
https://dev.intelrealsense.com/docs/self-calibration-for-depth-cameras
## 4. Installing ROS2 Wrapper for Intel Realsense
### 4.1 - Install git
```
sudo apt install git
```
### 4.2 - Install debian package from ROS servers
```
sudo apt install ros-humble-realsense2-*
```
## Running code
### Start the camera node with ros2 launch
```
source /opt/ros/humble/setup.bash
```
```
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```
### Start the MoveIt node
```
source /opt/ros/humble/setup.bash
```
```
cd $ros2_ws
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```
cd ~/
source ros2_ws/install/setup.bash
```
# How to Launch UR Driver and MoveIt (new working method with UR Driver installed from source in ws_moveit directory)
## Step 1
Turn on robot, load "Meat Capstone First Connection" program. Don't play program yet.
## Step 2
Plug in robot ethernet cable to laptop.
## Step 3
Open a new terminal.
## Step 4 - Terminal 1 - Change to ws_moveit directory and source
```
cd ws_moveit
source install/setup.bash
source /opt/ros/humble/setup.bash
```
## Step 5 - Launch UR Driver
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 launch_rviz:=true
```
## Step 6
Check RVIZ displays real UR robot position, then press play on program on UR5 tablet.
## Step 7
Open a new terminal.
## Step 8 - Terminal 2 - Change to ws_moveit directory and source
```
cd ws_moveit
source install/setup.bash
source /opt/ros/humble/setup.bash
```
## Step 9
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```
## Step 10
Ensure new RVIZ window displays real robot position, then drag arrow at the ebd effector of robot model to a new position.
Press plan and execute button on RVIZ and observe real robot moving.
# Testing:
### Trying to install ROS2 controller for VG10 
http://wiki.ros.org/onrobot/Tutorials/Control%20of%20an%20VG%20Gripper%20using%20the%20Modbus%20TCP%20protocol%20%28Noetic%29  
https://github.com/BrettRD/onrobot_grippers/tree/main  
### Porting ROS1 package to ROS2
https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS1-to-ROS2-porting.html  
### Activating the VG10 gripper through terminal once UR driver is running  
Pin 16 is channel A  
Pin 17 is channel B  
State 1 turns vacuum on:  
```
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 1}"  
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 1}"
```
```
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 1}" && ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 1}"
```
State 0 turns vacuum off  
```
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 0}"  
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 0}"
```
```
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 0}" && ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 0}"
```
# Testing Code
```
cd workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 launch_rviz:=true
```
```
use_tool_communication:=true tool_voltage:=24
```
