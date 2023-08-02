# Meat Sorting With Vacuum Cobot
2023 RMIT Capstone Project Repository

### Team:
-Michael Tynan  
-Andrew Felici  
-Harrison Martin  

### Supervisor:
-Ehsan Asadi

## Components:  
-Expand on all of these
### UR5e Cobot  
-Images and info
### OnRobot VG10 end effector  
### Quick release coupling  
### NVIDIA Jetson Xavier NX  
### Intel Realsense D435 Depth and RGB Camera  
### 3D printed camera mount  
-Longer Bolts  
-Offset in programming  

## System Setup:  
### Ubuntu 22.04.2 (Jammy Jellyfish)  
https://releases.ubuntu.com/jammy/

### ROS2 Humble Hawksbill  
https://docs.ros.org/en/humble/index.html

### YoloV5  
https://pytorch.org/hub/ultralytics_yolov5/

### Movit 2  
https://moveit.ros.org/moveit/ros/humble/2022/06/02/MoveIt-Humble-Release.html

### Intel Realsense SDK 2.0  
https://www.intelrealsense.com/sdk-2/  
https://github.com/IntelRealSense/realsense-ros

### Custom vacuum program

## Installation Steps
### 1. Linux OS Distribution
#### 1.1 -  Ensure systemd and udev-related packages are updated before installing ROS 2  
***Manually perform this utilizing the Software Updater that is native to the OS installation.***
### 2. ROS2 Distribution
#### 2.1 - Ensure that the Ubuntu Universe repository is enabled
```
sudo apt install software-properties-common  
sudo add-apt-repository universe
```
#### 2.2 - Add the ROS 2 GPG key with apt
```
sudo apt update && sudo apt install curl -y  
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
#### 2.3 - Add the repository to your sources list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
#### 2.4 - Update the apt repository caches after setting up the repositories
```
sudo apt update
```
#### 2.5 - Ensure your system is up to date before installing new packages
```
sudo apt upgrade
```
#### 2.6 - Perform ROS2 Humble Distribution Desktop Package Install - Includes: ROS, RViz, demos, tutorials
```
sudo apt install ros-humble-desktop
```
#### 2.7 - To begin developing, source the setup script
```
source /opt/ros/humble/setup.bash
```
### 3. Intel RealSense D435 SDK 2.0
#### 3.1 - Register the server's public key
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
## 4. Installing ROS2 Wrapper for Intel Realsense
### 4.1 - Install git
```
sudo apt install git
```
### 4.2 - Install debian package from ROS servers
```
sudo apt install ros-humble-realsense2-*
```
### 4.3 - Start the camera node with ros2 launch
```
source /opt/ros/humble/setup.bash
```
```
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```
