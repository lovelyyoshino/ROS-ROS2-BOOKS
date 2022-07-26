
# Visual Servoing for UAV's using Apriltags & YOLO

### Overview
This is a ROS package developed for controlled UAV descent based on camera images. This project was developed as part of  2019 NSF Student CPS-VO Challenge "No robot left behind!". The UAV development stack used here is based on OpenUAV flight stack.


**Author:**
        **[Harish Anand](https://web.asu.edu/jdas/people/harish-anand), hanand4 (at) asu (dot) edu;**
        **[Zhiang Chen](https://sese.asu.edu/node/3614), zch (at) asu (dot) edu;**


**Affiliation: [Prof. Jnaneshwar "JD" Das](https://sese.asu.edu/node/3438 "Jnaneshwar Das"), [Distributed Robotic Exploration and Mapping Systems Laboratory](https://web.asu.edu/jdas), ASU School of Earth and Space Exploration**



### Installation

#### Dependencies
This software is built on the Robotic Operating System ([ROS kinetic]) and Gazebo 7.14, which needs to be installed first.
Following commands are needed to setup OpenUAV ROS package.

 - OpenUAV setup

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
sudo apt-get -y install python-wstool python-rosinstall-generator python-catkin-tools 
wstool init ~/catkin_ws/src
rosinstall_generator --rosdistro kinetic --upstream-development mavros | tee /tmp/mavros.rosinstall
rosinstall_generator --rosdistro kinetic mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --as-root apt:false
mkdir ~/src
cd ~/src
git clone https://github.com/Moutwa/Firmware.git
cd Firmware
make posix_sitl_default
ln -s ~/src/Firmware ~/catkin_ws/src
ln -s ~/src/Firmware/Tools/sitl_gazebo ~/catkin_ws/src
cd ~/catkin_ws
catkin build
```

Add the following to the `.bashrc` file and source it (`source ~/.bashrc`). .
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=:/root/src/Firmware/Tools/sitl_gazebo/Build
export GAZEBO_MODEL_PATH=:/root/src/Firmware/Tools/sitl_gazebo/models
export GAZEBO_RESOURCE_PATH=:/root/src/Firmware/Tools/sitl_gazebo
source /usr/share/gazebo/setup.sh
```

 - Apriltag & Apriltag_ros (v2 or v3)
Install any dependencies for apriltag from [https://github.com/AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) 
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/AprilRobotics/apriltag_ros
git clone --recursive https://github.com/AprilRobotics/apriltag
```

 - YOLO 
Install the dependencies for YOLO from [https://github.com/leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros)
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros
```

#### Build
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/DREAMS-lab/nsf_cps_challenge
cp -r nsf_cps_challenge/models/* ~/.gazebo/models
catkin build
```

#### Testing
In terminal 1,
```
roslaunch nsf_cps_challenge openuav.launch
```
In terminal 2,
```
cd ~/catkin_ws/src/nsf_cps_challenge/scripts
python vs_without_yolo.py	
``` 
In terminal 3,
```
rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
rosservice call /mavros/cmd/arming "value: true"
```

Video:

![Preview Image](https://media.giphy.com/media/L1X6pi6CJGlLCts3kW/giphy.gif)

https://vimeo.com/337138301


