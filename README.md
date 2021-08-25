# Trajectory Optimization
TODO

## DEMO
TODO

## Prerequsite
Please follow [PX4 Gazebo](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo) to install the vehicle model, and make it compatible with your current ROS.
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd /path/to/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
```
add the following script to ```~/.bashrc```
```
source Tools/setup_gazebo.bash $<PX4-Autopilot_clone> $<PX4-Autopilot_clone>/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<PX4-Autopilot_clone>
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<PX4-Autopilot_clone>/Tools/sitl_gazebo
```

Also, install the [drone_gazebo](https://github.com/Zhefan-Xu/drone_gazebo) and the official [gazebo_models](https://github.com/osrf/gazebo_models) for gazebo models:
```
cd ~/catkin_ws
git clone https://github.com/Zhefan-Xu/drone_gazebo
cd path/to/save/gazebo_models
git clone https://github.com/osrf/gazebo_models
```
add the following to the ```~/.bashrc```
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/drone_gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:path/to/gazebo_models
```

Please install [cerlab_uav](https://github.com/Zhefan-Xu/cerlab_uav) for simulation environments.

## Installation
TODO

## How to Use
TODO
