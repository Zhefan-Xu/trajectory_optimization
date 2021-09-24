# Trajectory Optimization
This repo implements the Dynamic Polynomial-based Model Predictive Control (DPMPC) planner for navigation and dynamic obstacle avoidance. The experiments are available at [YouTube link](https://youtu.be/e03kZ8Zh0AI)

https://user-images.githubusercontent.com/55560905/134603301-1045f44d-f76e-44fc-ae22-16e337172300.mp4

**Related paper: Zhefan Xu, Di Deng, Yiping Dong, Kenji Shimada, DPMPC-Planner: A real-time UAV trajectory planning framework for complex static environments with dynamic obstacles, arxiv [link](https://arxiv.org/abs/2109.07024), 2021.**

## DEMO
Please watch the Youtube Video for all experiments.

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
```
git clone https://github.com/Zhefan-Xu/cerlab_uav
```
Build all above packages
```
cd ~/catkin_ws
catkin_make
```

## Installation

Please install [Acado Toolkit](https://acado.github.io/), [Quadprog++](https://github.com/liuq/QuadProgpp).

For  [Acado Toolkit](https://acado.github.io/), please follow the scritps:
```
cd path/to/packages
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
```
For  [Quadprog++](https://github.com/liuq/QuadProgpp), please follow the scritps:
```
cd path/to/packages
git clone https://github.com/liuq/QuadProgpp.git
cmake .; make; make install
```


## How to Use
Start the simulation (Environment Tree as the example):
```
roslaunch cerlab_uav uav_simulation_dynamic_tree.launch
```
