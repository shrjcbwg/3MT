# 3MT

3MT is developed aiming to test mutli-module based UAV system via Metamorphic Testing. 
The modified vresions of [AirSim](https://github.com/microsoft/AirSim), [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) and [Ego-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) are used.

## Setup and Config

### Prerequisites
1. The project has been tested on 18.04(ROS Melodic). Follow the documents to install [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on your ubuntu.

<!-- 2. -->

### Build AirSim


```
  sudo apt-get install gcc-8 g++-8
  sudo apt-get install ros-melodic-tf2-sensor-msgs
  ros-melodic-tf2-geometry-msgs ros-melodic-mavros*
  git clone https://github.com/shrjcbwg/AirSim.git
  cd AirSim
  ./setup.sh
  ./build.sh
  cd ros
  catkin build
  catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```

### Build AirSim

```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/shrjcbwg/Fast-Planner.git
  cd ../ 
  catkin_make
```

### Build 3MT
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/shrjcbwg/3MT.git
  cd ../
  catkin_make
```

