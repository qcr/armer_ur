# Armer UR
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/suddrey-qut/armer/workflows/Build/badge.svg?branch=master)](https://github.com/suddrey-qut/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/suddrey-qut/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/suddrey-qut/armer/context:python)
[![Coverage](https://codecov.io/gh/suddrey-qut/armer/branch/master/graph/badge.svg)](https://codecov.io/gh/suddrey-qut/armer)

*To be used with the [Armer Driver](https://github.com/qcr/armer)*

This package launches the UR drivers for use with the [Armer Driver](https://github.com/qcr/armer).

It interfaces with the [UR drivers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) so they must be installed and built as well.

## Installation

### Preinstallation step: Install UR drivers
1. Clone the driver and description to the Armer workspace

```
cd ~/armer_ws && \
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver && \ 
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
```
2. Install dependencies
```
sudo apt update && \
rosdep update && \
rosdep install --from-paths src --ignore-src -y
```
3. Build the workspace
```
catkin_make
```
The URCap helper program is also required for running on a physical robot.

### Armer UR installation
1. Clone this repository into the armer_ws/src folder.

```
cd ~/armer_ws
```
```sh
git clone https://github.com/qcr/armer_ur.git src/armer_ur
```
3. Install the required dependencies.
```sh
rosdep install --from-paths src --ignore-src -r -y 
```
4. Build the packages.
```sh
catkin_make 
```

## Usage
```sh
roslaunch armer_ur robot_bringup.launch 
```
 By default this will launch to control a physical UR5. To run a Swift simulation or specifiy a different UR model, the sim parameter can be set to true and the ur_model parameter can be set to the desired model such as "ur3". For example:

```sh
roslaunch armer_ur robot_bringup.launch sim:=true ur_model:=ur3
```
