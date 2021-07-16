# Armer UR

This package launches the drivers of the UR family of arms for use with the [Armer Driver](https://github.com/qcr/armer).

It interfaces with the [UR drivers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) so they must be installed and built as well.


## Installation

### Preinstallation step: Install UR drivers
1. Clone the driver and description to the Armer workspace

```
cd ~/armer_ws
```
```
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver && git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

```
2. Install dependencies
```
sudo apt update -qq 
``` 
```
rosdep update 
```
```
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
git clone https://github.com/qcr/armer_ur.git src
```
3. Install the required dependencies.
```sh
rosdep install --from-paths src --ignore-src -r -y 
```
4. Build the packages.
```sh
catkin_make 
```
5. Run 
```sh
roslaunch armer_ur robot_bringup.launch 
```
 By default this will launch to control a physical UR5. To run a Swift simulation or specifiy a different UR model, the sim parameter can be set to true and the ur_model parameter can be set to the desired model such as "ur3".

