<p align='center'>
    <h1 align="center">Fundaments of Robotics Project</h1>
    <p align="center">
    Project for the Fundaments of Robotics course at the University of Trento A.Y. 2022/2023
    </p>
    <p align='center'>
    Developed by:<br>
    Duong Anh Tu <br>
    </p>   
</p>

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Project Description](#project-description)
- [Project Structure](#project-structure)
- [Installation](#installation)


## Project Description
The goal of this project was to develop an autonomous robot that can perform pick and place tasks. The manipulator is an Ur5 that uses a Zed cam for the perception. The pick and place task consists in picking different types of "lego-like" Mega Bloks and placing them in their corresponding position. The robot is able to detect the blocks and perform the pick and place task autonomously.

![demo.gif](logs/demo.gif)

  
## Installation
Follow instructions in [locosim](https://github.com/anhtuduong/locosim) (recommend using [Docker](https://github.com/anhtuduong/locosim#usage-with-docker)) for [Window](https://github.com/mfocchi/lab-docker/blob/master/install_docker_windows.md)

#### Clone the UR5BlokVision repo:
```bash
cd ros_ws/src/
```
```bash
git clone git@github.com:anhtuduong/UR5BlokVision.git
```

#### Compile/Install the code
Whenever you modify some of the ROS packages (e.g. the ones that contain the xacro fles inside the robot_description folder), you need to install them to be sure they are been updated in the ROS install folder.
```bash
cd ~/ros_ws/
```
```bash
catkin_make install
```

## Start the robot simulation
```bash
python3 -i /ros_ws/src/UR5BlokVision/main.py
```

## Run the robot
Open a new terminal
```bash
python3 -i /ros_ws/src/UR5BlokVision/motion/main_motion.py
```
This code will execute the robot command API, which can be found in `/ros_ws/src/UR5BlokVision/motion/action_list.py`
