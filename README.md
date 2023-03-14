<p align='center'>
    <h1 align="center">Fundaments of Robotics Project</h1>
    <p align="center">
    Project for the Fundaments of Robotics course at the University of Trento A.Y. 2022/2023
    </p>
    <p align='center'>
    Developed by:<br>
    De Martini Davide <br>
    Duong Anh Tu <br>
    Zamberlan Giulio
    </p>   
</p>

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Project Description](#project-description)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [How to run the project](#how-to-run-the-project)
  - [Setup](#setup)
  - [Running](#running)
- [Known Issues](#known-issues)


## Project Description
The goal of this project was to develop an autonomous robot that can perform pick and place tasks. The manipulator is an Ur5 that uses a Zed cam for the perception. The pick and place task consists in picking different types of "lego" blocks and placing them in their corresponding position. The robot is able to detect the blocks and perform the pick and place task autonomously.

![final.gif](test/final.gif)

## Project Structure
The project is structured as follows:
- `motion` -> contains the catkin project for the motion planner and the task manager
  - `include` -> has the header files
  - `msg` -> has the custom messages
  - `src` -> has the source files
  - CMakeLists.txt -> It is the CMake file for the project
  - pakage.xml -> It is the package file for the project 
- `vision` -> contains the visions scripts and weights
  - `dataset` -> contains the dataset
  - `scripts` -> contains the scripts for the vision
  - `weights` -> contains the weights 
  
## Installation
The project has been developed and tested on Ubuntu 20.04 with ROS Noetic, also we used the [locosim](https://github.com/mfocchi/locosim) repository for the ur5 simulation. The installation of the project is the following:
1) Clone the [locosim](https://github.com/mfocchi/locosim) repository and follow the instructions to install it
2) Clone this repository in the `ros_ws/src` folder of the catkin workspace
3) Compile the project with the following command:
```BASH
cd ~/ros_ws
catkin_make install
source install/setup.bash
```
4) Install the vision dependencies with the following command:
- Install YOLOv5 dependencies
   
```BASH
cd ~
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip3 install -r requirements.txt
```
- Intall the other dependencies
```BASH
pip install torchvision==0.13.0
```


## How to run the project
### Setup
At first we have to modify a file in the locosim project in order to be able to use the gripper. To do that we have to modify the file `~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/params.py`
and change the line 32:
```PYTHON
'gripper_sim': True, 
```
Then to have the lego model in the world you have to add them:
```
cd ~/ros_ws/src/robotic_project
cp -r models ~/ros_ws/src/locosim/ros_impedance_controller/worlds/models
```
And add the custom world file
```BASH
cp lego.world ~/ros_ws/src/locosim/ros_impedance_controller/worlds
```
Last thing is to modify the ur5_generic.py file in the locosim project adding the following line at line 72
```PYTHON
self.world_name = 'lego.world'
```
Feel free to modify the world file in order to add more lego blocks and test it
Now we are able to run the project.
### Running
For running the project you need to run the following commands:
1) Run in one window the locosim simulation with the following command:
```BASH
python3 ~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py
```
2) Run in another window the task manager with the following command:
```BASH
rosrun motion taskManager
```
3) Run in another window the motion planner with the following command:
```BASH
rosrun motion motionPlanner
```
4) Run in another window the vision node with the following command:
```BASH
cd ~/ros_ws/src/robotics_project/vision/scripts
python3 vision.py
```

## Known Issues
If your pc has a old graphic card that has not a lot of VRAM errors can be generated when running the vision node, to solve this you have to add these line
in the `LegoDetect.py` file (add it after the imports):
```PYTHON
torch.cuda.empty_cache()
torch.backends.cudnn.enabled = False
```