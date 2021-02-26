- [Introductions](#introductions)
  - [What is this?](#what-is-this)
  - [Who are we?](#who-are-we)
- [The Project](#the-project)
  - [Technologies used](#technologies-used)
  - [Purpose](#purpose)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Authors and acknowledgment](#authors-and-acknowledgment)

# Introductions 

## What is this?
Pick and Place Project with the Mecademic Meca500 Six-Axis Industrial Robot Arm (simulation) [More info...](#the-project)
## Who are we?
We are a group of final year [University of West Indies](https://sta.uwi.edu/) students, all doing an undergraduate degree in Computer Science expecting graduate this year (2021).

# The Project

## Technologies used
This project mainly uses the following:

- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- [Meca500 ROS Package](https://github.com/Mecademic/ROS)
- Gazebo for simulation
- MoveIt! for moving it

## Purpose
more.....
# Getting Started

## Prerequisites

## Installation

# Usage
# Authors and acknowledgment



<!-- So to get started run below where the src folder is (parent folder of src) in this it should be `Project-Intersteller`

```bash
catkin_make
```

This will generate `devel` and `build` folders.

Then run:

```bash
source <Location-of-project>/Project-Interstellar/devel/setup.bash
```
also add this to `.bashrc` so you don't have to run it every time you open a shell.

To run both Gazebo and Rviz, run:

```bash
roslaunch meca_500_moveit_config demo_gazebo.launch 
```

you should be where I am at the time of the first commit.




# For launching the Gazebo plugin

On new terminal:
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<Path of gazebo_plugin folder>/build
```

Then, make sure and add plugin to the current Gazebo world file:
```bash
<plugin name="hello_world" filename="libhello_world.so"/>
```

Finally, start the gazebo world:
```bash
$ gzserver <Location of Gazebo world> --verbose
``` -->
