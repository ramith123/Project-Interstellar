# For the Demo Page, instructions on launching gazebo and Rviz

So to get started run below where the src folder is (parent folder of src) in this it should be `Project-Intersteller`

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