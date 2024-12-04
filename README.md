# Homemade Arm

Prerequisites: Ubuntu 20.04, ROS noetic

## How to install

```shell
git clone git@github.com:Grange007/homemade_arm.git
cd homemade_arm/src/
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
cd ..
catkin_make
source ./devel/setup.sh
roslaunch ave_arm_backpack ave_arm_backpack.launch 
```

To run the trajectory executor:

```python-repl
rosrun arm_motor_interface traj_executor.py
```
