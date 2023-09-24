# Package Initialization
## Cloning
Once the package is cloned please enter the artefacts_interbotix_moveit2 repository and run the following command

```
git submodule update --init --recursive
```
## Before Calling Launchiles
Before launching any files please ensure that the following commands have been run in the terminal

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
source /usr/share/gazebo/setup.bash
```

# Running Launchfiles
## Gazebo with Moveit (Real)

```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 use_gazebo:=true hardware_type:=actual
```

## Gazebo with Moveit (No Objects)

```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 use_gazebo:=true hardware_type:=gz_classic
```

## Gazebo with Moveit (Square Block)

```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 use_gazebo:=true hardware_type:=gz_classic world_filepath:=/home/decarabas/artefacts_interbotix_movit2/src/artefacts_demo_control/world/artefacts_demo_world.sdf
```
Please not that when running the command the world_filepath has to be replaced with the relative path on your machine to the world

# Running Nodes

## Full Pick and Place

```
ros2 run artefacts_demo_control artefacts_control
```

## Calling Joints

```
ros2 run artefacts_demo_control ex_joint_goal
```

## Calling Gripper

```
ros2 run artefacts_demo_control ex_gripper_command
```
