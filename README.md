# MEAM 520 Robot Arm Simulator

This is the ROS package for MEMA 520 course in Fall

## Set up

- see [wiki](https://github.com/yuwei-wu/meam520_sim/wiki) for details
- systems requirements: Ubuntu 16.04 and ROS Kinetics or Ubuntu 18.04 and ROS Melodic



## Usage

### 1. Launch the simulator

run:

```
roslaunch al5d_gazebo lab#.launch
```

eg:
```
roslaunch al5d_gazebo lab3.launch
```



### 2. Run your python code

### 3. Add markers in lab1


open the lanuch file "al5d_.launch" in the launch folder, in line 4-7 you will see:

```
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find al5d_gazebo)/worlds/al5d.world"/>
    <!-- more default parameters can be changed here -->
  </include>
```

change the al5d.world to la1.world as below:


```
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find al5d_gazebo)/worlds/lab1.world"/>
    <!-- more default parameters can be changed here -->
  </include>
```

then you can relaunch it and see four markers in the environment




