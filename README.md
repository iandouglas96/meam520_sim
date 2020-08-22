# MEAM 520 Robot Arm Simulator

This is the ROS package for MEAM 520 course in Fall

## 1. Set up

- see [wiki](https://github.com/yuwei-wu/meam520_sim/wiki) for details
- systems requirements for VM: Ubuntu 16.04 and ROS Kinetics or Ubuntu 18.04 and ROS Melodic


## 2. Usage


## 2.1 clone this repo in the src of your workspace

```
$ git clone https://github.com/yuwei-wu/meam520_sim.git
$ cd ..
$ catkin_make
```

### 2.1 Launch the simulator

run:

```
roslaunch al5d_gazebo lab#.launch
```

eg:
```
roslaunch al5d_gazebo lab3.launch
```



### 2.2 Run your python code

### 2.3 Add markers in lab1


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



## 3 Interact with Matlab


If you need to connect to a different ros Nework, check here: https://github.com/yuwei-wu/meam520_sim/wiki/Connect-to-a-different-ros-Nework

[Tutorials here](https://www.mathworks.com/help/ros/ug/get-started-with-ros.html)



