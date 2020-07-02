# MEAM 520 Robot Arm Simulator

This is the ros package for MEMA 520 course in Fall



## Usage

### 1 launch file


#### 1.1 for Lab0-Lab2, the model is with the gripper 

run:

```
roslaunch al5d_gazebo al5d.launch
```

the robot model is with the gripper


#### 1.2. For Lab 3 

the gripper is replaced by LIDAR, the arm model is changed 


```
roslaunch al5d_gazebo al5d_lidar.launch
```

then will launch the revised model


### 1.2 add markers in lab1


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




