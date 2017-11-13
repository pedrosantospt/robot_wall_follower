# robot_wall_follower

**Directory Structure**
src/  
├── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake  
├── robo_control  
│   ├── CMakeLists.txt  
│   ├── package.xml  
│   ├── README.md  
│   └── robo_control.cpp  
└── robo_world  
    ├── CMakeLists.txt  
    ├── launch  
    │   ├── includes  
    │   │   ├── create.launch.xml  
    │   │   ├── kobuki.launch.xml  
    │   │   └── roomba.launch.xml  
    │   ├── spawn_world_DD_between.launch  
    │   ├── spawn_world_D_inside.launch  
    │   └── spawn_world_D_outside.launch  
    ├── package.xml  
    ├── README.md  
    └── worlds  
        ├── world_DD_robot_between.world  
        ├── world_D_robot_inside.world  
        └── world_D_robot_outside.world  

**Requirements**

Gazebo 7.0
Turtlebot-sim


**How to compile**
```
git clone ...
```
```
cd robot_wall_follower
```
```
rm -r devel/ build/
```
```
rm src/CMakeLists.txt
```
```
catkin_make
```

**How to execute**
```
source devel/setup.bash
```
Map "D": robot wanders at random until find wall, then follows wall forever (inside wall)
```
roslaunch robo_world spawn_world_D_inside.launch
```
Map "D": robot wanders at random until find wall, then follows wall forever (outside wall)
```
roslaunch robo_world spawn_world_D_outside.launch
```
For extra merit, create randomly a robot in between a small D and a large D arena
```
roslaunch robo_world spawn_world_DD_between.launch
```
