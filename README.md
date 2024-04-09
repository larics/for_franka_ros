# for_franka_ros 

Fundamentals of robotics ROS package for the Franka Emika robot. 

## Environment setup

Dockerfile for creating environment for the laboratory exercises, alongside with instructions how to use is located [here](https://github.com/larics/docker_files/tree/master/ros-noetic/for_course_lab/or2324). 

Instructions for installing Docker can be found [here](https://docs.docker.com/engine/install/ubuntu/). 

After installing Docker, check [post-installation](https://docs.docker.com/engine/install/linux-postinstall/) steps to easily build, start and remove docker images. 


## Package instructions

### Use 

1. You can launch robot with: 
```
roslaunch for_franka_ros for_lab.launch
```

2. Open up another terminal and enter docker container with: 
```
docker exec -it for_cont bash
```

3. Run script with: 

```
python3 or_lab1.py
``` 

#### Topics

Topics that are used to control robot are under `control_arm_node` namespace. 

### Real robot [3rd laboratory exercise] 

In order to work with real robot, we need to specify correct `ROS_MASTER_URI` and `ROS_HOSTNAME` 
setup. 

You can do so with following commands: 
```
echo "export ROS_MASTER_URI=http://192.168.150.250:11311" >> ~/.bashrc  
echo "export ROS_HOSTNAME=<your_ip>" >> ~/.bashrc    
```

After that you can run your or_lab3 script with: 

```
python3 or_lab3.py
```

#### Sidenote

Robot trajectories (paths) while drawing a house won't be straight lines and it is normal and expected. 
However, from the figure obtained, you should see red and blue line, more or less in a same way. 

## Build workspace with autocomplete options

In order to use autocomplete and CMAKE_ARGS use following command: 
```
catkin build --cmake-args -DCMAKE_C_FLAGS="-DCMAKE_EXPORT_COMPILE_COMMANDS=1"
```

## Useful resources

* [ROS planning docs](https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#sequence-of-multiple-segments)  
* [FreeCAD docs](https://www.picuino.com/en/freecad-diferencia.html) for making obstacles for the experiments   

## Robot states

Current implemented robot states are: 
 * `JOINT_TRAJ_CTL` --> OMPL MoveIt! integrated planner 
 * `CART_TRAJ_CTL` --> Cartesian MoveIt! integrated planner
 * `SERVO_CTL` --> Not implemented yet!
 * `IDLE` --> If command sent, robot ignores cmd!

You can change states by invoking: 

```
rosservice call /control_arm_node/change_state "state: "<WANTED_STATE>""
```

## TODO: 
- [x] Create `control_arm.cpp` interface [simple moveit wrapper]
- [x] Add instructions for building/using Dockerfile 
- [x] Debug init HoCook 
- [ ] [py] Add [class inheritance](https://realpython.com/python-super/) and develop one Lab class that will be inherited by each instance of the lab
- [x] [cpp] Refactor 
- [x] [cpp] Add yaml topic name reading 
- [x] [cpp] Added init to the Cartesian path planning [Fails often] 
- [ ] [cpp] Add servo
- [ ] [cpp] Add analytic IK [choose closest solution]
- [ ] [cpp] Test joint pose commands 
- [ ] [cpp] Change to the shorter namespace 
- [ ] [cpp] Test with multiple robots (IFACE)
- [x] [cpp] IFACE test: Schunk
- [x] [cpp] IFACE test: Franka
- [ ] [cpp] IFACE test: Kinova 
- [ ] [docs] Add CMake stuff
