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

#### Sidenote

Robot trajectories (paths) while drawing a house won't be straight lines and it is normal and expected. 
However, from the figure obtained, you should see red and blue line, more or less in a same way. 

## TODO: 
- [x] Create `control_arm.cpp` interface [simple moveit wrapper]
- [x] Add instructions for building/using Dockerfile 
- [x] Create `or_lab1.py`
- [x] Test `or_lab1.py` 
- [ ] Edit prototyped methods 
