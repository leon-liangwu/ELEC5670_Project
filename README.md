## Dependency

- ROS
- V-REP
- Other tools: `ros-kinetic-hector-slam`, `opencv-python`, `dlib`

## Code Structure


```bash
cd ./robot_project
tree -L 2
.
├── launch
│   └── final_project.launch   # all in one launch file
├── rviz_cfg                   # rviz config file for demo,
│   └── demo.rviz              # auto loaded by final_project.launch
├── src
│   ├── image_localization     # node image_localization
│   ├── laser_mapping          # node laser_mappting
│   └── visual_servoing        # node visual_servoing
└── ttt
    └── env_modified.ttt       # env file used by V-REP
```

## Run
```bash
# terminal 1 run roscore
roscore
# terminal 2 start vrep simulation env
cd $VREP_dir 
./vrep.sh
# terminal 3 run all with one laucn file
cd ./robot_project
catkin_make
roslaunch launch/final_project.launch
```

The interface is shown as bellow:

<img src=".\figures\whole_dis.png" alt="Displaying" style="zoom:67%;" />

## Nodes

There are three ROS nodes in this project: *laser_mapping*, *image_localization* and *visual_servoing*. And these nodes are responsible for respective functions as follows: 

1. in node *laser_mapping*, we use arrow keys to move the robot and simultaneously build the 2D map through hector slam provided with laser scan data, 
2. in node *image_localization*, the robot detects and localize the images on the wall and mark them in the rviz, and also this node checks which area the robot enters in,
3. in node *visual_servoing*, it detects the yellow moving ball and if it occurs, it will switch from key controlling mode to auto tracking mode, and simultaneously the laser scan sensor turns off.
