# flight_space_mapping
This package is built to construct flight space map which is named Flight Space Mapping (FSM). Custom wind data plugin and extracted obstacle feature is provided and Octomap is used for construct 3D map.

Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
 1. Download from github and install by using flight_space_mapping.rosinstall 
 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 $ wget https://raw.githubusercontent.com/mkpark2017/flight_space_mapping/master/flight_space_mapping.rosinstall
 $ wstool merge flight_space_mapping.rosinstall
 $ wstool update
 ```
 
 2. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 3. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Note. Normally Kinetic-devel requires extra packages.
```
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model with velodyne lidar and maaping nodes with Rviz displayer.

```
$ roslaunch flight_space_mapping Flight_space_mapping.launch
```

I implement segmap_ros example using gazebo simulatior. Yoe can install "segmap_ros" through
```
$ https://github.com/ethz-asl/segmap
```

Run gazebo and segmap as follows
```
$ roslaunch flight_space_mapping double_UAV_UGV_env.launch
$ roslaunch flight_space_mapping double_UAV_UGV_segmapping.launch
```

You can control each UAV/UGV using following commands respectivly
```
$ rostopic pub /robot0/command/pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "world"}, pose: {position: {x: 5, y: 5, z: 25}, orientation: {w: 1.0}}}'
$ rostopic pub /robot1/command/pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "world"}, pose: {position: {x: 0, y: -20, z: 25}, orientation: {w: 1.0}}}'
$ rostopic pub -r 50 /robot2/cmd_vel geometry_msgs/Twist -- '[10.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
$ rostopic pub -r 50 /robot3/cmd_vel geometry_msgs/Twist -- '[10.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```



