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


Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model with velodyne lidar and maaping nodes with Rviz displayer.

```
$ roslaunch flight_space_mapping single_UAV_UGV_mapping.launch
```
