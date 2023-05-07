# Tinker 2022 Navigation

##  Usage

- Follow the guide [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation) to start a workspace for **Cartographer**. 
  * Note that cartographer requires `catkin_make_isolated` and `--use-ninja` when building the workspace. It is recommended to place it in another workspace. You may consult *wbc* you prefer packing them all in one.
- Grant access to Hokuyo laser port: `sudo chmod 777 /dev/ttyACM0` 
- Navigation: `roslaunch tinker_navigation nav.launch`
- SLAM only: `roslaunch tinker_navigation slam.launch`


#### *Note that* 

1. `slam.launch` is already called in `nav.launch`. 
2. The recording of ros bag and map file is not included in `slam.launch`, you will have to start another terminal and use the following commands to save your map:
   - Use `rosrun map_server map_saver` or`rosservice call /finish_trajectory 0` to save the map and end the trajectory, and then
   - `rosservice call /write_state "filename: 'map.pbstream' include_unfinished_submaps: true"`  and replace the filename with your desired one. 
3. Add map file with the option `-load_static_map`

### * Please refer to `project_debug.md` for general problems faced when building the workspace



## tinker_navigation

Using cartographer for SLAM and teb_loacl_planner for planning. 

`./tinker_navigation/config/navigation`: .yaml file of costmap and other planning parameters  
`./tinker_navigation/config/slam`: .lua & .rviz file for laser scan / camera pointcloud  
`./tinker_navigation/lauch`: .launch file, acml.launch is abandoned  
`./tinker_navigation/src` & `./tinker_navigation/scripts`: codes for specific tasks  



## tinker_description

Paramters and model of Tinker  
`./tinker_description/meshes`: .stl model, note that this model only contains the skeleton of tinker  
`./tinker_description/urdf` & `./tinker_description/launch`: .urdf description and urdf publisher  



## urg_node

External repo for Hokuyo laser scan

Use the following commands when facing unmet dependencies:

`sudo apt install ros-${ROS_DISTRO}-laser-proc`
`sudo apt install ros-${ROS_DISTRO}-urg-c`



## ira_laser_tools

External repo for multi laser. Unused in 2022.



## 