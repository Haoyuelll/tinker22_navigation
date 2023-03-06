# Tinker 2022 Navigation

Navigation: `roslaunch tinker_navigation nav.launch`

SLAM: `roslaunch tinker_navigation slam.launch`

*Note that* 
1. slam.launch will be called in nav.launch. 
2. The recording of ros bag and map file is not included in slam.launch
3. Add map file with the option `-load_static_map`

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

## ira_laser_tools
External repo for multi laser. Unused in 2022.