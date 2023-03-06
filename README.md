# Simplified Tinker 2020

** 屎山预警：很那啥的缝合怪版cartographer+rtabmap **  

Devices: rplidar3 + realsense D455  

*Note that*
1. `lib-realsense` will have to be installed separatedly
2. Highly recommend to create a separate workspace for cartographer

### Performance and Problems:
1. 能建图能规划
2. rtab VSLAM 在cpu占用率较低情况下仍会逐渐变慢，丢帧严重
3. 后来实际把realsense点云拍平到了z=0平面，但没有额外滤波！！加上抖动过于严重整个地面都是障碍物根本不能用
