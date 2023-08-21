# aruco_maker_pose_detector
ARUCO marker pose detector using depth image on ROS2

## launch realsense2_camera
```
$ ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640,480,30 depth_module.profile:=640,480,30 pointcloud.enable:=true decimation_filter.enable:=true align_depth.enable:=true
```


## launch node
```
$ ros2 run aruco_maker_pose_detector main_node
```