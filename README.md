# autonomous-vehicle-MDS


## Commands

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  pangyo_control get_imu 
``

### GPS

``rosrun  pangyo_control gps_data_pangyo.py 
``


## ROSBAG save

``rosbag record -a ``


## ROSBAG play

``rosbag play -($rosbag name).bag ``


## Protocol

![img](./docs/Protocol_set.png)
