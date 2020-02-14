# autonomous-vehicle-MDS


## Commands

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  pangyo_control get_imu 
``

### Getting ERP42 information (encoder, steer, speed, brake, gear)

``rosrun  stauto_control ERP42_to_PC
``

### GPS

``rosrun  pangyo_control gps_data_pangyo.py 
``
### AMCL

``roslaunch stauto_sensor HyphaROS_MiniCar_Racing.launch ``

## ROSBAG save

``rosbag record -a ``


## ROSBAG play

``rosbag play -($rosbag name).bag --clock --topic /velodyne_points /imu/data ``

## LEGO-LOAM
``roslaunch lego_loam run.launch ``

실시간 확인시 `use_sim_time` 값 false로 변경

## Protocol

![img](./docs/Protocol_set.png)
