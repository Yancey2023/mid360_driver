# Mid-360 Driver

This is an implementation of the Mid-360 driver, intended to serve as a replacement for [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).

It has the following features:

- It does not rely on Livox-SDK2 but directly implements UDP communication, making it highly lightweight.

- It supports automatically obtaining the LiDAR IP, eliminating the need to configure the LiDAR IP.

- It supports multiple LiDARs.

Special note:

- It publishes point clouds in the PointCloud2 format, which differs from the point cloud format of livox_ros_driver2. Users may need to modify code in other packages.

<img src="./img/ACE.jpg" width="200px">

## Install dependencies

1. Please make sure you have install ROS2.
2. Install Asio. If you are using ubuntu, you can install by following command: `sudo apt install libasio-dev`

## Param

here are some parameters you can set in config file:

```yaml
mid360_driver:
    ros__parameters:
        lidar_topic: /livox/lidar
        lidar_frame: livox_frame
        imu_topic: /livox/imu
        imu_frame: imu_frame
        lidar_publish_time_interval: 0.1
        host_ip: 192.168.1.50
        is_topic_name_with_lidar_ip: false # 是否在话题名后面加雷达ip，可以用于区分多个雷达
```

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
