# Mid-360 Driver

This is an implementation of the Mid-360 driver, intended to serve as a replacement for [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).

It has the following features:

- It does not rely on Livox-SDK2 but directly implements UDP communication, making it highly lightweight.

- It supports automatically obtaining the LiDAR IP, eliminating the need to configure the LiDAR IP.

- It supports multiple LiDARs.

- It also supports the Mid-70, see [Mid-70 driver](#mid-70-driver).

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

## Mid-70 driver

The Mid-70 is supported by a second node (`mid70_driver_node`) inside this package, so the Mid-360
path stays untouched. Unlike the Mid-360, the Mid-70 speaks a stateful private UDP protocol
(broadcast discovery, handshake, sampling enable, and a heartbeat that must stay >= 1Hz), so this
node runs its own discovery/handshake/reconnect state machine. Topics, point cloud fields and the
configuration style are the same as the Mid-360.

```bash
ros2 launch mid360_driver mid70_driver.launch.py   # driver only, params in config/mid70_param.yaml
ros2 topic hz /livox/lidar
```

| | Mid-360 | Mid-70 |
| --- | --- | --- |
| Node / launch | `mid360_driver_node` / `launch/mid360_driver.launch.py` | `mid70_driver_node` / `launch/mid70_driver.launch.py` |
| Config file | `config/param.yaml` | `config/mid70_param.yaml` |
| Topics | `lidar_topic` (PointCloud2) + `imu_topic` (Imu) | `lidar_topic` (PointCloud2, no IMU on Mid-70) |
| Point cloud | `x, y, z, intensity, timestamp` | identical |

here are the Mid-70 parameters you can set in config file:

```yaml
mid70_driver:
    ros__parameters:
        lidar_topic: /livox/lidar
        lidar_frame: livox_frame
        lidar_publish_time_interval: 0.1
        host_ip: 192.168.1.50 # empty = detect the local address by route to the LiDAR
        lidar_ip: "" # empty = listen for the LiDAR broadcast
        device_port: 65000 # LiDAR command and point cloud port
        discovery_port: 55000 # port the LiDAR broadcasts to
        cmd_port: 55001 # local port for command replies
        data_port: 56001 # local port for point clouds
        imu_port: 56002 # Mid-70 has no IMU, kept for protocol completeness
        return_mode: "" # empty = keep device setting / first / strongest / dual
        coordinate_mode: "" # empty = keep device setting / cartesian / spherical
        discovery_timeout: 10.0 # discovery wait per attempt [s]
        reconnect_timeout: 3.0 # re-handshake after point clouds stop for this long [s]
        drop_invalid_points: true # drop (0,0,0) no-return points and points beyond 1000 m
```

Notes:

- Only the first echo is published and points with a noise tag are dropped, exactly like the Mid-360
driver; use `return_mode` if you want the device to work in single echo (`first`/`strongest`) or
`dual` mode.
- The Mid-70 samples 100k points/s, so consecutive points in a packet are 10us apart.
- Point clouds that stop for `reconnect_timeout` trigger a new handshake (the LiDAR drops the session
if the heartbeat stops, so reconnecting is the normal recovery path).
- Several Mid-70s: start one node per LiDAR (different `lidar_topic`, `host_ip`/`data_port`).

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
