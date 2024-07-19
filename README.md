# lidro_irl100

This is a `ROS2` driver for `IRL-100`.

`IRL-100`  
: 4-channel laser-scan sensor

Currently, this driver is
- beta release.
- tested on `humble`.
_(However, it is anticipated that it will be able to run other distributions without modification.)_
- supportive for both `laserscan` and `pointcloud2`

## Quick build and run

### How to build
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### How to run the example
```
ros2 launch lidro_filter lidro_filter.launch.py
```

### How to run rviz2
```
source /opt/ros/humble/setup.bash
rviz2 -d rviz/default.rviz
```

## How to configure

refer to: `config/lidro.yaml`
```
lidro_irl100_node:
    ros__parameters:
        device: /dev/ttyUSB0
        baudrate: 1500000
        time_offset: 0.0
        enabled: true
        repeat_delay: 0.0
        frame_id: lidar_link
        timestamp_first_packet: false
        range_min: 0.0 # unit: mm
        range_max: 1000.0 # unit: mm
        # angle_min: -3.14
        # angle_max: 3.14
        output_laserscan: true #default false
        # output_pcl2: false #default true
        # flip_x_axis: false #default true
        # detect_ground: false #default true
```

## Default topics
### publish
- pointcloud2 : `out`
- laserscan: `scan_out_0`, `scan_out_1`, `scan_out_2`, `scan_out_3`

## Specification
- Horizontal resolution: 0.4 (deg) = 900 points / channel
- Vertical resolution: 9 (deg). range: 0.0 ~ 27.0
