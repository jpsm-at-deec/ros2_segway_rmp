# ros2_segway_rmp


    ros2 launch ros2_segway_rmp ros2_segway_rmp_launch.xml
-------------------

    ros2 topic list -t
-------------------

    /cmd_vel [geometry_msgs/msg/Twist]
    /odom [nav_msgs/msg/Odometry]
    /parameter_events [rcl_interfaces/msg/ParameterEvent]
    /rosout [rcl_interfaces/msg/Log]
    /segway_status [segway_interfaces/msg/Stamped]



# ov

    .
    ├── ...
    ├── src                    
    │   ├── libsegwayrmp [a]
    │   ├── ros2_segway_rmp [b]    
    │   ├── segway_interfaces [c]
    │   └── serial [d]
    └── ...

[a] https://github.com/jpsm-at-deec/libsegwayrmp

[b] https://github.com/jpsm-at-deec/ros2_segway_rmp/

[c] https://github.com/jpsm-at-deec/segway_interfaces

[d] https://github.com/jpsm-at-deec/serial
