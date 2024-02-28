# ros2_segway_rmp


ros2 launch ros2_segway_rmp ros2_segway_rmp_launch.xml
-------------------

ros2 topic list
-------------------

/cmd_vel

/odom

/parameter_events

/rosout

/segway_status


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
