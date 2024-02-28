# ros2_segway_rmp


    ros2 launch ros2_segway_rmp ros2_segway_rmp_launch.xml
    
-------------------

    ros2 topic list -t
    
...........

    /cmd_vel [geometry_msgs/msg/Twist]
    /odom [nav_msgs/msg/Odometry]
    /parameter_events [rcl_interfaces/msg/ParameterEvent]
    /rosout [rcl_interfaces/msg/Log]
    /segway_status [segway_interfaces/msg/Stamped]

-------------------

    ros2 interface show segway_interfaces/msg/Stamped

...........

    std_msgs/Header header
    	builtin_interfaces/Time stamp
    		int32 sec
    		uint32 nanosec
    	string frame_id
    Segwaystatus    segway
    	int8    LIGHT=1
    	int8    TALL=2
    	int8    HEAVY=3
    	int8    BALANCE=1
    	int8    TRACTOR=2
    	int8    POWER_DOWN=3
    	float32 pitch_angle
    	float32 pitch_rate
    	float32 roll_angle
    	float32 roll_rate
    	float32 left_wheel_velocity
    	float32 right_wheel_velocity
    	float32 yaw_rate
    	float32 servo_frames
    	float32 left_wheel_displacement
    	float32 right_wheel_displacement
    	float32 forward_displacement
    	float32 yaw_displacement
    	float32 left_motor_torque
    	float32 right_motor_torque
    	int8    operation_mode
    	int8    gain_schedule
    	float32 ui_battery
    	float32 powerbase_battery
    	bool    motors_enable
 



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
