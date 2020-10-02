# ROS_files

rqt_mouse_teleop - plugin for ROS RQT, used for controlling robot with mouse.
This plugin is publishing Twist commands. Plugin have acceleration option.
It's still in development.
Maximum speeds, acceleration values and topic name is set is source code (Python).
![Alt text](rqt_mouse_teleop/image_rqt2.png?raw=true "Image")

"rqt_battery_state" - plugin for ROS RQT, used for displaying robot battery state. It's subscribing to universal "BatteryState" messages.

"init_pos_publisher" - simple publisher of "initialpose"[PoseWithCovarianceStamped].  
I write this node for "fake_localization" - it need sending "initialpose" for normal working.


