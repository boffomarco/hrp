# Set environment
source ~/.bashrc
source ~/rise/devel/setup.bash
clear

# Record data on RPi4
rosbag record /rtabmap/odom /rtabmap/odom_info /GPSfix /battery_status /cmd_vel /current_status /gps_left/NMEA_fix /gps_left/NMEA_hv /gps_right/NMEA_fix /gps_right/NMEA_hv /header /imu_euler /loop /imu_left/imu_manager /imu_left/imu/data_raw /imu_left/imu/mag /imu_right/imu_manager /imu_right/imu/data_raw /imu_right/imu/mag /odom /motor_feedback_diff_drive /sensor_status /rosout /rosout_agg /tf /tf_static /wheel_current /wheel_encoder /wheel_power
rosbag record -a

# Launch Automower with IMUs on RPi4
roslaunch am_driver_safe  automower_hrp.launch

# Launch GPS on RPi3
roslaunch am_sensors gps.launch

# Launch driver on PC
rosrun am_driver hrp_teleop.py

# Launch Camera
roslaunch am_sensors rs_camera.launch
# Launch RTABMAP
roslaunch am_sensors rtabmap.launch     rtabmap_args:="--delete_db_on_start"     depth_topic:=/camera/aligned_depth_to_color/image_raw     rgb_topic:=/camera/color/image_raw     camera_info_topic:=/camera/color/camera_info     approx_sync:=false

# Launch AEKF5
rosrun am_driver_safe AEKF5_last.py


# Mapping Testing Experiments
rosrun am_driver sim_hrp_teleop.py
rosrun am_driver_safe AEKF5_SimulatedSensors.py
rosrun am_driver_safe GroundTruthSimulated.py
rosrun am_driver_safe CollisionOccupancyMap.py
roslaunch am_sensors simulateSensors.launch

# Connect to RPi devices
ssh ubuntu@$mynet.221   # RPi 3 - user: ubuntu - psw: ubuntu
ssh ubuntu@$mynet.91    # RPi 4 - user: ubuntu - psw: raspberry



# Play bags
rosbag play --clock --hz=300 -q --rate=0.1 ~/Videos/OutTests/KistaGardPark_2021-03-18-16-39-14__th-0.22-lat59.406820-long17.940523.bag --topics /imu_right/imu/data /imu_left/imu/data /GPSfix /odom /tf /tf_static /cmd_vel

rosbag play --clock --hz=300 -q --rate=0.33 --start=62 ~/Videos/2021-04-01-16-09-01.bag --topics /imu_right/imu/data /imu_left/imu/data /GPSfix /odom /tf /tf_static /cmd_vel



rosbag play --clock --hz=300 -q --rate=0.33 --start=210 /media/marco/writable/home/ubuntu/Bags/2021-04-30-15-47-56.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix


rosbag play --clock --hz=300 -q --rate=0.5 --start=140 /media/marco/writable/home/ubuntu/Bags/2021-04-30-15-31-23.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix

rosbag play --clock --hz=300 -q --rate=0.5 --start=119 /media/marco/writable/home/ubuntu/Bags/2021-04-21-12-42-20.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix

rosbag play --clock --hz=300 -q --rate=0.5 --start=140 /media/marco/writable/home/ubuntu/Bags/2021-04-30-15-38-12.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix


rosbag play --clock --hz=300 -q --rate=0.5 /media/marco/writable/home/ubuntu/Bags/2021-05-13-14-12-59.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom

rosbag play --clock --hz=300 -q --start=161 /media/marco/writable/home/ubuntu/Bags/2021-05-13-14-12-59.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom

rosbag play --clock --hz=300 -q --rate=0.25 ~/Videos/CompleteOutTest/2021-05-13-14-12-59.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom

rosbag play --clock --hz=300 -q --rate=0.08 ~/Videos/CompleteOutTest/2021-05-13-14-12-59.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom

rosbag play --clock --hz=300 -q --start=161 ~/Videos/CompleteOutTest/2021-05-13-14-12-59.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom

rosbag play --clock --hz=300 -q ~/Videos/2021-05-13-14-12-59.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom

rosbag play --clock --hz=300 -q ~/Videos/GT.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom /Odom_Ground

rosbag play --clock --hz=300 -q ~/Videos/GT_Final.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /current_status /gps_left/NMEA_fix /gps_right/NMEA_fix /rtabmap/odom /Odom_Ground
