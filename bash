

rosbag play --clock --hz=300 -q --rate=0.1 ~/Videos/OutTests/KistaGardPark_2021-03-18-16-39-14__th-0.22-lat59.406820-long17.940523.bag --topics /imu_right/imu/data /imu_left/imu/data /GPSfix /odom /tf /tf_static /cmd_vel

rosbag play --clock --hz=300 -q --rate=0.33 --start=62 ~/Videos/2021-04-01-16-09-01.bag --topics /imu_right/imu/data /imu_left/imu/data /GPSfix /odom /tf /tf_static /cmd_vel



rosbag play --clock --hz=300 -q --rate=0.33 --start=210 /media/marco/writable/home/ubuntu/Bags/2021-04-30-15-47-56.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix


rosbag play --clock --hz=300 -q --rate=0.5 --start=140 /media/marco/writable/home/ubuntu/Bags/2021-04-30-15-31-23.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix

rosbag play --clock --hz=300 -q --rate=0.5 --start=119 /media/marco/writable/home/ubuntu/Bags/2021-04-21-12-42-20.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix

rosbag play --clock --hz=300 -q --rate=0.5 --start=140 /media/marco/writable/home/ubuntu/Bags/2021-04-30-15-38-12.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel /gps_left/fix /gps_right/fix

