

rosbag play --clock --hz=300 -q --rate=0.1 ~/Videos/OutTests/KistaGardPark_2021-03-18-16-39-14__th-0.22-lat59.406820-long17.940523.bag --topics /imu_right/imu/data /imu_left/imu/data /GPSfix /odom /tf /tf_static /cmd_vel

rosbag play --clock --hz=300 -q --rate=0.33 --start=62 ~/Videos/2021-04-01-16-09-01.bag --topics /imu_right/imu/data /imu_left/imu/data /GPSfix /odom /tf /tf_static /cmd_vel



rosbag play --clock --hz=300 -q --rate=0.33 --start=210 /media/marco/writable/home/ubuntu/Bags/2021-04-21-13-01-06.bag --topics /imu_right/imu/data_raw /imu_left/imu/data_raw /GPSfix /odom /tf /tf_static /cmd_vel

