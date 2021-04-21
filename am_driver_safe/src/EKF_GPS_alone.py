#!/usr/bin/env python3

# From https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py

import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
from sensor_msgs.msg import NavSatFix
from am_driver.msg import WheelEncoder
from nav_msgs.msg import Odometry


import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import pymap3d as pm


class EKF_Euler_GPS():

    def __init__(self):
        print("Initialising EKF_Euler_GPS")

        rospy.init_node("EKF_Euler_GPS", anonymous=True)

        self.pose_enc_pub = rospy.Publisher('Pose_Enc', Pose, queue_size=20)

        self.pose_gps_pub = rospy.Publisher('Pose_GPS', Pose, queue_size=20)

        self.pose_pred_pub = rospy.Publisher('Pred_EKF', Pose, queue_size=20)

        self.pose_ekf_pub = rospy.Publisher('EKF_Euler_GPS', PoseWithCovarianceStamped, queue_size=20)
        self.odom_ekf_pub = rospy.Publisher('odom_EKF_Euler_GPS', Odometry, queue_size=20)

        rospy.Subscriber('wheel_encoder', WheelEncoder, self.Enc2PoseEuler)

        rospy.Subscriber('GPSfix', NavSatFix, self.NavSat2PoseGPS)

        # Kalman final values
        self.x_t = 0.0
        self.y_t = 0.0
        self.th_t = 0.3 # Manually set to follow the GPS

        # Encoder values
        self.x_enc = 0.0
        self.y_enc = 0.0
        self.th_enc = 0.3 # Manually set to follow the GPS

        self.leftPulses = 0
        self.lastLeftPulses = 0

        self.rightPulses = 0
        self.lastRightPulses = 0

        # Automower parameters
        self.base_width = 0.464500 # Original measurement
        self.base_width = 0.435 # Measured the internal side with stick -> 0.435
        self.wheel_diameter = 0.245
        #self.wheel_diameter = 0.24 # Measured the inner side with stick -> 0.238
        self.wheel_pulses_per_turn = 349
        self.wheel_meter_per_tick = (2.0 * math.pi * self.wheel_diameter / 2.0) / self.wheel_pulses_per_turn

        # GPS values
        self.new_measure = False
        self.z_x = 0.0
        self.z_y = 0.0
        self.z_th = 0.0

        # Manually set mean of satellite position
        self.lat_mean = 59.406827 #
        self.long_mean = 17.940565 #


        # State Vector [x y yaw]'
        self.xEst = np.zeros((3, 1))
        self.xTrue = np.zeros((3, 1))
        self.PEst = np.eye(3)


        # Covariance for EKF simulation
        self.Q = np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            np.deg2rad(5)  # variance of yaw angle (Not really considered, as we are not measuring the yaw)
        ]) ** 2  # predict state covariance

        self.z_cov = 12.25 # HDOP = 5, prec = 1.5 -> (((HDOP*prec)^2)/(2*(prec)^2)
        self.z_cov = self.z_cov / 10 # Trying to lower the cov
        self.z_th_cov = np.deg2rad(10)
        self.R = np.diag([self.z_cov, self.z_cov, self.z_th_cov])  # Observation x,y position covariance



    def NavSat2PoseGPS(self, GPSfix):
        gps_e , gps_n, gps_u = pm.geodetic2enu(GPSfix.latitude,GPSfix.longitude,0,self.lat_mean,self.long_mean,0)

        delta_z_th = math.atan2((- gps_e - self.z_y),( gps_n - self.z_x))
        #print(delta_z_th)
        self.z_th = delta_z_th

        self.z_x = gps_n
        self.z_y = - gps_e

        self.z_cov = GPSfix.position_covariance[0] # Original value of covariance from Automower
        self.z_cov = self.z_cov / 4.5 # Scale value of HDOP (Averaging among n&e covariances and removing 1.5*1.5 scale)
        self.z_cov = self.z_cov / 10 # Trying to lower the cov

        self.R = np.diag([self.z_cov, self.z_cov, self.z_th_cov])  # Observation x,y,th position covariance
        #self.R = np.diag([self.z_cov, self.z_cov])  # Observation x,y position covariance

        # Save that a new measure has been made
        self.new_measure = True

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.z_th)

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(self.z_x, self.z_y, 0.), Quaternion(*odom_quat))

        # publish the message
        self.pose_gps_pub.publish(pose)



    def jacob_f(self, th_t, delta_d):
        jF = np.array([
            [1.0, 0.0, - delta_d * math.sin(th_t)],
            [0.0, 1.0, delta_d * math.cos(th_t)],
            [0.0, 0.0, 1.0]])

        return jF


    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0]
            ,[0, 0, 1]
        ])

        return jH


if __name__ == '__main__':

    try:

        ekf_euler_gps = EKF_Euler_GPS()

        rospy.spin()
        #odomCheck.run()

    except rospy.ROSInterruptException:
        pass
