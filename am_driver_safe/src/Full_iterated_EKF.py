#!/usr/bin/env python3

# From https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py

import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
from sensor_msgs.msg import NavSatFix, Imu
from am_driver.msg import WheelEncoder
from nav_msgs.msg import Odometry

import math

import threading

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

import numpy as np

import pymap3d as pm

# import the random module
import random


class Full_EKF():

    def __init__(self):

        # Define name of the Node
        rospy.init_node("Full_EKF", anonymous=True)

        # Define the self.lock to allow multi-threading
        self.lock = threading.Lock()

        # Get the current time
        now = rospy.get_time()

        # Define set of topics to subscribe to

        rospy.Subscriber('cmd_vel', Twist, self.Control)
        self.control_measure = False
        self.control_t = now
        self.control_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('wheel_encoder', WheelEncoder, self.Encoder)
        self.encoder_measure = False
        self.encoder_t = now
        self.encoder_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('imu_left/imu/data', Imu, self.ImuLeft)
        self.imu_left_measure = False
        self.imu_left_t = now
        self.imu_left_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('imu_right/imu/data', Imu, self.ImuRight)
        self.imu_right_measure = False
        self.imu_right_t = now
        self.imu_right_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('GPSfix', NavSatFix, self.GPS)
        self.gps_measure = False
        self.gps_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        #rospy.Subscriber('VisualOdometry', Odometry, self.VisualOdometry)
        self.visual_odometry_measure = False
        self.visual_odometry_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Define set of topics to publish

        self.odom_control_pub = rospy.Publisher('Odom_Control', Odometry, queue_size=20)

        self.odom_encoder_pub = rospy.Publisher('Odom_Encoder', Odometry, queue_size=20)

        #self.odom_imu_l_pub = rospy.Publisher('Odom_IMU_L', Odometry, queue_size=20)

        #self.odom_imu_r_pub = rospy.Publisher('Odom_IMU_R', Odometry, queue_size=20)

        self.odom_gps_pub = rospy.Publisher('Odom_GPS', Odometry, queue_size=20)

        #self.odom_pred_ekf_pub = rospy.Publisher('Odom_Pred_EKF', Odometry, queue_size=20)

        self.odom_full_ekf_pub = rospy.Publisher('Odom_Full_EKF', Odometry, queue_size=20)


        # Kalman states
        self.x_t = 0.0
        self.y_t = 0.0
        self.yaw_t = 0.0
        self.x_dot_t = 0.0
        self.yaw_dot_t = 0.0
        self.x_dot2_t = 0.0

        # State-Vector
        self.X_t = np.array([self.x_t,      self.y_t,      self.yaw_t,
                        self.x_dot_t,  self.yaw_dot_t, self.x_dot2_t])
        # Filter Covariance Matrix
        self.P_t = np.eye(6)


        # Initialise Measurements Vector
        self.Z = np.array([])
        # Initialise Measurements Covariance Matrix
        self.R = np.array([])
        # Initialise Measurements Matrix
        self.H = np.zeros((6,0))
        # Initialise Measurements Jacobian Matrix
        self.J_H = np.zeros((6,0))

        print("Initialised Full_EKF")



    # Prediction step with only the kinematic model
    def Predict(self, dt):

        # Make sure the execution is safe
        self.lock.acquire()
        try:

            # State-Transition Matrix
            A = np.array([  [1.0, 0.0, 0.0, cos(self.X_t[2])*dt, 0.0, cos(self.X_t[2])*(dt**2)/2],
                            [0.0, 1.0, 0.0, sin(self.X_t[2])*dt, 0.0, sin(self.X_t[2])*(dt**2)/2],
                            [0.0, 0.0, 1.0, 0.0,  dt, 0.0],
                            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

            # Noise Variance
            sigma_noise = 0.001

            # Noise Matrix
            W = np.array([  random.gauss(mu = 0, sigma = sigma_noise),
                            random.gauss(mu = 0, sigma = sigma_noise),
                            random.gauss(mu = 0, sigma = sigma_noise/10),
                            random.gauss(mu = 0, sigma = sigma_noise/10),
                            random.gauss(mu = 0, sigma = sigma_noise/100),
                            random.gauss(mu = 0, sigma = sigma_noise/100)])

            # Jacobian of Transition Matrix
            J_A = np.array([[1.0, 0.0, 0.0, -sin(self.X_t[2])*dt, 0.0, -sin(self.X_t[2])*(dt**2)/2],
                            [0.0, 1.0, 0.0, cos(self.X_t[2])*dt, 0.0, cos(self.X_t[2])*(dt**2)/2],
                            [0.0, 0.0, 1.0, 0.0,  dt, 0.0],
                            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

            # Prediction Covariance
            Q = np.array([  [sigma_noise, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, sigma_noise, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, sigma_noise, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, sigma_noise/10, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, sigma_noise/10, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, sigma_noise/100]])


            # Prediction State
            self.X_Pred = A @ self.X_t + W

            # Prediction Covariance Matrix
            self.P_t = J_A @ self.P_t @ J_A.T + Q # ??? + A@Q@A.T ???
            self.P_t = (self.P_t + self.P_t.T) / 2 # Ensure that it is symmetric

        finally:
            self.lock.release() # release self.lock, no matter what


    # Update step with the measurements
    def Update(self):

        # Make sure the execution is safe
        self.lock.acquire()
        try:

            # Add Control measures directly at each update
            self.Z = np.append(self.Z, np.array([self.control_state[0], self.control_state[1]]))
            self.R = np.append(self.R, np.array([0.01,0.001])) # Tuned

            self.H = np.column_stack([self.H, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,1,0])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,1,0])])


            # Transpose matrices after their creation
            Update_H = self.H.T
            Update_J_H = self.J_H.T
            Update_R = np.diag(self.R)
            # Store measurements vector for update
            Update_Z = self.Z


            # Initialise Measurements Vector
            self.Z = np.array([])
            # Initialise Measurements Covariance Matrix
            self.R = np.array([])
            # Initialise Measurements Matrix
            self.H = np.zeros((6,0))
            # Initialise Measurements Jacobian Matrix
            self.J_H = np.zeros((6,0))

            # Predicted using measurements matrix
            Z_pred = Update_H @ self.X_t
            # Innovation
            Y = Update_Z - Z_pred
            # Innovation Covariance
            S = Update_J_H @ self.P_t @ Update_J_H.T + Update_R
            # Kalman Gain
            K = self.P_t @ Update_J_H.T @ np.linalg.pinv(S) # Pseudo-Inverse of S to avoid Singularity
            # State Update
            self.X_t = self.X_t + K @ Y
            # Covariance Update
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t @ (np.eye(6) - K @ Update_J_H).T + K @ Update_R @ K.T
            # Ensure P is symmetric
            self.P_t = (self.P_t + self.P_t.T) / 2

        finally:
            self.lock.release() # release self.lock, no matter what





        # Send the Update to Ros
        header = Header()
        header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        header.frame_id = "odom"

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_t[2])

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(self.X_t[0], self.X_t[1], 0.), Quaternion(*odom_quat))

        pose_covariance     = [0] * 36
        pose_covariance[0]  = self.P_t[0][0]
        pose_covariance[1]  = self.P_t[0][1]
        pose_covariance[5]  = self.P_t[0][2]
        pose_covariance[6]  = self.P_t[1][0]
        pose_covariance[7]  = self.P_t[1][1]
        pose_covariance[11] = self.P_t[1][2]
        pose_covariance[30] = self.P_t[2][0]
        pose_covariance[31] = self.P_t[2][1]
        pose_covariance[35] = self.P_t[2][2]

        pose_ekf = PoseWithCovariance(pose, pose_covariance)

        # next, we'll publish the pose message over ROS
        twist = Twist(Vector3(self.X_t[3], 0, self.X_t[5]),Vector3(0.0, 0.0, self.X_t[4]))

        twist_covariance     = [0] * 36
        twist_covariance[0]  = self.P_t[3][3]
        twist_covariance[5]  = self.P_t[3][4]
        twist_covariance[30]  = self.P_t[4][3]
        twist_covariance[35]  = self.P_t[4][4]

        twist_ekf = TwistWithCovariance(twist, twist_covariance)

        odom_ekf = Odometry(header, "base_link", pose_ekf, twist_ekf)

        # publish the message
        self.odom_full_ekf_pub.publish(odom_ekf)


    def Control(self, cmd_vel):

        now = rospy.get_time()

        dt = now - self.control_t

        self.control_t = now

        z_x_dot = cmd_vel.linear.x
        z_yaw_dot = cmd_vel.angular.z


        z_x_dot_cov = 0.001
        z_yaw_dot_cov = 0.01

        """
        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_x_dot, z_yaw_dot]))
            self.R = np.append(self.R, np.array([z_x_dot_cov,z_yaw_dot_cov]))

            self.H = np.column_stack([self.H, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,1,0])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,1,0])])

            self.control_measure = True
        finally:
            self.lock.release() # release self.lock, no matter what
        """

        self.control_state = np.array([z_x_dot, z_yaw_dot])
        #print("Control " + str(self.control_state))

        # Send the Update to Ros
        header = Header()
        header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        header.frame_id = "odom"

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_t[2])

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(self.X_t[0], self.X_t[1], 0.), Quaternion(*odom_quat))

        pose_covariance = [0] * 36

        pose_control = PoseWithCovariance(pose, pose_covariance)

        twist_covariance = [0] * 36
        twist_covariance[0] = z_x_dot_cov
        twist_covariance[35] = z_yaw_dot_cov

        twist_control = TwistWithCovariance(cmd_vel, twist_covariance)

        odom_control = Odometry(header, "base_link", pose_control, twist_control)

        # publish the message
        self.odom_control_pub.publish(odom_control)


    def Encoder(self, wheel_encoder):

        # Automower parameters
        base_width = 0.464500 # Original measurement
        base_width = 0.435 # Measured the internal side with stick -> 0.435
        wheel_diameter = 0.245
        #self.wheel_diameter = 0.24 # Measured the inner side with stick -> 0.238
        wheel_pulses_per_turn = 349
        wheel_meter_per_tick = (2.0 * math.pi * wheel_diameter / 2.0) / wheel_pulses_per_turn


        lastLeftPulses = self.encoder_state[4]
        lastRightPulses = self.encoder_state[5]

        leftPulses = wheel_encoder.rwheelAccum
        rightPulses = wheel_encoder.lwheelAccum

        deltaLeftPulses = leftPulses - lastLeftPulses
        deltaRightPulses = rightPulses - lastRightPulses

        if(lastLeftPulses and lastRightPulses): #and deltaLeftPulses and deltaRightPulses):

            leftDist  = - deltaLeftPulses * wheel_meter_per_tick
            rightDist = deltaRightPulses * wheel_meter_per_tick

            delta_d = ( rightDist + leftDist ) / 2

            delta_yaw = ( rightDist - leftDist ) / base_width

            delta_x = delta_d * math.cos( self.X_t[2] )
            delta_y = delta_d * math.sin( self.X_t[2] )


            z_x   = self.X_t[0] - delta_x
            z_y   = self.X_t[1] - delta_y
            z_yaw = self.X_t[2] + delta_yaw

            z_yaw = math.atan2(math.sin(z_yaw), math.cos(z_yaw))
            #print("dir " + str(z_yaw))

            z_yaw = ((z_yaw + math.pi) % (2*math.pi)) - math.pi

            #print(str(z_yaw) + "  - prev " + str(self.X_t[2]))

            delta_z_yaw = (((self.X_t[2] - z_yaw) + math.pi) % (2*math.pi)) - math.pi

            z_yaw = self.X_t[2] - delta_z_yaw

            z_cov = 1
            z_yaw_cov = np.deg2rad(1)


            # Make sure the execution is safe
            self.lock.acquire()
            try:
                self.Z = np.append(self.Z, np.array([z_x, z_y, z_yaw]))
                self.R = np.append(self.R, np.array([z_cov,z_cov,z_yaw_cov]))

                self.H = np.column_stack([self.H, np.array([1,0,0,0,0,0]), np.array([0,1,0,0,0,0]), np.array([0,0,1,0,0,0])])
                self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0,0]), np.array([0,1,0,0,0,0]), np.array([0,0,1,0,0,0])])

                # Transpose matrices after their creation
                Update_H = self.H.T
                Update_J_H = self.J_H.T
                Update_R = np.diag(self.R)
                # Store measurements vector for update
                Update_Z = self.Z


                # Initialise Measurements Vector
                self.Z = np.array([])
                # Initialise Measurements Covariance Matrix
                self.R = np.array([])
                # Initialise Measurements Matrix
                self.H = np.zeros((6,0))
                # Initialise Measurements Jacobian Matrix
                self.J_H = np.zeros((6,0))

                # Predicted using measurements matrix
                Z_pred = Update_H @ self.X_t
                # Innovation
                Y = Update_Z - Z_pred
                # Innovation Covariance
                S = Update_J_H @ self.P_t @ Update_J_H.T + Update_R
                # Kalman Gain
                K = self.P_t @ Update_J_H.T @ np.linalg.pinv(S) # Pseudo-Inverse of S to avoid Singularity
                # State Update
                self.X_t = self.X_t + K @ Y
                # Covariance Update
                self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t
                # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite
                self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t @ (np.eye(6) - K @ Update_J_H).T + K @ Update_R @ K.T
                # Ensure P is symmetric
                self.P_t = (self.P_t + self.P_t.T) / 2
            finally:
                self.lock.release() # release self.lock, no matter what


            self.encoder_state = np.array([z_x, z_y, z_yaw, 0.0, lastLeftPulses, lastRightPulses])
            print("Encoder " + str(self.encoder_state[0:3]))


            # Send the Update to Ros
            header = Header()
            header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            header.frame_id = "odom"

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, z_yaw)

            # next, we'll publish the pose message over ROS
            pose = Pose(Point(z_x, z_y, 0.), Quaternion(*odom_quat))

            pose_covariance     = [0] * 36
            pose_covariance[0]  = z_cov
            pose_covariance[7]  = z_cov
            pose_covariance[35] = z_yaw_cov

            pose_encoder = PoseWithCovariance(pose, pose_covariance)

            odom_encoder = Odometry(header, "base_link", pose_encoder, TwistWithCovariance())

            # publish the message
            self.odom_encoder_pub.publish(odom_encoder)


        # Store prev values
        self.encoder_state[4] = leftPulses
        self.encoder_state[5] = rightPulses


    def GPS(self, GPSfix):

        lat_mean = 59.406820  # Manually set based on test
        long_mean = 17.940523 # Manually set based on test

        gps_e , gps_n, gps_u = pm.geodetic2enu(GPSfix.latitude,GPSfix.longitude,0,lat_mean,long_mean,0)

        z_yaw_direct = math.atan2((- gps_e - self.gps_state[1]),( gps_n - self.gps_state[0]))
        delta_z_yaw = math.atan2(math.sin(z_yaw_direct), math.cos(z_yaw_direct))
        #print("dir " + str(z_yaw_direct))

        z_yaw_direct = ((z_yaw_direct + math.pi) % (2*math.pi)) - math.pi

        #print(str(z_yaw_direct) + "  - prev " + str(self.gps_state[2]))

        delta_z_yaw = (((self.X_t[2] - z_yaw_direct) + math.pi) % (2*math.pi)) - math.pi

        #print("del " + str(delta_z_yaw))

        z_yaw = self.X_t[2] - delta_z_yaw
        z_x = gps_n
        z_y = - gps_e

        z_cov = GPSfix.position_covariance[0] # Original value of covariance from Automower
        z_cov = z_cov / 4.5 # Scale value of HDOP (Averaging among n&e covariances and removing 1.5*1.5 scale)
        z_cov = z_cov / 10 # Trying to lower the cov, by a lot
        z_yaw_cov = np.deg2rad(1)

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_x, z_y, z_yaw]))
            self.R = np.append(self.R, np.array([z_cov,z_cov,z_yaw_cov]))

            self.H = np.column_stack([self.H, np.array([1,0,0,0,0,0]), np.array([0,1,0,0,0,0]), np.array([0,0,1,0,0,0])])
            self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0,0]), np.array([0,1,0,0,0,0]), np.array([0,0,1,0,0,0])])

            # Transpose matrices after their creation
            Update_H = self.H.T
            Update_J_H = self.J_H.T
            Update_R = np.diag(self.R)
            # Store measurements vector for update
            Update_Z = self.Z

            # Initialise Measurements Vector
            self.Z = np.array([])
            # Initialise Measurements Covariance Matrix
            self.R = np.array([])
            # Initialise Measurements Matrix
            self.H = np.zeros((6,0))
            # Initialise Measurements Jacobian Matrix
            self.J_H = np.zeros((6,0))

            # Predicted using measurements matrix
            Z_pred = Update_H @ self.X_t
            # Innovation
            Y = Update_Z - Z_pred
            # Innovation Covariance
            S = Update_J_H @ self.P_t @ Update_J_H.T + Update_R
            # Kalman Gain
            K = self.P_t @ Update_J_H.T @ np.linalg.pinv(S) # Pseudo-Inverse of S to avoid Singularity
            # State Update
            self.X_t = self.X_t + K @ Y
            # Covariance Update
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t @ (np.eye(6) - K @ Update_J_H).T + K @ Update_R @ K.T
            # Ensure P is symmetric
            self.P_t = (self.P_t + self.P_t.T) / 2
        finally:
            self.lock.release() # release self.lock, no matter what

        self.gps_state = np.array([z_x,z_y,z_yaw])
        print(" GPS " + str(self.gps_state))

        # Send the Update to Ros
        header = Header()
        header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        header.frame_id = "odom"

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, z_yaw)

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(z_x,z_y, 0.), Quaternion(*odom_quat))

        pose_covariance     = [0] * 36
        pose_covariance[0]  = z_cov
        pose_covariance[7]  = z_cov
        pose_covariance[35] = z_yaw_cov

        pose_gps = PoseWithCovariance(pose, pose_covariance)

        odom_gps = Odometry(header, "base_link", pose_gps, TwistWithCovariance())

        # publish the message
        self.odom_gps_pub.publish(odom_gps)


    def ImuLeft(self, imu):

        now = rospy.get_time()

        dt = now - self.imu_left_t

        self.imu_left_t = now

        z_yaw_dot = - imu.angular_velocity.z
        z_x_dot2 = imu.linear_acceleration.x


        z_yaw_dot_cov = 1
        z_x_dot2_cov = 2


        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_yaw_dot, z_x_dot2]))
            self.R = np.append(self.R, np.array([z_yaw_dot_cov, z_x_dot2_cov,]))

            self.H = np.column_stack([self.H, np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])

            # Transpose matrices after their creation
            Update_H = self.H.T
            Update_J_H = self.J_H.T
            Update_R = np.diag(self.R)
            # Store measurements vector for update
            Update_Z = self.Z


            # Initialise Measurements Vector
            self.Z = np.array([])
            # Initialise Measurements Covariance Matrix
            self.R = np.array([])
            # Initialise Measurements Matrix
            self.H = np.zeros((6,0))
            # Initialise Measurements Jacobian Matrix
            self.J_H = np.zeros((6,0))

            # Predicted using measurements matrix
            Z_pred = Update_H @ self.X_t
            # Innovation
            Y = Update_Z - Z_pred
            # Innovation Covariance
            S = Update_J_H @ self.P_t @ Update_J_H.T + Update_R
            # Kalman Gain
            K = self.P_t @ Update_J_H.T @ np.linalg.pinv(S) # Pseudo-Inverse of S to avoid Singularity
            # State Update
            self.X_t = self.X_t + K @ Y
            # Covariance Update
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t @ (np.eye(6) - K @ Update_J_H).T + K @ Update_R @ K.T
            # Ensure P is symmetric
            self.P_t = (self.P_t + self.P_t.T) / 2
        finally:
            self.lock.release() # release self.lock, no matter what


        self.imu_left_state = np.array([z_yaw_dot, z_x_dot2])
        print("Imu Left " + str(self.imu_left_state))


    def ImuRight(self, imu):

        now = rospy.get_time()

        dt = now - self.imu_right_t

        self.imu_right_t = now

        z_yaw_dot = - imu.angular_velocity.z
        z_x_dot2 = imu.linear_acceleration.x


        z_yaw_dot_cov = 1
        z_x_dot2_cov = 2


        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_yaw_dot, z_x_dot2]))
            self.R = np.append(self.R, np.array([z_yaw_dot_cov, z_x_dot2_cov,]))

            self.H = np.column_stack([self.H, np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])

            # Transpose matrices after their creation
            Update_H = self.H.T
            Update_J_H = self.J_H.T
            Update_R = np.diag(self.R)
            # Store measurements vector for update
            Update_Z = self.Z


            # Initialise Measurements Vector
            self.Z = np.array([])
            # Initialise Measurements Covariance Matrix
            self.R = np.array([])
            # Initialise Measurements Matrix
            self.H = np.zeros((6,0))
            # Initialise Measurements Jacobian Matrix
            self.J_H = np.zeros((6,0))

            # Predicted using measurements matrix
            Z_pred = Update_H @ self.X_t
            # Innovation
            Y = Update_Z - Z_pred
            # Innovation Covariance
            S = Update_J_H @ self.P_t @ Update_J_H.T + Update_R
            # Kalman Gain
            K = self.P_t @ Update_J_H.T @ np.linalg.pinv(S) # Pseudo-Inverse of S to avoid Singularity
            # State Update
            self.X_t = self.X_t + K @ Y
            # Covariance Update
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite
            self.P_t = (np.eye(6) - K @ Update_J_H) @ self.P_t @ (np.eye(6) - K @ Update_J_H).T + K @ Update_R @ K.T
            # Ensure P is symmetric
            self.P_t = (self.P_t + self.P_t.T) / 2
        finally:
            self.lock.release() # release self.lock, no matter what


        self.imu_right_state = np.array([z_yaw_dot, z_x_dot2])
        print("Imu Right " + str(self.imu_right_state))



def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0]).flatten()
    py = np.array(fx[1, :] + xEst[1]).flatten()
    plt.plot(px, py, "--r")





if __name__ == '__main__':

    print("Start Full_EKF")

    full_ekf = None

    try:
        # Initialise the Kalman Filter
        full_ekf = Full_EKF()
        # Wait for the updates (Variable dt)
        #rospy.spin()
        # Continuosly try to get updates and run Prediction and Update (Constant dt)
        hertz = 250
        rate = rospy.Rate(hertz) # 250hz - highest frequency of the sensors (IMU)

        # Get the time
        start = rospy.get_time()

        # Wait for the system to start running
        while start == 0:
            start = rospy.get_time()
            # Sleep before next iteration
            rate.sleep()

        # State Vector [x y theta v omega a]'
        X = np.zeros((2, 1))

        Z = np.zeros((2, 1))

        # history
        hX = X
        hZ = Z

        # Start with the fusion
        end = start
        while not rospy.is_shutdown():
            # Update dt at each iteration
            start = rospy.get_time()
            dt = start - end
            print("KALMAN - " + str(start) + "  "  +  str(dt))
            # Prediction step
            full_ekf.Predict(dt)
            # Update step
            full_ekf.Update()
            print(str(full_ekf.X_t[0:3]))
            # Reset time
            end = start

            # store data history
            hX = np.hstack((hX, np.array([full_ekf.X_t[0],full_ekf.X_t[1]]).reshape(2,1)))
            hZ = np.hstack((hZ, np.array([full_ekf.gps_state[0],full_ekf.gps_state[1]]).reshape(2,1)))

            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hX[0, :].flatten(),
                     hX[1, :].flatten(), "-b")
            plt.plot(hZ[0, :],
                     hZ[1, :], ".g")
            plot_covariance_ellipse(full_ekf.X_t, full_ekf.P_t)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(1/hertz)

            # Sleep before next iteration
            #rate.sleep()

    except rospy.ROSInterruptException:

        pass

    print(full_ekf.X_t)
    print(full_ekf.P_t)
    print("End Full_EKF")
