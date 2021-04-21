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

        # Check if the filter is ready to start
        self.filter = False

        # Get the current time
        now = rospy.get_time()

        # Define set of topics to subscribe to

        rospy.Subscriber('cmd_vel', Twist, self.Control)
        self.control_measure = False
        self.control_t = -1
        self.control_state = np.array([0.0, 0.0])

        rospy.Subscriber('odom', Odometry, self.Odometer)
        self.odometer_measure = False
        self.odometer_t = now
        self.odometer_state = np.array([0.0, 0.0])
        self.odometer_bias = np.array([0.0, 0.0])
        self.odometer_var = np.array([0.0, 0.0])

        rospy.Subscriber('GPSfix', NavSatFix, self.GPS)
        self.gps_measure = False
        self.gps_state = np.array([0.0, 0.0, 0.0])
        self.gps_bias = np.array([0.0, 0.0, 0.0])
        self.gps_var = np.array([0.0, 0.0, 0.0])

        rospy.Subscriber('imu_left/imu/data/enu', Imu, self.ImuLeft)
        self.imu_left_measure = False
        self.imu_left_t = now
        self.imu_left_state = np.array([0.0, 0.0, 0.0])
        self.imu_left_bias = np.array([0.0, 0.0, 0.0])
        self.imu_left_var = np.array([np.deg2rad(10), 1.0, 2.0])

        rospy.Subscriber('imu_right/imu/data/enu', Imu, self.ImuRight)
        self.imu_right_measure = False
        self.imu_right_t = now
        self.imu_right_state = np.array([0.0, 0.0, 0.0])
        self.imu_right_bias = np.array([0.0, 0.0, 0.0])
        self.imu_right_var = np.array([np.deg2rad(10), 1.0, 2.0])


        #rospy.Subscriber('VisualOdometry', Odometry, self.VisualOdometry)
        #visual_odometry_measure = False
        #visual_odometry_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #visual_odometry_bias = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Define set of topics to publish

        self.odom_control_pub = rospy.Publisher('Odom_Control', Odometry, queue_size=20)

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
        self.P_t = np.eye(6)*1e-5
        # Filter Innovation Matrix
        self.K = np.diag(np.zeros(6))


        # Initialise Measurements Vector
        self.Z = np.array([])
        # Initialise Measurements Covariance Matrix
        self.R = np.array([])
        # Initialise Measurements Matrix
        self.H = np.zeros((6,0))
        # Initialise Measurements Jacobian Matrix
        self.J_H = np.zeros((6,0))

        #print("Initialised Full_EKF")


    # Prediction step with only the kinematic model
    def Predict(self, dt):

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
                        random.gauss(mu = 0, sigma = sigma_noise),
                        random.gauss(mu = 0, sigma = sigma_noise),
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
                        [0.0, 0.0, 0.0, sigma_noise, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, sigma_noise, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, sigma_noise/100]])

        # Check control difference
        u = np.array([  0.0,
                        0.0,
                        0.0,
                        self.control_state[0] - self.X_t[3],
                        self.control_state[1] - self.X_t[4],
                        0.0])

        steps = 10
        B = np.diag(np.array([0,0,0,1/steps,1/steps,0]))

        # Make sure the execution is safe
        self.lock.acquire()
        try:

            # Prediction State
            self.X_Pred = A @ self.X_t + B @ u + W

            # Prediction Covariance Matrix
            self.P_Pred = J_A @ self.P_t @ J_A.T + Q # ??? + A@Q@A.T ???
            self.P_Pred = (self.P_Pred + self.P_Pred.T) / 2 # Ensure that it is symmetric

        finally:
            self.lock.release() # release self.lock, no matter what


    # Prediction step without measurement updates
    def UpdateNoMeasures(self):

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.X_t = self.X_Pred
            self.P_t = self.P_Pred
        finally:
            self.lock.release() # release self.lock, no matter what

        #print("UpdateNoMeasures \t" + str(self.X_t[0:3]))


    # Update step with the measurements
    def Update(self):
        # Check if there are more updates
        if(self.odometer_measure or self.imu_left_measure or self.imu_right_measure or self.gps_measure):
        #if(True): # Always update, since control will be there

            # Make sure the execution is safe
            self.lock.acquire()
            try:
                # Reset Measurements check
                self.odometer_measure = self.encoder_measure = self.imu_left_measure = self.imu_right_measure = self.gps_measure = False


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

            finally:
                self.lock.release() # release self.lock, no matter what


            # Predicted using measurements matrix
            Z_pred = Update_H @ self.X_Pred
            # Innovation
            Y = Update_Z - Z_pred
            # Innovation Covariance
            S = Update_J_H @ self.P_Pred @ Update_J_H.T + Update_R
            # Kalman Gain
            self.K = self.P_Pred @ Update_J_H.T @ np.linalg.inv(S)
            # State Update
            self.X_t = self.X_Pred + self.K @ Y
            # Conventional Covariance Update
            self.P_t = (np.eye(6) - self.K @ Update_J_H) @ self.P_Pred
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite (More time consuming) - Low improvements
            #self.P_t = (np.eye(6) - self.K @ Update_J_H) @ self.P_Pred @ (np.eye(6) - self.K @ Update_J_H).T + self.K @ Update_R @ self.K.T
            # Ensure P is symmetric
            self.P_t = (self.P_t + self.P_t.T) / 2

            #print("Update \t\t\t" + str(self.X_t[0:3]))
        else:
            # Keep just the prediction if no new measurements have been received
            self.UpdateNoMeasures()

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

        self.control_t = 1

        z_x_dot = cmd_vel.linear.x
        z_yaw_dot = cmd_vel.angular.z

        z_x_dot_cov = 0.001
        z_yaw_dot_cov = 0.01

        self.control_state = np.array([z_x_dot, z_yaw_dot])
        #print("         Control \t\t" + str(self.control_state))

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


    def Wait(self):
        self.odometer_history = list()
        self.gps_history = list()
        self.imu_left_history = list()
        self.imu_right_history = list()


        while(self.control_t == -1):
            """if(self.odometer_measure):
                # Make sure the execution is safe
                self.lock.acquire()
                try:
                    self.odometer_history.append(self.odometer_state)
                    self.odometer_measure = False
                finally:
                    self.lock.release() # release self.lock, no matter what
            """

            if(self.gps_measure):
                # Make sure the execution is safe
                self.lock.acquire()
                try:
                    self.gps_history.append(self.gps_state)
                    self.gps_measure = False
                finally:
                    self.lock.release() # release self.lock, no matter what

            if(self.imu_left_measure):
                # Make sure the execution is safe
                self.lock.acquire()
                try:
                    self.imu_left_history.append(self.imu_left_state)
                    self.imu_left_measure = False
                finally:
                    self.lock.release() # release self.lock, no matter what

            if(self.imu_right_measure):
                # Make sure the execution is safe
                self.lock.acquire()
                try:
                    self.imu_right_history.append(self.imu_right_state)
                    self.imu_right_measure = False
                finally:
                    self.lock.release() # release self.lock, no matter what

        self.gps_history = np.vstack(self.gps_history)
        self.gps_bias = np.mean(self.gps_history, axis=0)

        self.imu_left_history = np.vstack(self.imu_left_history)
        #self.imu_left_bias = np.mean(self.imu_left_history, axis=0)
        #self.imu_left_var = np.std(self.imu_left_history, axis=0)

        self.imu_right_history = np.vstack(self.imu_right_history)
        #self.imu_right_bias = np.mean(self.imu_right_history, axis=0)
        #self.imu_right_var = np.std(self.imu_right_history, axis=0)

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            # State-Vector
            self.X_Pred = np.array([0, 0, (self.imu_right_state[0] + self.imu_left_state[0])/2, 0, 0, 0])
            # Filter Covariance Matrix
            #self.P_Pred = np.eye(6)

            # State-Vector
            self.X_t = np.array([0, 0, (self.imu_right_state[0] + self.imu_left_state[0])/2, 0, 0, 0])
            # Filter Covariance Matrix
            #self.P_t = np.eye(6)

            # Initialise Measurements Vector
            self.Z = np.array([])
            # Initialise Measurements Covariance Matrix
            self.R = np.array([])
            # Initialise Measurements Matrix
            self.H = np.zeros((6,0))
            # Initialise Measurements Jacobian Matrix
            self.J_H = np.zeros((6,0))
            self.odometer_measure = False
            self.gps_measure = False
            self.gps_state = np.array([0.0, 0.0, 0.0])
            self.imu_left_measure = False
            self.imu_right_measure = False

            self.filter = True

        finally:
            self.lock.release() # release self.lock, no matter what
        return


    def Odometer(self, Odometry):

        z_x_dot = Odometry.twist.twist.linear.x
        z_yaw_dot = Odometry.twist.twist.angular.z

        z_x_dot_cov = 0.1
        z_yaw_dot_cov = 0.5

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_x_dot, z_yaw_dot]))
            self.R = np.append(self.R, np.array([z_x_dot_cov,z_yaw_dot_cov]))

            self.H = np.column_stack([self.H, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,1,0])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,1,0])])

            self.odometer_measure = True

            self.odometer_state = np.array([z_x_dot,z_yaw_dot])
        finally:
            self.lock.release() # release self.lock, no matter what

        #print("     Odometer \t\t\t" + str(self.odometer_state))



    def GPS(self, GPSfix):

        if (not self.filter):
            # Make sure the execution is safe
            self.lock.acquire()
            try:
                self.gps_measure = True

                self.gps_state = np.array([GPSfix.latitude,GPSfix.longitude,0])
            finally:
                self.lock.release() # release self.lock, no matter what
            #print("     GPS \t\t" + str(self.gps_state))
        else:
            #lat_mean = 59.406820  # Manually set based on test
            #long_mean = 17.940523 # Manually set based on test

            gps_e , gps_n, gps_u = pm.geodetic2enu(GPSfix.latitude,GPSfix.longitude,0,self.gps_bias[0],self.gps_bias[1],0)

            z_yaw_direct = math.atan2((gps_n - self.gps_state[1]),( gps_e - self.gps_state[0]))
            #delta_z_yaw = math.atan2(math.sin(z_yaw_direct), math.cos(z_yaw_direct))
            #print("dir " + str(z_yaw_direct))

            z_yaw_direct = ((z_yaw_direct + math.pi) % (2*math.pi)) - math.pi

            #print(str(z_yaw_direct) + "  - prev " + str(self.gps_state[2]))

            delta_z_yaw = ((( z_yaw_direct - self.X_t[2]) + math.pi) % (2*math.pi)) - math.pi

            #print("del " + str(delta_z_yaw))

            z_yaw = self.X_t[2] + delta_z_yaw
            z_x = gps_e
            z_y = gps_n

            z_cov = GPSfix.position_covariance[0] # Original value of covariance from Automower
            z_cov = z_cov / 4.5 # Scale value of HDOP (Averaging among n&e covariances and removing 1.5*1.5 scale)
            #z_cov = z_cov / 10 # Trying to lower the cov
            z_yaw_cov = np.deg2rad(22.5)

            """
            # Make sure the execution is safe
            self.lock.acquire()
            try:
                self.Z = np.append(self.Z, np.array([z_x, z_y, z_yaw]))
                self.R = np.append(self.R, np.array([z_cov,z_cov,z_yaw_cov]))

                self.H = np.column_stack([self.H, np.array([1,0,0,0,0,0]), np.array([0,1,0,0,0,0]), np.array([0,0,1,0,0,0])])
                self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0,0]), np.array([0,1,0,0,0,0]), np.array([0,0,1,0,0,0])])

                self.gps_measure = True

                self.gps_state = np.array([z_x,z_y,z_yaw])
            finally:
                self.lock.release() # release self.lock, no matter what
            """

            #print("     GPS \t\t" + str(self.gps_state))

            # Send the Update to ROS
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

        euler = tf.transformations.euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        z_yaw_direct = euler[2]
        z_yaw_direct = ((z_yaw_direct + math.pi) % (2*math.pi)) - math.pi
        delta_z_yaw = ((( z_yaw_direct - self.X_t[2]) + math.pi) % (2*math.pi)) - math.pi

        z_yaw = self.X_t[2] + delta_z_yaw
        z_yaw_dot = - imu.angular_velocity.z - self.imu_left_bias[1]
        z_x_dot2 = 0#imu.linear_acceleration.y - self.imu_left_bias[2]

        z_yaw_cov = self.imu_left_var[0]**2 #np.deg2rad(5)
        z_yaw_dot_cov = self.imu_left_var[1]**2 #1
        z_x_dot2_cov = self.imu_left_var[2]**2 #4


        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_yaw, z_yaw_dot, z_x_dot2]))
            self.R = np.append(self.R, np.array([z_yaw_cov, z_yaw_dot_cov, z_x_dot2_cov,]))

            self.H = np.column_stack([self.H, np.array([0,0,1,0,0,0]), np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,1,0,0,0]), np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])

            self.imu_left_measure = True

            self.imu_left_state = np.array([z_yaw, z_yaw_dot, z_x_dot2])
        finally:
            self.lock.release() # release self.lock, no matter what


        #print("     Imu Left \t\t\t\t" + str(self.imu_left_state))


    def ImuRight(self, imu):

        now = rospy.get_time()

        dt = now - self.imu_right_t

        self.imu_right_t = now

        euler = tf.transformations.euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        z_yaw_direct = euler[2]
        z_yaw_direct = ((z_yaw_direct + math.pi) % (2*math.pi)) - math.pi
        delta_z_yaw = ((( z_yaw_direct - self.X_t[2]) + math.pi) % (2*math.pi)) - math.pi

        z_yaw = self.X_t[2] + delta_z_yaw
        z_yaw_dot = - imu.angular_velocity.z - self.imu_right_bias[1]
        z_x_dot2 = 0#imu.linear_acceleration.y - self.imu_right_bias[2]

        z_yaw_cov = self.imu_right_var[0]**2 #np.deg2rad(5)
        z_yaw_dot_cov = self.imu_right_var[1]**2 #1
        z_x_dot2_cov = self.imu_right_var[2]**2 #4


        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.Z = np.append(self.Z, np.array([z_yaw, z_yaw_dot, z_x_dot2]))
            self.R = np.append(self.R, np.array([z_yaw_cov, z_yaw_dot_cov, z_x_dot2_cov,]))

            self.H = np.column_stack([self.H, np.array([0,0,1,0,0,0]), np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,1,0,0,0]), np.array([0,0,0,0,1,0]), np.array([0,0,0,0,0,1])])

            self.imu_right_measure = True

            self.imu_right_state = np.array([z_yaw, z_yaw_dot, z_x_dot2])
        finally:
            self.lock.release() # release self.lock, no matter what

        #print("     Imu Right \t\t\t\t" + str(self.imu_right_state))



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
    return px, py


def plotFinalCovarianceP(P, m):

    fig = plt.figure(figsize=(6, 6))

    #print(P)

    im = plt.imshow(P, interpolation="none", cmap=plt.get_cmap('binary'))

    plt.title('Covariance Matrix $P$ (after %i Filter Steps)' % (m))

    ylocs, ylabels = plt.yticks()
    # set the locations of the yticks
    plt.yticks(np.arange(6))
    # set the locations and labels of the yticks
    plt.yticks(np.arange(6),('$x$', '$y$', '$\\theta$', '$v$', '$\dot \\theta$', '$a$'), fontsize=22)

    xlocs, xlabels = plt.xticks()
    # set the locations of the yticks
    plt.xticks(np.arange(6))
    # set the locations and labels of the yticks
    plt.xticks(np.arange(6),('$x$', '$y$', '$\\theta$', '$v$', '$\dot \\theta$', '$a$'), fontsize=22)

    """
    plt.xlim([-0.5,4.5])
    plt.ylim([4.5, -0.5])
    """

    from mpl_toolkits.axes_grid1 import make_axes_locatable
    divider = make_axes_locatable(plt.gca())
    cax = divider.append_axes("right", "5%", pad="3%")
    plt.colorbar(im, cax=cax)


    plt.tight_layout()

    plt.pause(1)


def plotHistory(H, m, title):

    fig, (ax0, ax1, ax2, ax3, ax4, ax5) = plt.subplots(6, sharex=True)
    fig.suptitle(title + ' (after %i Filter Steps)' % (m))
    p0 = ax0.plot(range(m), H[0], label='$x$')
    p1 = ax1.plot(range(m), H[1], label='$y$')
    p2 = ax2.plot(range(m), H[2], label='$\\theta$')
    p3 = ax3.plot(range(m), H[3], label='$v$')
    p4 = ax4.plot(range(m), H[4], label='$\dot \\theta$')
    p5 = ax5.plot(range(m), H[5], label='$a$')

    ax0.legend(shadow=True, fancybox=True)
    ax1.legend(shadow=True, fancybox=True)
    ax2.legend(shadow=True, fancybox=True)
    ax3.legend(shadow=True, fancybox=True)
    ax4.legend(shadow=True, fancybox=True)
    ax5.legend(shadow=True, fancybox=True)

    plt.xlabel('Filter Steps')
    plt.ylabel('')
    plt.legend(loc='best')


    plt.pause(1)



if __name__ == '__main__':

    #print("Start Full_EKF")

    full_ekf = None

    try:
        # Initialise the Kalman Filter
        full_ekf = Full_EKF()
        # Wait for the updates (Variable dt)
        #rospy.spin()
        # Continuosly try to get updates and run Prediction and Update (Constant dt)
        hertz = 250 # highest frequency of the sensors (IMU)
        rate = rospy.Rate(hertz)

        # Get the time
        start = rospy.get_time()

        #print("Wait for the system to start")
        # Wait for the system to start running
        while start == 0:
            start = rospy.get_time()
            # Sleep before next iteration
            rate.sleep()

        #print("Wait for the first command before starting")
        full_ekf.Wait()

        # State Vector [x y theta v omega a]'
        X = np.zeros((6, 1))
        # GPS Measures [x y theta]
        Z = np.zeros((3, 1))
        # Innovation K
        K = np.zeros((6, 1))
        # Covariance P_t
        P = np.zeros((6, 1))

        # history
        hDt = list()
        hX = X
        hZ = Z
        hK = K
        hP = P
        hPx = list()
        hPy = list()


        #print("Start fusion")
        # Start with the fusion
        end = rospy.get_time()
        while not rospy.is_shutdown():
            # Update dt at each iteration
            start = rospy.get_time()
            dt = start - end
            hDt.append(dt)

            #print("KALMAN - " + str(start) + "  "  +  str(dt))
            # Prediction step
            full_ekf.Predict(dt)
            # Update step
            full_ekf.Update()

            # store data history
            hX = np.hstack((hX, full_ekf.X_t.reshape(6,1)))
            hZ = np.hstack((hZ, np.array([full_ekf.gps_state[0],full_ekf.gps_state[1],full_ekf.gps_state[2]]).reshape(3,1)))
            hK = np.hstack((hK, np.array([full_ekf.K[0,0], full_ekf.K[1,0], full_ekf.K[2,0], full_ekf.K[3,0], full_ekf.K[4,0], full_ekf.K[5,0]]).reshape(6,1)))
            hP = np.hstack((hP, np.array([full_ekf.P_t[0,0], full_ekf.P_t[1,1], full_ekf.P_t[2,2], full_ekf.P_t[3,3], full_ekf.P_t[4,4], full_ekf.P_t[5,5]]).reshape(6,1)))

            """ Plot in real time (time consuming)
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(hX[0, :].flatten(),
                    hX[1, :].flatten(), "-b")
            plt.plot(hZ[0, :],
                    hZ[1, :], ".g")
            px, py = plot_covariance_ellipse(full_ekf.X_t, full_ekf.P_t)
            plt.plot(px, py, "--r")
            hPx.append(px)
            hPy.append(py)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(1/hertz)
            """

            # Sleep before next iteration
            rate.sleep()

            # Reset time
            end = start

    except rospy.ROSInterruptException:

        pass


    #for i in range(len(hPx)):
    #    plt.plot(hPx[i], hPy[i], "--r")

    #print(full_ekf.X_t)
    #print(full_ekf.P_t)
    #print("End Full_EKF")

    plotFinalCovarianceP(full_ekf.P_t, hP.shape[1])
    plotHistory(hK, hK.shape[1], 'Innovation Gain $K$')
    plotHistory(hP, hP.shape[1], 'Covariance Matrix $P$')

    plt.figure(10)
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(hX[0, :].flatten(),
            hX[1, :].flatten(), "-b")
    plt.plot(hZ[0, :],
            hZ[1, :], ".g")


    plt.figure(5)
    bins =  np.arange(0,0.1,0.001)
    plt.hist(hDt,bins = bins)
    plt.pause(1)


    # Wait for the user input to terminate the program
    input("Press any key to terminate the program\n")
    # Print bye message
    #print("THE END")
