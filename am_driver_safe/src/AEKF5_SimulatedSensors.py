#!/usr/bin/env python3

# Using am_sensors/simulatedSensors

# [TODO]
# - Differentiate between std in static or moving behaviour

import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
from sensor_msgs.msg import NavSatFix, Imu
from am_driver.msg import WheelEncoder
from am_driver.msg import SensorStatus, CurrentStatus
from nav_msgs.msg import Odometry

import math

import threading

import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib as mpl

import seaborn as sns
from scipy.spatial.transform import Rotation as Rot

import numpy as np

import pymap3d as pm

# import the random module
import random


class AEKF_Sim():

    def __init__(self):

        # Define name of the Node
        rospy.init_node("AEKF5_Sim", anonymous=True)

        # Define the run type of the Filter
        self.test = True
        self.print = False
        self.ros = True

        # Define the self.lock to allow multi-threading
        self.lock = threading.Lock()

        # Check if the filter is ready to start
        self.filter = False

        # Get the current time
        now = rospy.get_time()

        # Kalman states
        self.x_t = 0.0
        self.y_t = 0.0
        self.yaw_t = 0.0
        self.x_dot_t = 0.0
        self.yaw_dot_t = 0.0

        # Frequency of the Kalman filter
        self.rate = 250
        # Steps to slowly account the control input
        self.steps = 250

        # State-Vector
        self.X_t = np.array([self.x_t,      self.y_t,      self.yaw_t,
                             self.x_dot_t,  self.yaw_dot_t])
        self.X_Pred = self.X_t
        self.X_control = np.array([self.x_t,      self.y_t,      self.yaw_t,
                                   self.x_dot_t,  self.yaw_dot_t])
        self.X_wheel_odom = np.array([self.x_t,      self.y_t,      self.yaw_t,
                                   self.x_dot_t,  self.yaw_dot_t])
        self.X_visual_odom = np.array([self.x_t,      self.y_t,      self.yaw_t,
                                   self.x_dot_t,  self.yaw_dot_t])
        # Filter Covariance Matrix
        self.P_t = np.eye(5)*1e-5
        self.P_Pred = self.P_t
        # Filter Innovation Matrix
        self.K = np.diag(np.zeros(5))


        # Initialise Measurements Vector
        self.Z = np.array([])
        # Initialise Measurements Covariance Matrix
        self.R = np.array([])
        # Initialise Measurements Matrix
        self.H = np.zeros((5,0))
        # Initialise Measurements Jacobian Matrix
        self.J_H = np.zeros((5,0))

        print("Initialised AEKF_Sim")


        # Define set of topics to subscribe to

        rospy.Subscriber('Odom_Ground', Odometry, self.GroundTruth)
        self.ground_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('cmd_vel', Twist, self.Control)
        self.control_fusion = True
        self.control_measure = False
        self.control_t = -1
        self.control_state = np.array([0.0, 0.0])
        rospy.Subscriber('current_status', CurrentStatus, self.CurrentStatus)
        self.current_status = 1

        rospy.Subscriber('/wheel_odometry/odom', Vector3, self.WheelOdometer)
        self.wheel_fusion = True
        self.wheel_odometer_measure = False
        self.wheel_odometer_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.wheel_odometer_bias = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.wheel_odometer_var = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('/visual_odometry/odom', Vector3, self.VisualOdometer)
        self.visual_fusion = True
        self.visual_odometer_measure = False
        self.visual_odometer_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.visual_odometer_bias = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.visual_odometer_var = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.gps_th = 0
        rospy.Subscriber('automower_gps/GPSfix', Vector3, self.GPS)
        self.gps_fusion = True
        self.gps_measure = False
        self.gps_state = np.array([0.0, 0.0, 0.0])
        self.gps_bias = np.array([0.0, 0.0, 0.0])
        self.gps_var = np.array([0.0, 0.0, 0.0])

        rospy.Subscriber('gps_left/NMEA_fix', Vector3, self.gps_left)
        self.gps_left_fusion = True
        self.gps_left_measure = False
        self.gps_left_hz = 5
        self.gps_left_state = np.array([0.0, 0.0, 0.0])
        self.gps_left_bias = np.array([0.0, 0.0, 0.0])
        self.gps_left_var = np.array([0.0, 0.0, 0.0])

        rospy.Subscriber('gps_right/NMEA_fix', Vector3, self.gps_right)
        self.gps_right_fusion = True
        self.gps_right_measure = False
        self.gps_right_hz = 5
        self.gps_right_state = np.array([0.0, 0.0, 0.0])
        self.gps_right_bias = np.array([0.0, 0.0, 0.0])
        self.gps_right_var = np.array([0.0, 0.0, 0.0])

        rospy.Subscriber('imu_left/data_raw', Vector3, self.ImuLeft)
        self.imu_left_fusion = True
        self.imu_left_measure = False
        self.imu_left_t = now
        self.imu_left_state = np.array([0.0, 0.0, 0.0])
        self.imu_left_bias = np.array([0.0, 0.0, 0.0])
        self.imu_left_var = np.array([np.deg2rad(10), 0.05, 2.5])

        rospy.Subscriber('imu_right/data_raw', Vector3, self.ImuRight)
        self.imu_right_fusion = True
        self.imu_right_measure = False
        self.imu_right_t = now
        self.imu_right_state = np.array([0.0, 0.0, 0.0])
        self.imu_right_bias = np.array([0.0, 0.0, 0.0])
        self.imu_right_var = np.array([np.deg2rad(10), 0.05, 2.5])

        # Define set of topics to publish
        if(self.test and self.ros):
            self.odom_control_pub = rospy.Publisher('Odom_Control', Odometry, queue_size=20)

            self.odom_wheel_pub = rospy.Publisher('Odom_Wheel_Sim', Odometry, queue_size=20)
            self.odom_visual_pub = rospy.Publisher('Odom_Visual_Sim', Odometry, queue_size=20)

            self.odom_aekf_sim_pub = rospy.Publisher('Odom_AEKF_Sim', Odometry, queue_size=20)



    # Prediction step with only the kinematic model
    def Predict(self, dt):

        dt =  0.004  # Fix it to avoid computation issues and keep a discrete time system

        # State-Transition Matrix
        A_t = np.array([[1.0, 0.0, 0.0, cos(self.X_t[2])*dt, 0.0],
                        [0.0, 1.0, 0.0, sin(self.X_t[2])*dt, 0.0],
                        [0.0, 0.0, 1.0, 0.0,  dt],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])
        A_control = np.array([[1.0, 0.0, 0.0, cos(self.X_control[2])*dt, 0.0],
                              [0.0, 1.0, 0.0, sin(self.X_control[2])*dt, 0.0],
                              [0.0, 0.0, 1.0, 0.0,  dt],
                              [0.0, 0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 1.0]])
        A_wheel_odom = np.array([[1.0, 0.0, 0.0, cos(self.X_wheel_odom[2])*dt, 0.0],
                                [0.0, 1.0, 0.0, sin(self.X_wheel_odom[2])*dt, 0.0],
                                [0.0, 0.0, 1.0, 0.0,  dt],
                                [0.0, 0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 1.0]])
        A_visual_odom = np.array(  [[1.0, 0.0, 0.0, cos(self.X_visual_odom[2])*dt, 0.0],
                                    [0.0, 1.0, 0.0, sin(self.X_visual_odom[2])*dt, 0.0],
                                    [0.0, 0.0, 1.0, 0.0,  dt],
                                    [0.0, 0.0, 0.0, 1.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 1.0]])


        # Jacobian of Transition Matrix
        J_A = np.array([[1.0, 0.0, 0.0, -sin(self.X_t[2])*dt, 0.0],
                        [0.0, 1.0, 0.0,  cos(self.X_t[2])*dt, 0.0],
                        [0.0, 0.0, 1.0, 0.0,  dt ],
                        [0.0, 0.0, 0.0, 1.0, 0.0 ],
                        [0.0, 0.0, 0.0, 0.0, 1.0 ]])

        # Prediction Covariance
        sigma = ( ( abs(self.imu_left_state[2]) + abs(self.imu_right_state[2]) ) / 2 ) ** 2
        # Q2 Complete
        Q = np.array([  [(sin(self.X_t[2])**2)*(dt**2)/2,                   -(sin(self.X_t[2])*cos(self.X_t[2]))*(dt**2)/2, 0.0,        -(sin(self.X_t[2]))*(dt**2),    0.0  ],
                        [-(sin(self.X_t[2])*cos(self.X_t[2]))*(dt**2)/2,    (cos(self.X_t[2])**2)*(dt**2)/2,                0.0,        (cos(self.X_t[2]))*(dt**2),     0.0  ],
                        [0.0,                                               0.0,                                            dt**2/2,    0.0,                            dt**2],
                        [-(sin(self.X_t[2]))*(dt**2),                       (cos(self.X_t[2]))*(dt**2),                     0.0,        dt,                             0.0  ],
                        [0.0,                                               0.0,                                            dt**2,      0.0,                            dt   ]]) * sigma

        # Check control difference
        u_t = np.array([0.0,
                        0.0,
                        0.0,
                        (self.control_state[0] - self.X_t[3]),
                        (self.control_state[1] - self.X_t[4])])
        u_control = np.array([0.0,
                             0.0,
                             0.0,
                             (self.control_state[0] - self.X_control[3]),
                             (self.control_state[1] - self.X_control[4])])
        u_wheel_odom = np.array([0.0,
                             0.0,
                             0.0,
                             (self.wheel_odometer_state[0] - self.X_wheel_odom[3]),
                             (self.wheel_odometer_state[1] - self.X_wheel_odom[4])])
        u_visual_odom = np.array([0.0,
                             0.0,
                             0.0,
                             (self.visual_odometer_state[0] - self.X_visual_odom[3]),
                             (self.visual_odometer_state[1] - self.X_visual_odom[4])])

        B = np.diag(np.array([0,0,0, dt, dt]))

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            # Control data
            self.X_control = A_control @ self.X_control + u_control
            # Wheel Odom data
            self.X_wheel_odom = A_wheel_odom @ self.X_wheel_odom + u_wheel_odom
            # Odom data
            self.X_visual_odom = A_visual_odom @ self.X_visual_odom + u_visual_odom

            # Prediction State
            if(self.control_fusion):
                self.X_Pred = A_t @ self.X_t + B @ u_t
                # Prediction Covariance Matrix
                self.P_Pred = J_A @ self.P_t @ J_A.T + Q
            else:
                self.X_Pred = A_t @ self.X_t
                # Prediction Covariance Matrix
                self.P_Pred = J_A @ self.P_t @ J_A.T + Q

        finally:
            self.lock.release() # release self.lock, no matter what

        if(self.test and self.ros):
            # Send the Update of the Control to ROS
            header = Header()
            header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            header.frame_id = "odom"

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_control[2])

            # next, we'll publish the pose message over ROS
            pose = Pose(Point(self.X_control[0], self.X_control[1], 0.), Quaternion(*odom_quat))

            pose_covariance     = [0] * 36

            pose_control = PoseWithCovariance(pose, pose_covariance)

            # next, we'll publish the pose message over ROS
            twist = Twist(Vector3(self.X_control[3], 0, 0),Vector3(0.0, 0.0, self.X_control[4]))

            twist_covariance     = [0] * 36

            twist_control = TwistWithCovariance(twist, twist_covariance)

            odom_control = Odometry(header, "base_link", pose_control, twist_control)

            # publish the message
            self.odom_control_pub.publish(odom_control)

        if(self.test and self.ros):
            # Send the Update of the Odometry to Ros
            header = Header()
            header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            header.frame_id = "odom"

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_wheel_odom[2])

            # next, we'll publish the pose message over ROS
            pose = Pose(Point(self.X_wheel_odom[0], self.X_wheel_odom[1], 0.), Quaternion(*odom_quat))

            pose_covariance     = [0] * 36

            pose_odom = PoseWithCovariance(pose, pose_covariance)

            # next, we'll publish the pose message over ROS
            twist = Twist(Vector3(self.X_wheel_odom[3], 0, 0),Vector3(0.0, 0.0, self.X_wheel_odom[4]))

            twist_covariance     = [0] * 36

            twist_odom = TwistWithCovariance(twist, twist_covariance)

            odom_odom = Odometry(header, "base_link", pose_odom, twist_odom)

            # publish the message
            self.odom_wheel_pub.publish(odom_odom)

        if(self.test and self.ros):
            # Send the Update of the Odometry to Ros
            header = Header()
            header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            header.frame_id = "odom"

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_visual_odom[2])

            # next, we'll publish the pose message over ROS
            pose = Pose(Point(self.X_visual_odom[0], self.X_visual_odom[1], 0.), Quaternion(*odom_quat))

            pose_covariance     = [0] * 36

            pose_odom = PoseWithCovariance(pose, pose_covariance)

            # next, we'll publish the pose message over ROS
            twist = Twist(Vector3(self.X_visual_odom[3], 0, 0),Vector3(0.0, 0.0, self.X_visual_odom[4]))

            twist_covariance     = [0] * 36

            twist_odom = TwistWithCovariance(twist, twist_covariance)

            odom_odom = Odometry(header, "base_link", pose_odom, twist_odom)

            # publish the message
            self.odom_visual_pub.publish(odom_odom)

        if(self.test or self.ros):
            # Send the Prediction to Ros
            header = Header()
            header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            header.frame_id = "odom"

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_Pred[2])

            # next, we'll publish the pose message over ROS
            pose = Pose(Point(self.X_Pred[0], self.X_Pred[1], 0.), Quaternion(*odom_quat))

            pose_covariance     = [0] * 36
            pose_covariance[0]  = self.P_Pred[0][0]
            pose_covariance[1]  = self.P_Pred[0][1]
            pose_covariance[5]  = self.P_Pred[0][2]
            pose_covariance[6]  = self.P_Pred[1][0]
            pose_covariance[7]  = self.P_Pred[1][1]
            pose_covariance[11] = self.P_Pred[1][2]
            pose_covariance[30] = self.P_Pred[2][0]
            pose_covariance[31] = self.P_Pred[2][1]
            pose_covariance[35] = self.P_Pred[2][2]

            pose_ekf = PoseWithCovariance(pose, pose_covariance)

            # next, we'll publish the pose message over ROS
            twist = Twist(Vector3(self.X_Pred[3], 0, 0),Vector3(0.0, 0.0, self.X_Pred[4]))

            twist_covariance     = [0] * 36
            twist_covariance[0]  = self.P_Pred[3][3]
            twist_covariance[5]  = self.P_Pred[3][4]
            twist_covariance[30]  = self.P_Pred[4][3]
            twist_covariance[35]  = self.P_Pred[4][4]

            twist_ekf = TwistWithCovariance(twist, twist_covariance)

            odom_ekf = Odometry(header, "base_link", pose_ekf, twist_ekf)

            # publish the message
            self.odom_aekf_sim_pub.publish(odom_ekf)


    # Prediction step without measurement updates
    def UpdateNoMeasures(self):

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.X_t = self.X_Pred
            self.P_t = self.P_Pred
        finally:
            self.lock.release() # release self.lock, no matter what

        if(self.test and self.print):
            print("UpdateNoMeasures \t" + str(self.X_t[0:3]))


    # Update step with the measurements
    def Update(self):
        # Check if there are more updates
        if(self.wheel_odometer_measure or self.visual_odometer_measure or self.imu_left_measure or self.imu_right_measure or self.gps_measure or self.gps_left_measure >= self.gps_left_hz or self.gps_right_measure >= self.gps_right_hz ):
            # Debug test
            text = "Update "

            # Make sure the execution is safe
            self.lock.acquire()
            try:
                if(self.wheel_odometer_measure):

                    self.Z = np.append(self.Z, np.array([self.wheel_odometer_state[0], self.wheel_odometer_state[1]]))
                    self.R = np.append(self.R, np.array([self.wheel_odometer_state[2], self.wheel_odometer_state[3]]))

                    self.H = np.column_stack([self.H, np.array([0,0,0,1,0]), np.array([0,0,0,0,1])])
                    self.J_H = np.column_stack([self.J_H, np.array([0,0,0,1,0]), np.array([0,0,0,0,1])])

                    text += " WOdom "
                else:
                    text += " \t "


                if(self.visual_odometer_measure):

                    self.Z = np.append(self.Z, np.array([self.visual_odometer_state[0], self.visual_odometer_state[1]]))
                    self.R = np.append(self.R, np.array([self.visual_odometer_state[2], self.visual_odometer_state[3]]))

                    self.H = np.column_stack([self.H, np.array([0,0,0,1,0]), np.array([0,0,0,0,1])])
                    self.J_H = np.column_stack([self.J_H, np.array([0,0,0,1,0]), np.array([0,0,0,0,1])])

                    text += " VOdom "
                else:
                    text += " \t "


                if(self.gps_measure):

                    self.Z = np.append(self.Z, np.array([self.gps_state[0], self.gps_state[1], self.gps_state[2]]))
                    self.R = np.append(self.R, np.array([self.gps_state[3], self.gps_state[4], self.gps_state[5]]))

                    self.H = np.column_stack([self.H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0]), np.array([0,0,1,0,0])])
                    self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0]), np.array([0,0,1,0,0])])

                    text += " GPS "
                else:
                    text += " \t "


                if(self.gps_left_measure):

                    self.Z = np.append(self.Z, np.array([self.gps_left_state[0], self.gps_left_state[1]]))
                    self.R = np.append(self.R, np.array([self.gps_left_state[3], self.gps_left_state[4]]))

                    self.H = np.column_stack([self.H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0])])
                    self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0])])

                    self.gps_left_measure = False

                    text += " gps_l "
                else:
                    text += " \t "


                if(self.gps_right_measure):

                    self.Z = np.append(self.Z, np.array([self.gps_right_state[0], self.gps_right_state[1]]))
                    self.R = np.append(self.R, np.array([self.gps_right_state[3], self.gps_right_state[4]]))

                    self.H = np.column_stack([self.H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0])])
                    self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0])])

                    self.gps_right_measure = False

                    text += " gps_r "
                else:
                    text += " \t "


                if(self.imu_left_measure):

                    self.Z = np.append(self.Z, np.array([self.imu_left_state[1], self.imu_left_state[2]]))
                    self.R = np.append(self.R, np.array([self.imu_left_state[4], self.imu_left_state[5]]))

                    self.H = np.column_stack([self.H, np.array([0,0,0,0,1]), np.array([0,0,0,0,0])])
                    self.J_H = np.column_stack([self.J_H, np.array([0,0,0,0,1]), np.array([0,0,0,0,0])])

                    text += " imu_l "
                else:
                    text += " \t "


                if(self.imu_right_measure):

                    self.Z = np.append(self.Z, np.array([self.imu_right_state[1], self.imu_right_state[2]]))
                    self.R = np.append(self.R, np.array([self.imu_right_state[4], self.imu_right_state[5]]))

                    self.H = np.column_stack([self.H, np.array([0,0,0,0,1]), np.array([0,0,0,0,0])])
                    self.J_H = np.column_stack([self.J_H, np.array([0,0,0,0,1]), np.array([0,0,0,0,0])])

                    text += " imu_r "
                else:
                    text += " \t "

                # Reset Measurements check
                self.wheel_odometer_measure = self.visual_odometer_measure = self.gps_measure = self.imu_left_measure = self.imu_right_measure = False

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
                self.H = np.zeros((5,0))
                # Initialise Measurements Jacobian Matrix
                self.J_H = np.zeros((5,0))

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
            #self.P_t = (np.eye(5) - self.K @ Update_J_H) @ self.P_Pred
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite (More time consuming) - Low improvements
            self.P_t = (np.eye(5) - self.K @ Update_J_H) @ self.P_Pred @ (np.eye(5) - self.K @ Update_J_H).T + self.K @ Update_R @ self.K.T
            # Ensure P is symmetric
            #self.P_t = (self.P_t + self.P_t.T) / 2

            if(self.test and self.print):
                print(text + "\t" + str(self.X_t[0:3]))
        else:
            # Keep just the prediction if no new measurements have been received
            self.UpdateNoMeasures()

        if(self.test or self.ros):
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
            twist = Twist(Vector3(self.X_t[3], 0, 0),Vector3(0.0, 0.0, self.X_t[4]))

            twist_covariance     = [0] * 36
            twist_covariance[0]  = self.P_t[3][3]
            twist_covariance[5]  = self.P_t[3][4]
            twist_covariance[30]  = self.P_t[4][3]
            twist_covariance[35]  = self.P_t[4][4]

            twist_ekf = TwistWithCovariance(twist, twist_covariance)

            odom_ekf = Odometry(header, "base_link", pose_ekf, twist_ekf)

            # publish the message
            self.odom_aekf_sim_pub.publish(odom_ekf)


    def Control(self, cmd_vel):

        self.control_t = 1

        z_x_dot = cmd_vel.linear.x
        z_yaw_dot = cmd_vel.angular.z

        z_delta_x_dot = z_x_dot - self.control_state[0] * self.rate / self.steps

        z_x_dot_cov = 0.001
        z_yaw_dot_cov = 0.01
        z_delta_x_dot_cov = 0.01

        if(self.current_status == 1):
            self.control_state = np.array([z_x_dot, z_yaw_dot, z_delta_x_dot])

        if(self.test and self.print):
            print("         Control \t\t" + str(self.control_state))

    def CurrentStatus(self, status):

        self.current_status = status.state

        if(self.test and self.print):
            print("         CurrentStatus \t" + str(self.current_status))

    def GroundTruth(self, Ground):

        z_x = Ground.pose.pose.position.x
        z_y = Ground.pose.pose.position.y

        euler = tf.transformations.euler_from_quaternion([Ground.pose.pose.orientation.x, Ground.pose.pose.orientation.y, Ground.pose.pose.orientation.z, Ground.pose.pose.orientation.w])
        z_yaw_direct = euler[2]
        z_yaw_direct = ((z_yaw_direct + math.pi) % (2*math.pi)) - math.pi
        delta_z_yaw = ((( z_yaw_direct - self.X_t[2]) + math.pi) % (2*math.pi)) - math.pi

        z_yaw = self.X_t[2] + delta_z_yaw

        z_x_dot = Ground.twist.twist.linear.x
        z_yaw_dot = Ground.twist.twist.angular.z

        self.ground_state = np.array([z_x, z_y, z_yaw, z_x_dot, z_yaw_dot])

        if(self.test and self.print):
            print("     Ground \t\t\t" + str(self.ground_state))


    def WheelOdometer(self, Odometry):

        z_x_dot_cov = 0.005
        z_yaw_dot_cov = 0.025

        z_x_dot = Odometry.x
        z_yaw_dot  = Odometry.y

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.wheel_odometer_state = np.array([z_x_dot, z_yaw_dot, z_x_dot_cov**2, z_yaw_dot_cov**2])

            self.wheel_odometer_measure = True
        finally:
            self.lock.release() # release self.lock, no matter what

        if(self.test and self.print):
            print("     WheelOdometer \t\t\t" + str(self.wheel_odometer_state))


    def VisualOdometer(self, VisualOdometry):

        z_x_dot_cov = 0.05
        z_yaw_dot_cov = 0.05

        z_x_dot = VisualOdometry.x
        z_yaw_dot  = VisualOdometry.y

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.visual_odometer_state = np.array([z_x_dot, z_yaw_dot, z_x_dot_cov**2, z_yaw_dot_cov**2])

            self.visual_odometer_measure = True
        finally:
            self.lock.release() # release self.lock, no matter what

        if(self.test and self.print):
            print("     VisualOdometer \t\t" + str(self.visual_odometer_state))



    def GPS(self, GPSfix):

        z_cov = 1
        z_yaw_cov = 0.1

        z_x     = GPSfix.x
        z_y     = GPSfix.y
        z_yaw   = GPSfix.z

        z_yaw_direct = ((z_yaw + math.pi) % (2*math.pi)) - math.pi
        delta_z_yaw = ((( z_yaw_direct - self.X_t[2]) + math.pi) % (2*math.pi)) - math.pi
        z_yaw = self.X_t[2] + delta_z_yaw

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.gps_measure = True

            self.gps_state = np.array([z_x,z_y,z_yaw, z_cov**2, z_cov**2 ,z_yaw_cov**2])
        finally:
            self.lock.release() # release self.lock, no matter what

        if(self.test and self.print):
            print("     GPS \t\t" + str(self.gps_state))


    def gps_left(self, gps_left_fix):

        z_cov = 3
        z_yaw_cov = 0.2

        z_x     = gps_left_fix.x
        z_y     = gps_left_fix.y
        z_yaw   = gps_left_fix.z

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            if(self.gps_left_fusion):
                self.gps_left_measure = True

            self.gps_left_state = np.array([z_x,z_y,0, z_cov**2,z_cov**2,0])
        finally:
            self.lock.release() # release self.lock, no matter what


        if(self.test and self.print):
            print("     gps_left \t\t" + str(self.gps_left_state))


    def gps_right(self, gps_right_fix):

        z_cov = 3
        z_yaw_cov = 0.2

        z_x     = gps_right_fix.x
        z_y     = gps_right_fix.y
        z_yaw   = gps_right_fix.z


        # Make sure the execution is safe
        self.lock.acquire()
        try:
            if(self.gps_right_fusion):
                self.gps_right_measure = True

            self.gps_right_state = np.array([z_x,z_y,0, z_cov**2,z_cov**2,0])
        finally:
            self.lock.release() # release self.lock, no matter what


        if(self.test and self.print):
            print("     gps_right \t\t" + str(self.gps_right_state))



    def ImuLeft(self, imu):

        z_yaw_cov = self.imu_left_var[0]**2
        z_yaw_dot_cov = self.imu_left_var[1]**2
        z_x_dot2_cov = self.imu_left_var[2]**2

        z_yaw     = imu.x
        z_yaw_dot = imu.y
        z_x_dot2  = imu.z

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.imu_left_state = np.array([z_yaw, z_yaw_dot, z_x_dot2, z_yaw_cov, z_yaw_dot_cov, z_x_dot2_cov])

            if(self.imu_left_fusion):
                self.imu_left_measure = True
        finally:
            self.lock.release() # release self.lock, no matter what


        if(self.test and self.print):
            print("     Imu Left \t\t\t\t" + str(self.imu_left_state))


    def ImuRight(self, imu):

        z_yaw_cov = self.imu_left_var[0]**2
        z_yaw_dot_cov = self.imu_left_var[1]**2
        z_x_dot2_cov = self.imu_left_var[2]**2

        z_yaw     = imu.x
        z_yaw_dot = imu.y
        z_x_dot2  = imu.z

        # Make sure the execution is safe
        self.lock.acquire()
        try:
            self.imu_right_state = np.array([z_yaw, z_yaw_dot, z_x_dot2, z_yaw_cov, z_yaw_dot_cov, z_x_dot2_cov])


            if(self.imu_right_fusion):
                self.imu_right_measure = True
        finally:
            self.lock.release() # release self.lock, no matter what

        if(self.test and self.print):
            print("     Imu Right \t\t\t\t" + str(self.imu_right_state))



def groundTruthRMSE(hX, hG):
    RMSE = np.zeros((5, 1))
    for i in range(5):
        MSE = np.square(np.subtract(hG[i,].flatten(),hX[i,].flatten())).mean()
        RMSE[i] = math.sqrt(MSE)
    return RMSE

def groundTruthAME(hX, hG):
    AME = np.zeros((5, 1))
    for i in range(5):
        AME[i] = np.absolute(np.subtract(hG[i,].flatten(), hX[i,].flatten())).mean()
    return AME

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

    fig = plt.figure(figsize=(5, 5))

    #print(P)

    im = plt.imshow(P, interpolation="none", cmap=plt.get_cmap('binary'))

    plt.title('Covariance Matrix $P$ (after %i Filter Steps)' % (m))

    ylocs, ylabels = plt.yticks()
    # set the locations of the yticks
    plt.yticks(np.arange(5))
    # set the locations and labels of the yticks
    plt.yticks(np.arange(5),('$x$', '$y$', '$\\theta$', '$v$', '$\omega$'), fontsize=22)

    xlocs, xlabels = plt.xticks()
    # set the locations of the yticks
    plt.xticks(np.arange(6))
    # set the locations and labels of the yticks
    plt.xticks(np.arange(6),('$x$', '$y$', '$\\theta$', '$v$', '$\omega$', '$a$'), fontsize=22)

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

    fig, (ax0, ax1, ax2, ax3, ax4) = plt.subplots(5, sharex=True)
    fig.suptitle(title)# + ' (after %i Filter Steps)' % (m))
    p0 = ax0.plot(range(m), H[0], label='$x$')
    p1 = ax1.plot(range(m), H[1], label='$y$')
    p2 = ax2.plot(range(m), H[2], label='$\\theta$')
    p3 = ax3.plot(range(m), H[3], label='$v$')
    p4 = ax4.plot(range(m), H[4], label='$\\omega$')

    ax0.legend(shadow=True, fancybox=True)
    ax1.legend(shadow=True, fancybox=True)
    ax2.legend(shadow=True, fancybox=True)
    ax3.legend(shadow=True, fancybox=True)
    ax4.legend(shadow=True, fancybox=True)

    plt.xlabel('Filter Steps')
    plt.ylabel('')
    plt.legend(loc='best')


    plt.pause(1)



if __name__ == '__main__':

    ### DEFINE PLOTTING VARIABLES
    plt.rc('text', usetex=True)         # Use LaTexmat
    plt.rc('font', family='serif')
    plt.rc('font', serif='Times New Roman')
    #sns.set(rc={'text.usetex': True})  # Add background

    print("Start AEKF_Sim")

    aekf_sim = None


    # HISTORY
    # Time steps dt
    hDt = list()
    ## State Vector [x y theta v omega a]'
    hX = np.zeros((5, 1))
    hG = hX
    #hC = hX
    ## Odometry Measures [x y theta v omega a=0]
    #hW = np.zeros((5, 1))
    #hV = np.zeros((5, 1))
    ## GPS Measures [x y theta]
    #hZ = np.zeros((5, 1))
    #hZ_left = np.zeros((5, 1))
    #hZ_right = np.zeros((5, 1))
    ## Innovation K
    hK = np.zeros((5, 1))
    ## Covariance P_t
    hP = np.zeros((5, 1))
    ## Root Mean Square Error for each coordinate
    #RMSE = np.zeros((5, 1))
    #hRMSE_G_X = RMSE
    #hRMSE_G_C = RMSE
    #hRMSE_G_W = RMSE
    #hRMSE_G_V = RMSE
    #hRMSE_G_Z = RMSE
    #hRMSE_G_Z_left = RMSE
    #hRMSE_G_Z_right = RMSE

    try:
        # Initialise the Kalman Filter
        aekf_sim = AEKF_Sim()
        # Wait for the updates (Variable dt)
        #rospy.spin()
        # Continuosly try to get updates and run Prediction and Update (Constant dt)
        hertz = 250 # highest frequency of the sensors (IMU)
        rate = rospy.Rate(hertz)

        # Get the time
        start = rospy.get_time()

        print("Wait for the system to start")
        # Wait for the system to start running
        while start == 0:
            start = rospy.get_time()
            # Sleep before next iteration
            rate.sleep()

        print("Wait for the first command before starting")
        # Wait for first command
        while(aekf_sim.control_t == -1):
            start = rospy.get_time()
            rate.sleep()

        print("Start fusion")
        # Start with the fusion
        end = rospy.get_time()
        while not rospy.is_shutdown():
            # Update dt at each iteration
            start = rospy.get_time()
            dt = start - end
            hDt.append(dt)

            #print("KALMAN - " + str(start) + "  "  +  str(dt))
            # Prediction step
            aekf_sim.Predict(dt)
            # Update step
            aekf_sim.Update()

            # store data history
            hX = np.hstack((hX, aekf_sim.X_t.reshape(5,1)))
            hG = np.hstack((hG, aekf_sim.ground_state.reshape(5,1)))
            #hC = np.hstack((hC, aekf_sim.X_control.reshape(5,1)))
            #hW = np.hstack((hW, aekf_sim.X_wheel_odom.reshape(5,1)))
            #hV = np.hstack((hV, aekf_sim.X_visual_odom.reshape(5,1)))
            #hZ = np.hstack((hZ, np.array([aekf_sim.gps_state[0],aekf_sim.gps_state[1],aekf_sim.gps_state[2],0,0,0]).reshape(5,1)))
            #hZ_left = np.hstack((hZ_left, np.array([aekf_sim.gps_left_state[0],aekf_sim.gps_left_state[1],aekf_sim.gps_left_state[2],0,0,0]).reshape(5,1)))
            #hZ_right = np.hstack((hZ_right, np.array([aekf_sim.gps_right_state[0],aekf_sim.gps_right_state[1],aekf_sim.gps_right_state[2],0,0,0]).reshape(5,1)))
            hK = np.hstack((hK, np.array([aekf_sim.K[0,0], aekf_sim.K[1,0], aekf_sim.K[2,0], aekf_sim.K[3,0], aekf_sim.K[4,0]]).reshape(5,1)))
            hP = np.hstack((hP, np.array([aekf_sim.P_t[0,0], aekf_sim.P_t[1,1], aekf_sim.P_t[2,2], aekf_sim.P_t[3,3], aekf_sim.P_t[4,4]]).reshape(5,1)))

            # Compute the ground Truth RMSE between Ground and Kalman
            #RMSE_G_X = groundTruthRMSE(hG, hX)
            #hRMSE_G_X = np.hstack((hRMSE_G_X, RMSE_G_X.reshape(5,1)))
            ## Compute the ground Truth RMSE between Ground and Control
            #RMSE_G_C = groundTruthRMSE(hG, hC)
            #hRMSE_G_C = np.hstack((hRMSE_G_C, RMSE_G_C.reshape(5,1)))
            ## Compute the ground Truth RMSE between Ground and Wheel Odometry
            #RMSE_G_W = groundTruthRMSE(hG, hW)
            #hRMSE_G_W = np.hstack((hRMSE_G_W, RMSE_G_W.reshape(5,1)))
            ## Compute the ground Truth RMSE between Ground and Visual Odometry
            #RMSE_G_V = groundTruthRMSE(hG, hV)
            #hRMSE_G_V = np.hstack((hRMSE_G_V, RMSE_G_V.reshape(5,1)))
            ## Compute the ground Truth RMSE between Ground and GPS
            #RMSE_G_Z = groundTruthRMSE(hG, hZ)
            #hRMSE_G_Z = np.hstack((hRMSE_G_Z, RMSE_G_Z.reshape(5,1)))
            ## Compute the ground Truth RMSE between Ground and GPS_left
            #RMSE_G_Z_left = groundTruthRMSE(hG, hZ_left)
            #hRMSE_G_Z_left = np.hstack((hRMSE_G_Z_left, RMSE_G_Z_left.reshape(5,1)))
            ## Compute the ground Truth RMSE between Ground and GPS_right
            #RMSE_G_Z_right = groundTruthRMSE(hG, hZ_right)
            #hRMSE_G_Z_right = np.hstack((hRMSE_G_Z_right, RMSE_G_Z_right.reshape(5,1)))

            """ Plot in real time (time consuming)
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(hX[0, :].flatten(),
                    hX[1, :].flatten(), "-b")
            plt.plot(hZ[0, :],
                    hZ[1, :], ".g")
            px, py = plot_covariance_ellipse(aekf_sim.X_t, aekf_sim.P_t)
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

    print("X_t")
    print(aekf_sim.X_t)
    print("P_t")
    print(aekf_sim.P_t)
    print("Final AME: Ground vs Kalman")
    print(groundTruthAME(hG, hX))
    print("Final RMSE: Ground vs Kalman")
    print(groundTruthRMSE(hG, hX))
    #print("Final RMSE: Ground vs WheelOdometry")
    #print(RMSE_G_W)
    #print("Final RMSE: Ground vs VisualOdometry")
    #print(RMSE_G_V)
    #print("Final RMSE: Ground vs Control")
    #print(RMSE_G_C)
    #print("Final RMSE: Ground vs GPS")
    #print(RMSE_G_Z)
    #print("Final RMSE: Ground vs GPS left")
    #print(RMSE_G_Z_left)
    #print("Final RMSE: Ground vs GPS right")
    #print(RMSE_G_Z_right)

    #plotFinalCovarianceP(aekf_sim.P_t, hP.shape[1])
    #plotHistory(hK, hK.shape[1], 'Innovation Gain $K$')
    #plotHistory(hP, hP.shape[1], 'Covariance Matrix $P$')
    #plotHistory(hRMSE_G_X, hRMSE_G_X.shape[1], 'RMSE: Ground vs Kalman')
    #plotHistory(hRMSE_G_C, hRMSE_G_C.shape[1], 'RMSE: Ground vs Control')
    #plotHistory(hRMSE_G_W, hRMSE_G_W.shape[1], 'RMSE: Ground vs WheelOdometry')
    #plotHistory(hRMSE_G_V, hRMSE_G_V.shape[1], 'RMSE: Ground vs VisualOdometry')
    #plotHistory(hRMSE_G_Z, hRMSE_G_Z.shape[1], 'RMSE: Ground vs GPS')
    #plotHistory(hRMSE_G_Z_left, hRMSE_G_Z_left.shape[1], 'RMSE: Ground vs GPS left')
    #plotHistory(hRMSE_G_Z_right, hRMSE_G_Z_right.shape[1], 'RMSE: Ground vs GPS right')



    #plt.figure(20)
    #plt.cla()
    ## for stopping simulation with the esc key.
    #plt.gcf().canvas.mpl_connect('key_release_event',
    #        lambda event: [exit(0) if event.key == 'escape' else None])
    #plt.plot(hZ_left[0, :],
    #         hZ_left[1, :], color='red', marker='3', linestyle="None",
    #         label = "GPS Left")
    #plt.plot(hZ_right[0, :],
    #         hZ_right[1, :], color='orange', marker='4', linestyle="None",
    #         label = "GPS Right")
    #plt.plot(hZ[0, :],
    #         hZ[1, :], color='magenta', marker='1', linestyle="None",
    #         label = "GPS")
    #plt.plot(hC[0, :].flatten(),
    #         hC[1, :].flatten(), color='deepskyblue', linestyle='dashed', linewidth = 2,
    #         label = "Control")
    #plt.plot(hW[0, :].flatten(),
    #         hW[1, :].flatten(), color='teal', linestyle='dotted', linewidth = 2,
    #         label = "Wheel")
    #plt.plot(hV[0, :].flatten(),
    #         hV[1, :].flatten(), color='lawngreen', linestyle='dotted', linewidth = 2,
    #         label = "Visual")
    #plt.plot(hG[0, :].flatten(),
    #         hG[1, :].flatten(), color='black', linestyle='solid', linewidth = 2,
    #         label = "Ground")
    #plt.plot(hX[0, :].flatten(),
    #         hX[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
    #         label = "EKF")
    #plt.legend()



    ## Plot histogram of features
    #fig, ax = plt.subplots()
    # the histogram of the data
    #num_bins = np.arange(0,0.01,0.0005)
    #n, bins, patches = ax.hist(hDt, num_bins, density=True, stacked=True)
    # Gaussian Kernel
    #from scipy.stats import norm
    #density = sum(norm(dt).pdf(bins) for dt in hDt)
    #plt.fill_between(bins, density, alpha=0.5)
    #sns.distplot(hDt, hist=True, kde=True, rug=True,)
             #color = 'blue', bins=10000,
             #hist_kws={'edgecolor':'black'},
             #label = "Gaussian Kernel",
             #kde_kws={'linewidth': 1})
    ## add a 'best fit' line
    #mu = np.mean(hDt)
    #sigma = np.std(hDt)
    #y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
    #    np.exp(-0.5 * (1 / sigma * (bins - mu))**2))
    #ax.plot(bins, y, '--', label = "Gaussian Distribution")
    ## Plot real values
    #ax.plot(hDt, np.full_like(hDt, -0.01), '|k', markeredgewidth=1)
    ## Labels
    #ax.set_title('Histogram of $\Delta_t$: $\mu='+ str(mu)+'$, $\sigma='+ str(sigma)+'$') #Assign title
    #ax.set_xlabel('$\Delta_t$: min=$' + str(np.min(hDt)) + '$ MAX=$' + str(np.max(hDt)) +'$') #Assign x label
    #ax.set_ylabel('Number of steps') #Assign y label
    #plt.legend()
    ## Tweak spacing to prevent clipping of ylabel
    #fig.tight_layout()
    #plt.pause(1)


    resultFileName = "/home/marco/Videos/Tests/SimulatedTest.txt"

    with open(resultFileName, "w+") as resultFile:
        resultFile.write("Number of steps: " + str(len(hDt)))
        resultFile.write("\nX_t\n")
        np.savetxt(resultFile, aekf_sim.X_t)
        resultFile.write("\nP_t\n")
        np.savetxt(resultFile, aekf_sim.P_t)
        resultFile.write("\nHistory of X\n")
        np.savetxt(resultFile, hX)
        resultFile.write("\nHistory of P_t\n")
        np.savetxt(resultFile, hP)
        resultFile.write("\nHistory of K_t\n")
        np.savetxt(resultFile, hK)
        resultFile.write("\nAME : Ground vs EKF\n")
        np.savetxt(resultFile, groundTruthAME(hG, hX))
        resultFile.write("\nRMSE : Ground vs EKF\n")
        np.savetxt(resultFile, groundTruthRMSE(hG, hX))
    #    resultFile.write("\nHistory of Ground\n")
    #    np.savetxt(resultFile, hG)
    #    resultFile.write("\nHistory of Control\n")
    #    np.savetxt(resultFile, hC)
    #    resultFile.write("\nHistory of Wheel Odometry\n")
    #    np.savetxt(resultFile, hW)
    #    resultFile.write("\nHistory of Visual Odometry\n")
    #    np.savetxt(resultFile, hV)
    #    resultFile.write("\nHistory of GPS\n")
    #    np.savetxt(resultFile, hZ)
    #    resultFile.write("\nHistory of GPS left\n")
    #    np.savetxt(resultFile, hZ_left)
    #    resultFile.write("\nHistory of GPS right\n")
    #    np.savetxt(resultFile, hZ_right)

    # Wait for the user input to terminate the program
    input("Press any key to terminate the program\n")
    # Print bye message
    print("THE END")
