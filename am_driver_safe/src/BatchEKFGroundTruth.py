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

import time

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

        # Define set of topics to publish

        self.odom_control_pub = rospy.Publisher('Odom_Control', Odometry, queue_size=20)

        self.odom_ground_pub = rospy.Publisher('Odom_Ground', Odometry, queue_size=20)


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

        print("Initialised Full_EKF")


    # Prediction step with only the kinematic model
    def Transation(self, dt):

        # State-Transition Matrix
        A = np.array([  [1.0, 0.0, 0.0, cos(self.X_t[2])*dt, 0.0, cos(self.X_t[2])*(dt**2)/2],
                        [0.0, 1.0, 0.0, sin(self.X_t[2])*dt, 0.0, sin(self.X_t[2])*(dt**2)/2],
                        [0.0, 0.0, 1.0, 0.0,  dt, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        # Check control difference
        u = np.array([  0.0,
                        0.0,
                        0.0,
                        self.control_state[0] - self.X_t[3],
                        self.control_state[1] - self.X_t[4],
                        0.0])

        steps = 2
        B = np.diag(np.array([0,0,0,1/steps,1/steps,0]))

        # Prediction State
        self.X_t = A @ self.X_t + B @ u

        # Send the Update to Ros
        header = Header()
        header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        header.frame_id = "odom"

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_t[2])

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(self.X_t[0], self.X_t[1], 0.), Quaternion(*odom_quat))

        pose_covariance     = [0] * 36
        pose_ground_ekf = PoseWithCovariance(pose, pose_covariance)

        # next, we'll publish the pose message over ROS
        twist = Twist(Vector3(self.X_t[3], 0, self.X_t[5]),Vector3(0.0, 0.0, self.X_t[4]))

        twist_covariance     = [0] * 36

        twist_ground_ekf = TwistWithCovariance(twist, twist_covariance)

        odom_ground_ekf = Odometry(header, "base_link", pose_ground_ekf, twist_ground_ekf)

        # publish the message
        self.odom_ground_pub.publish(odom_ground_ekf)


    def Control(self, cmd_vel):

        self.control_t = 1

        z_x_dot = cmd_vel.linear.x
        z_yaw_dot = cmd_vel.angular.z

        z_x_dot_cov = 0.001
        z_yaw_dot_cov = 0.01

        self.control_state = np.array([z_x_dot, z_yaw_dot])
        print("         Control \t\t" + str(self.control_state))

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




if __name__ == '__main__':

    print("Start Full_EKF")

    full_ekf = None

    try:
        # Initialise the Kalman Filter
        full_ekf = Full_EKF()
        # Wait for the updates (Variable dt)
        #rospy.spin()
        # Continuosly try to get updates and run Prediction and Update (Constant dt)
        hertz = 200 # highest frequency of the sensors (IMU)
        rate = rospy.Rate(hertz)

        # Get the time
        start = rospy.get_time()

        print("Wait for the system to start")
        # Wait for the system to start running
        while start == 0:
            start = rospy.get_time()
            # Sleep before next iteration
            rate.sleep()


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


        print("Start fusion")
        # Start with the fusion
        end = rospy.get_time()
        while not rospy.is_shutdown():
            # Update dt at each iteration
            start = rospy.get_time()
            dt = start - end
            hDt.append(dt)

            print("KALMAN - " + str(start) + "  "  +  str(dt))
            #print("State " + str(full_ekf.X_t))
            # Transation step
            full_ekf.Transation(dt)

            # store data history
            hX = np.hstack((hX, full_ekf.X_t.reshape(6,1)))


            # Sleep before next iteration
            rate.sleep()
            #time.sleep(0.005)
            # Reset time
            end = start

    except rospy.ROSInterruptException:

        pass


    #for i in range(len(hPx)):
    #    plt.plot(hPx[i], hPy[i], "--r")

    print(full_ekf.X_t)
    print(full_ekf.P_t)
    print("End Full_EKF")

    plt.figure(10)
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(hX[0, :].flatten(),
            hX[1, :].flatten(), "-b")


    plt.figure(5)
    bins =  np.arange(0,0.01,0.0001)
    plt.hist(hDt)#,bins = bins)
    plt.pause(1)


    # Wait for the user input to terminate the program
    input("Press any key to terminate the program\n")
    # Print bye message
    print("THE END")
