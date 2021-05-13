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


class GroundTruth():

    def __init__(self):

        # Define name of the Node
        rospy.init_node("GroundTruth", anonymous=True)

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

        # Define set of topics to subscribe to
        rospy.Subscriber('cmd_vel', Twist, self.Control)
        self.control_t = -1

        # Publish Ground Truth Twist
        self.ground_pub = rospy.Publisher('GroundTruth', Twist, queue_size=20)
        self.ground_twist = np.array([0.0, 0.0])

        # Define GT variables
        self.delay = 0.1
        self.xVel = 0.3
        self.yawVel = 1
        # Define GT path
        # 2021-04-30-15-47-56
        #self.ground_truth_dt     = [self.delay*3, 4/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 9/self.xVel,self.delay, math.pi/2/self.yawVel,  self.delay, 2/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 2/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 4/self.xVel,    self.delay, math.pi/2/self.yawVel, self.delay, 3/self.xVel,    self.delay, math.pi/2/self.yawVel, self.delay, 4/self.xVel,    self.delay, math.pi/2/self.yawVel, self.delay, 3/self.xVel,    self.delay, math.pi/2/self.yawVel, self.delay, 4/self.xVel,    self.delay, math.pi/2/self.yawVel, self.delay, 3/self.xVel,    self.delay, math.pi/2/self.yawVel ]
        #self.ground_truth_xVel   = [0,            self.xVel,      0,          0,                      0,          self.xVel,  0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                     0,          self.xVel,      0,          0,                     0,          self.xVel,      0,          0,                     0,          self.xVel,      0,          0,                     0,          self.xVel,      0,          0,                     0,          self.xVel,      0,          0                     ]
        #self.ground_truth_yawVel = [0,            0,              0,          -self.yawVel,           0,          0,          0,          self.yawVel,            0,          0,              0,          self.yawVel,            0,          0,              0,          self.yawVel,            0,          0,              0,          -self.yawVel,           0,          0,              0,          self.yawVel,            0,          0,              0,          -self.yawVel,          0,          0,              0,          -self.yawVel,          0,          0,              0,          -self.yawVel,          0,          0,              0,          -self.yawVel,          0,          0,              0,          -self.yawVel,          0,          0,              0,          -self.yawVel          ]

        # Define GT path
        # 2021-04-30-15-31-23
        #self.ground_truth_dt     = [self.delay*3, 18/self.xVel,   self.delay, math.pi/2/self.yawVel,  self.delay, 5/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 5/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 15/self.xVel,   self.delay, math.pi/2/self.yawVel,  self.delay]
        #self.ground_truth_xVel   = [0,            self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,          self.xVel,      0,          0,                      0,        ]
        #self.ground_truth_yawVel = [0,            0,              0,          self.yawVel,            0,          0,              0,          self.yawVel,            0,          0,              0,          self.yawVel,            0,          0,              0,          -self.yawVel,           0,          0,              0,          self.yawVel,            0,        ]


        # Define GT path
        # 2021-04-21-12-42-20
        #self.ground_truth_dt     = [self.delay*3, 10/self.xVel,  self.delay,  math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 10/self.xVel,   self.delay,  math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 10/self.xVel,   self.delay,  math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 10/self.xVel,   self.delay,  math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 10/self.xVel,   self.delay,  math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 10/self.xVel,   self.delay,  math.pi/2/self.yawVel,  self.delay, 3/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 0.5/self.xVel,  self.delay  ]
        #self.ground_truth_xVel   = [0,            self.xVel,      0,          0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,      0           ]
        #self.ground_truth_yawVel = [0,            0,              0,          -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0,           -self.yawVel,           0,          0,              0           ]


        # Define GT path
        # 2021-04-30-15-38-12
        self.ground_truth_dt     = [self.delay*3, 15/self.xVel,  self.delay,  math.pi/2/self.yawVel,  self.delay, 5/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 2.5/self.xVel, self.delay ]
        self.ground_truth_xVel   = [0,            self.xVel,      0,          0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,     0          ]
        self.ground_truth_yawVel = [0,            0,              0,          self.yawVel,            0,          0,              0,           -self.yawVel,           0,          0,             0          ]




        # Define set of topics to publish
        if(self.test and self.ros):
            self.odom_t_pub = rospy.Publisher('Odom_Ground', Odometry, queue_size=20)


        # State Variables
        self.x_t = 0.0
        self.y_t = 0.0
        self.yaw_t = 0.0
        self.x_dot_t = 0.0
        self.yaw_dot_t = 0.0
        self.x_dot2_t = 0.0

        # State-Vector
        self.X_t = np.array([self.x_t,      self.y_t,      self.yaw_t,
                             self.x_dot_t,  self.yaw_dot_t, self.x_dot2_t])

        print("Initialised GroundTruth")


    # Prediction step with only the kinematic model
    def Predict(self, dt, xVel, yawVel):

        # State-Transition Matrix
        A_t = np.array([[1.0, 0.0, 0.0, cos(self.X_t[2])*dt, 0.0, cos(self.X_t[2])*(dt**2)/2],
                        [0.0, 1.0, 0.0, sin(self.X_t[2])*dt, 0.0, sin(self.X_t[2])*(dt**2)/2],
                        [0.0, 0.0, 1.0, 0.0,  dt, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        # Check control difference
        u_t = np.array([0.0,
                        0.0,
                        0.0,
                        (xVel - self.X_t[3]),
                        (yawVel - self.X_t[4]),
                        - self.X_t[5]]) # To ensure Zero Acceleration behaviour


        # Make sure the execution is safe
        self.lock.acquire()
        try:
            # Ground truth data
            self.X_t = A_t @ self.X_t + u_t

        finally:
            self.lock.release() # release self.lock, no matter what


        if(self.test and self.ros):
            # Send the Update of the Ground Truth to Ros
            header = Header()
            header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            header.frame_id = "odom"

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.X_t[2])

            # next, we'll publish the pose message over ROS
            pose = Pose(Point(self.X_t[0], self.X_t[1], 0.), Quaternion(*odom_quat))

            pose_covariance     = [0] * 36

            pose_t = PoseWithCovariance(pose, pose_covariance)

            # next, we'll publish the twist message over ROS
            twist = Twist(Vector3(self.X_t[3], 0, self.X_t[5]),Vector3(0.0, 0.0, self.X_t[4]))

            twist_covariance     = [0] * 36

            twist_t = TwistWithCovariance(twist, twist_covariance)

            odom_t = Odometry(header, "base_link", pose_t, twist_t)

            # publish the message
            self.odom_t_pub.publish(odom_t)


    def generate(self):

        start = rospy.get_time()

        # Continuosly try to get updates
        hertz = 250 # highest frequency of the sensors (IMU)
        rate = rospy.Rate(hertz)

        # Wait for first command
        while(self.control_t == -1):
            start = rospy.get_time()
            rate.sleep()


        print("Start generation")
        # Start with the fusion
        end = rospy.get_time()
        # Iterate over man made commands
        for i in range(0, len(self.ground_truth_dt)):

            start = rospy.get_time()
            while (rospy.get_time() - start < self.ground_truth_dt[i]):


                # next, we'll publish the twist message over ROS
                twist = Twist(Vector3(self.ground_truth_xVel[i], 0, 0),Vector3(0.0, 0.0, self.ground_truth_yawVel[i]))

                self.ground_pub.publish((twist))

                # Update dt at each iteration
                dt = rospy.get_time() - end

                # Prediction step
                if(self.test and self.ros):
                    self.Predict(dt, self.ground_truth_xVel[i], self.ground_truth_yawVel[i])

                end = rospy.get_time()

                rate.sleep()


        print("End of GT generation")

        return


    def Control(self, cmd_vel):

        if(self.control_t < 0):
            self.control_t = 1

            if(self.test and self.print):
                print("\t Control \t x' = " + str(cmd_vel.linear.x) + " \t yaw' = " + str(cmd_vel.angular.z))





if __name__ == '__main__':

    print("Start GroundTruth")

    groundTruth = None

    try:
        # Initialise the GroundTruth
        groundTruth = GroundTruth()

        # Continuosly try to get updates
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
        groundTruth.generate()


    except rospy.ROSInterruptException:

        pass

