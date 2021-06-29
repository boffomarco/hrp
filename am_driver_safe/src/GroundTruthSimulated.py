#!/usr/bin/env python3

# From https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py

import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
from sensor_msgs.msg import NavSatFix, Imu
from am_driver.msg import WheelEncoder, SensorStatus, CurrentStatus
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

        # Switch among defined controls or simulated controls
        self.controls = False

        # Get the current time
        now = rospy.get_time()

        rospy.Subscriber('cmd_vel', Twist, self.Control)
        self.control_fusion = True
        self.control_measure = False
        self.control_t = -1
        self.control_state = np.array([0.0, 0.0])

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
        #self.ground_truth_dt     = [self.delay*3, 15/self.xVel,  self.delay,  math.pi/2/self.yawVel,  self.delay, 5/self.xVel,    self.delay,  math.pi/2/self.yawVel,  self.delay, 2.5/self.xVel, self.delay ]
        #self.ground_truth_xVel   = [0,            self.xVel,      0,          0,                      0,          self.xVel,      0,           0,                      0,          self.xVel,     0          ]
        #self.ground_truth_yawVel = [0,            0,              0,          self.yawVel,            0,          0,              0,           -self.yawVel,           0,          0,             0          ]


        # Define GT path
        # 2021-05-13-14-12-59
        self.ground_truth_dt     = [self.delay*2, 62.5/0.3014,    self.delay, math.pi/2/1.4,  self.delay, 20/0.2965,   self.delay,  math.pi/2/self.yawVel,  self.delay, 62.5/0.3009, self.delay,  math.pi/2/0.9,  self.delay, 20/0.2976,   self.delay*490, 15/0.3096,   self.delay,  math.pi/2/1.3,  self.delay, 62.5/0.3014,  self.delay,  math.pi/2/1.05,  self.delay, 15/0.3325,   self.delay,  math.pi/2/0.97,     self.delay,   62.5/0.2991,  self.delay*543,  math.pi/0.98,   self.delay*176,  62.5/0.3024,  self.delay,  math.pi/2/1.17,    self.delay,  15/0.332,   self.delay,  math.pi/2/0.9,    self.delay,   30/0.3064,   self.delay,   1.267/0.9,   self.delay,  10/0.45,    self.delay,  math.pi/1,  self.delay,  2.8/0.4,  self.delay,  0.3035/0.7,  self.delay,  7/0.3,     self.delay,  math.pi/2/1.25,   self.delay,  30.3/0.2906,    self.delay, math.pi/2/0.97,  self.delay,  15/0.328, self.delay*184,  math.pi/1.3,  self.delay*980, ]
        self.ground_truth_xVel   = [0,            0.3014   ,      0,          0,              0,          0.2965,      0,           0,                      0,          0.3009,      0,           0,              0,          0.2976,      0,              0.3096,      0,           0,              0,          0.3014,       0,           0,               0,          0.3325,      0,           0,                  0,            0.2991,       0,               0,              0,               0.3024,       0,           0,                 0,           0.332,      0,           0,                0,            0.3064,      0,            0,           0,           0.45,       0,           0,          0,           0.4,      0,           0,           0,           0.3,       0,           0,                0,           0.2906,         0,          0,               0,           0.328,      0,               0,             0,          ]
        self.ground_truth_yawVel = [0,            0,              0,          -1.4,           0,          0,           0,           -self.yawVel,           0,          0,           0,           -0.9,           0,          0,           0,              0,           0,           -1.3,           0,          0,            0,           -1.05,           0,          0,           0,           -0.97,              0,            0,            0,               0.98,           0,               0,            0,           1.17,              0,           0,          0,           0.9,              0,            0,           0,            -0.9,        0,           0,          0,           1,          0,           0,        0,           -0.7,        0,           0,         0,          -1.25,            0,           0,              0,          0.97,            0,           0,          0,               1.3,           0,          ]


        # Define GT path
        # Collision Map
        #self.delay = 0.1
        #self.xVel = 1
        #self.yawVel = 1
        #self.ground_truth_dt     = [self.delay,    3/self.xVel,    self.delay, math.pi/2/self.yawVel,  self.delay, 6/self.xVel, self.delay, math.pi/2/self.yawVel,  self.delay, 6/self.xVel, self.delay, math.pi/2/self.yawVel,  self.delay, 6/self.xVel, self.delay, math.pi/2/self.yawVel,  self.delay, 3/self.xVel, self.delay*10, ]
        #self.ground_truth_xVel   = [0,             self.xVel,      0,          0,                      0,          self.xVel,   0,          0,                      0,          self.xVel,   0,          0,                      0,          self.xVel,   0,          0,                      0,          self.xVel,   0,             ]
        #self.ground_truth_yawVel = [0,             0,              0,          -self.yawVel,           0,          0,           0,          -self.yawVel,           0,          0,           0,          -self.yawVel,           0,          0,           0,          -self.yawVel,           0,          0,           0,             ]

        #self.ground_truth_dt     = [self.delay,  math.pi/2/self.yawVel,  self.delay,    3/self.xVel,   ]
        #self.ground_truth_xVel   = [0,           0,                      0,             self.xVel,     ]
        #self.ground_truth_yawVel = [0,           -self.yawVel,           0,             0,             ]

        # Define set of topics to publish
        if(self.test and self.ros):
            self.odom_t_pub = rospy.Publisher('Odom_Ground', Odometry, queue_size=20)

        # Total path length
        self.length = 0.0
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
        dt = 0.004 # Fix dt to avoid computation issues

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

        # Ground truth data
        self.X_t = A_t @ self.X_t + u_t

        self.length = self.length + xVel * dt

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

        if(self.controls):
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

        else:

            end = rospy.get_time()
            while not rospy.is_shutdown():
                # Update dt at each iteration
                dt = rospy.get_time() - end

                # Exit when changing x velocity
                if( not (self.control_state[0] == 0 or abs(self.control_state[0]) == 0.3)):
                    print(self.control_state[0])
                    return

                # Prediction step
                if(self.test and self.ros):
                    self.Predict(dt, self.control_state[0], self.control_state[1])

                end = rospy.get_time()

                rate.sleep()

        return


    def Control(self, cmd_vel):

        if(self.control_t < 0):
            self.control_t = 1

        z_x_dot = cmd_vel.linear.x
        z_yaw_dot = cmd_vel.angular.z

        z_delta_x_dot = 0

        self.control_state = np.array([z_x_dot, z_yaw_dot, z_delta_x_dot])

        if(self.test and self.print):
            print("         Control \t\t" + str(self.control_state))



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

        """
        print("Wait for the system to start")
        # Wait for the system to start running
        while start == 0:
            start = rospy.get_time()
            # Sleep before next iteration
            rate.sleep()
        """

        print("Wait for the first command before starting")
        groundTruth.generate()


    except rospy.ROSInterruptException:

        pass



    print("Path of length in meters = " + str(groundTruth.length))
    print("End of GT generation")
