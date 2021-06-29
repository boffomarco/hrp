#!/usr/bin/env python3

import time

import math

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate

from am_driver.msg import SensorStatus

import threading

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

import numpy as np

# import the random module
import random


class SimulatedSensor():
    def __init__(self):

        # Define name of the Node
        rospy.init_node("SimulatedSensor", anonymous=True)

        # Fetch the parameters
        self.pubTopic = rospy.get_param(rospy.search_param('pubTopic'), 'topic')
        self.pubFrequency = rospy.get_param(rospy.search_param('pubFrequency'), 1)
        self.index = [0.0, 0.0, 0.0]
        self.index[0] = rospy.get_param(rospy.search_param('index_0'), 0)
        self.index[1] = rospy.get_param(rospy.search_param('index_1'), 1)
        self.index[2] = rospy.get_param(rospy.search_param('index_2'), 2)
        self.std = [0.0, 0.0, 0.0]
        self.std[0] = rospy.get_param(rospy.search_param('std_0'), 1)
        self.std[1] = rospy.get_param(rospy.search_param('std_1'), 1)
        self.std[2] = rospy.get_param(rospy.search_param('std_2'), 1)

        # Frequency of the sensors
        self.rate = rospy.Rate(self.pubFrequency)

        # Publish
        self.topic_pub = rospy.Publisher(self.pubTopic, Vector3, queue_size=20)

        # Subscribe Ground Truth
        rospy.Subscriber('/Odom_Ground', Odometry, self.GroundTruth)
        self.ground_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


        print("Initialised "+str())

    def simulate(self):

        # Get the time
        start = rospy.get_time()

        # Generate data
        while not rospy.is_shutdown():
            simMeasure = [0.0, 0.0, 0.0]

            simMeasure[0] = random.gauss(mu=self.ground_state[self.index[0]], sigma=self.std[0])
            simMeasure[1] = random.gauss(mu=self.ground_state[self.index[1]], sigma=self.std[1])
            simMeasure[2] = random.gauss(mu=self.ground_state[self.index[2]], sigma=self.std[2])

            v3 = Vector3(simMeasure[0],simMeasure[1],simMeasure[2])

            self.topic_pub.publish(v3)

            # Sleep before next iteration
            self.rate.sleep()


    def GroundTruth(self, Odom_Ground):
        # Store the robot's pose
        self.ground_state[0] = Odom_Ground.pose.pose.position.x
        self.ground_state[1] = Odom_Ground.pose.pose.position.y
        euler = tf.transformations.euler_from_quaternion([Odom_Ground.pose.pose.orientation.x, Odom_Ground.pose.pose.orientation.y, Odom_Ground.pose.pose.orientation.z, Odom_Ground.pose.pose.orientation.w])
        self.ground_state[2] = euler[2] # In radians
        self.ground_state[3] = Odom_Ground.twist.twist.linear.x
        self.ground_state[4] = Odom_Ground.twist.twist.angular.z
        self.ground_state[5] = 0



if __name__ == "__main__":

    try:
        # Create the object
        sensor = SimulatedSensor()

        # Get the time
        start = rospy.get_time()

        print("Wait for the system to start")
        # Wait for the system to start running
        while start == 0:
            start = rospy.get_time()

            # Sleep before next iteration
            time.sleep(0.001)

        print("Simulate")
        sensor.simulate()




    except rospy.ROSInterruptException:
        # Wait for the user input to terminate the program
        input("Press any key to terminate the program\n")
        pass

    # Print bye message
    print("THE END")


