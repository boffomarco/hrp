#!/usr/bin/env python3

import math

import rospy
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

# https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
# https://wiki.ros.org/map_server


class CollisionOccupancyMap():

    def __init__(self):

        # Define name of the Node
        rospy.init_node("CollisionOccupancyMap", anonymous=True)

        # Define the run type of the Filter
        self.test = True
        self.print = False
        self.ros = True

        # Define the self.lock to allow multi-threading
        self.lock = threading.Lock()

        # Check if the filter is ready to start
        self.map = False

        # Get the current time
        now = rospy.get_time()

        # Define set of topics to subscribe to

        rospy.Subscriber('sensor_status', SensorStatus, self.Sensor)
        self.collision = False

        rospy.Subscriber('Odom_Ground', Odometry, self.GroundTruth)
        self.ground_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('Odom_Full_EKF', Odometry, self.EKF)
        self.ekf_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.map = OccupancyGrid()

        #self.map.header.stamp = rospy.get_time()
        self.map.header.frame_id = "odom"

        # Resolution in meters
        self.map.info.resolution = 0.5
        # Able to account for area of 5000m^2 + 20% for Automower 450X
        self.map.info.width  = int( 75 / self.map.info.resolution )
        self.map.info.height = int( 75  / self.map.info.resolution )

        # Set the map as horizontal and at the center of the robot
        self.map.info.origin.position.x = - self.map.info.width / 2 * self.map.info.resolution
        self.map.info.origin.position.y = - self.map.info.height / 2 * self.map.info.resolution
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.x = 0.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 0.0

        # Set the map at 50 with borders at -1
        self.map.data = []
        for i in range(0, self.map.info.width*self.map.info.height):
            if((i%self.map.info.width == 0) or ((i + 1)%self.map.info.width == 0)):
                self.map.data.append(-1)
            elif((i < self.map.info.width) or (i > (self.map.info.height-1)*self.map.info.width)):
                self.map.data.append(-1)
            else:
                self.map.data.append(50)






        # Define set of topics to publish
        if(self.test and self.ros):
            self.map_pub = rospy.Publisher('CollisionMap', OccupancyGrid, latch=True, queue_size=20)
            self.map_pub.publish(self.map)

        print("Initialised CollisionOccupancyMap")

    def Sensor(self, sensor_status):
        if(sensor_status.sensorStatus == 4):
            print(Collision)
        print("Sensor")

    def GroundTruth(self, Odom_Ground):
        print("GroundTruth")

    def EKF(self, Odom_Full_EKF):
        print("EKF")


if __name__ == '__main__':

    print("Start CollisionOccupancyMap")

    collision = None

    try:
        # Initialise the CollisionOccupancyMap
        collision = CollisionOccupancyMap()
        """
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
        """
        rospy.spin()
        #collision.Check()

    except rospy.ROSInterruptException:

        pass

    # Wait for the user input to terminate the program
    input("Press any key to terminate the program\n")
    # Print bye message
    print("THE END")
