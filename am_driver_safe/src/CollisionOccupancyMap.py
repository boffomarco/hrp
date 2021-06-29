#!/usr/bin/env python3

# [TODO]
# - Use of Pose Covariance in the mapping (not really needed)

import math
from time import time_ns

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate

from am_driver.msg import SensorStatus, Mode
from std_msgs.msg import UInt16

import threading

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

import numpy as np

# import the random module
import random

# https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
# https://wiki.ros.org/map_server


class CollisionOccupancyMap():

    def __init__(self, boundary):

        # Define name of the Node
        rospy.init_node("CollisionOccupancyMap", anonymous=True)

        # Define the run type of the Filter
        self.test = True
        self.print = False
        self.ros = True

        # Check if the map should define the boundary or map
        self.boundary = boundary

        # Get the current time
        now = rospy.get_time()

        # Define set of topics to subscribe to

        rospy.Subscriber('sensor_status', SensorStatus, self.Sensor)
        rospy.Subscriber('cmd_mode', UInt16, self.GetMode)
        self.collision = False

        rospy.Subscriber('Odom_Ground', Odometry, self.GroundTruth)
        self.ground_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        rospy.Subscriber('Odom_AEKF_Sim', Odometry, self.EKF)
        self.ekf_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Save the DOFs of the Robot
        self.X_g = [0.0,0.0,0.0]
        self.X_t = [0.0,0.0,0.0]
        self.P_t = [0.0,0.0,0.0]
        # Size of Automower
        self.X_size = [-0.22, 0.50] # 72cm length on the Automower 450X
        self.Y_size = [-0.23, 0.23] # 56cm width on the Automower 450X


        self.maxObject = 25 # Value to discriminate if there is an object or not


        # Create occupancy grid
        self.map = OccupancyGrid()
        #self.map.header.stamp = rospy.get_time()
        self.map.header.frame_id = "odom"
        # Resolution in meters
        self.map.info.resolution = 0.1
        # Able to account for area of 5000m^2 + 20% for Automower 450X
        self.map.info.width  = int( 5 * 2 / self.map.info.resolution )
        self.map.info.height = int( 5 * 2 / self.map.info.resolution )
        # Set the map as horizontal and at the center of the robot
        self.map.info.origin.position.x = - self.map.info.width / 2 * self.map.info.resolution
        self.map.info.origin.position.y = - self.map.info.height / 2 * self.map.info.resolution
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.x = 0.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 0.0
        # Initialise the occupancy grid
        # Set the map at 50 with borders at -1
        self.map.data = []
        for i in range(0, self.map.info.width*self.map.info.height):
            if((i%self.map.info.width == 0) or ((i + 1)%self.map.info.width == 0)):
                self.map.data.append(-1)
            elif((i < self.map.info.width) or (i > (self.map.info.height-1)*self.map.info.width)):
                self.map.data.append(-1)
            else:
                self.map.data.append(50)

        # Set the Ground Truth Map
        self.map_gt = OccupancyGrid()
        #self.map.header.stamp = rospy.get_time()
        self.map_gt.header.frame_id = "odom"
        # Resolution in meters
        self.map_gt.info.resolution = 0.1
        # Able to account for area of 5000m^2 + 20% for Automower 450X
        self.map_gt.info.width  = int( 5 * 2 / self.map_gt.info.resolution )
        self.map_gt.info.height = int( 5 * 2 / self.map_gt.info.resolution )
        # Set the map as horizontal and at the center of the robot
        self.map_gt.info.origin.position.x = - self.map_gt.info.width / 2 * self.map_gt.info.resolution
        self.map_gt.info.origin.position.y = - self.map_gt.info.height / 2 * self.map_gt.info.resolution
        self.map_gt.info.origin.position.z = 0.0
        self.map_gt.info.origin.orientation.x = 0.0
        self.map_gt.info.origin.orientation.y = 0.0
        self.map_gt.info.origin.orientation.z = 0.0
        self.map_gt.info.origin.orientation.w = 0.0
        # Initialise the occupancy grid of the ground truth
        # Set the map at 50 with borders at -1
        self.map_gt.data = []
        for i in range(0, self.map_gt.info.width*self.map_gt.info.height):
            if((i%self.map_gt.info.width == 0) or ((i + 1)%self.map_gt.info.width == 0)):
                self.map_gt.data.append(-1)
            elif((i < self.map_gt.info.width) or (i > (self.map_gt.info.height-1)*self.map_gt.info.width)):
                self.map_gt.data.append(-1)
            else:
                self.map_gt.data.append(50)

        # Define set of topics to publish
        if(self.test and self.ros):
            # AEKF5 Map
            self.map_pub = rospy.Publisher('CollisionMap', OccupancyGrid, latch=True, queue_size=20)
            self.map_pub.publish(self.map)
            # Ground Truth Map
            self.map_gt_pub = rospy.Publisher('CollisionMap_GT', OccupancyGrid, latch=True, queue_size=20)
            self.map_gt_pub.publish(self.map_gt)

        print("Initialised CollisionOccupancyMap")

    def Sensor(self, sensor_status):
        if(sensor_status.sensorStatus == 4):
            if(self.test and self.print):
                print("Collision: ON")
            self.collision = True
        else:
            if(self.test and self.print):
                print("Collision: OFF")
            self.collision = False

        if(self.test and self.print):
            print("Sensor")

    def GetMode(self, mode):
        if(int(mode.data) == 146):
            if(self.test and self.print):
                print("Boundary: ON")
            self.boundary = True
        if(int(mode.data) == 147):
            if(self.test and self.print):
                print("Boundary: OFF")
            self.boundary = False
        # Collision injection swith
        if(int(mode.data) == 275):
            if(self.test and self.print):
                print("Collision: OFF")
            self.collision = False
        if(int(mode.data) == 276):
            if(self.test and self.print):
                print("Collision: ON")
            self.collision = True

        if(self.test and self.print):
            print("Mode")

    def GroundTruth(self, Odom_Ground):

        # Store the robot's pose
        self.X_g[0] = Odom_Ground.pose.pose.position.x
        self.X_g[1] = Odom_Ground.pose.pose.position.y

        euler = tf.transformations.euler_from_quaternion([Odom_Ground.pose.pose.orientation.x, Odom_Ground.pose.pose.orientation.y, Odom_Ground.pose.pose.orientation.z, Odom_Ground.pose.pose.orientation.w])
        self.X_g[2] = euler[2]


    def EKF(self, Odom_Full_EKF):

        # Store the robot's pose
        self.X_t[0] = Odom_Full_EKF.pose.pose.position.x
        self.X_t[1] = Odom_Full_EKF.pose.pose.position.y

        euler = tf.transformations.euler_from_quaternion([Odom_Full_EKF.pose.pose.orientation.x, Odom_Full_EKF.pose.pose.orientation.y, Odom_Full_EKF.pose.pose.orientation.z, Odom_Full_EKF.pose.pose.orientation.w])
        self.X_t[2] = euler[2]

        # Store the robot's pose variance
        self.P_t[0] = Odom_Full_EKF.pose.covariance[0]
        self.P_t[1] = Odom_Full_EKF.pose.covariance[7]
        self.P_t[2] = Odom_Full_EKF.pose.covariance[35]


    def UpdateMap(self):
        if(self.boundary): # Boundary setting
            # Pose
            self.updateCell(self.X_t[0], self.X_t[1], 0, False)
            # Iterate over Y axis (symmetric)
            for i in range(0, int( - self.Y_size[0] / self.map.info.resolution)):
                for j in range(0, int( - self.X_size[0] / self.map.info.resolution)): # Iterate over X axis on the back
                    self.updateCell(self.X_t[0] - j * math.cos(self.X_t[2]) * self.map.info.resolution + i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] - j * math.sin(self.X_t[2]) * self.map.info.resolution - i * math.cos(self.X_t[2]) * self.map.info.resolution, 0, False )
                    self.updateCell(self.X_t[0] - j * math.cos(self.X_t[2]) * self.map.info.resolution - i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] - j * math.sin(self.X_t[2]) * self.map.info.resolution + i * math.cos(self.X_t[2]) * self.map.info.resolution, 0, False )
                for j in range(0, int(self.X_size[1] / self.map.info.resolution)): # Iterate over X axis in front
                    self.updateCell(self.X_t[0] + j * math.cos(self.X_t[2]) * self.map.info.resolution + i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] + j * math.sin(self.X_t[2]) * self.map.info.resolution - i * math.cos(self.X_t[2]) * self.map.info.resolution, 0, False )
                    self.updateCell(self.X_t[0] + j * math.cos(self.X_t[2]) * self.map.info.resolution - i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] + j * math.sin(self.X_t[2]) * self.map.info.resolution + i * math.cos(self.X_t[2]) * self.map.info.resolution, 0, False )

            if(self.test and self.print):
                print("Boundary")

        else: # Collision detection
            # Pose
            self.updateCell(self.X_t[0], self.X_t[1], -2, False) # minus to remove the overlapping of the following cycles
            # Iterate over Y axis (symmetric)
            for i in range(0, int( - self.Y_size[0] / self.map.info.resolution)):
                for j in range(0, int( - self.X_size[0] / self.map.info.resolution)): # Iterate over X axis on the back
                    self.updateCell(self.X_t[0] - j * math.cos(self.X_t[2]) * self.map.info.resolution + i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] - j * math.sin(self.X_t[2]) * self.map.info.resolution - i * math.cos(self.X_t[2]) * self.map.info.resolution, 2, False )
                    self.updateCell(self.X_t[0] - j * math.cos(self.X_t[2]) * self.map.info.resolution - i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] - j * math.sin(self.X_t[2]) * self.map.info.resolution + i * math.cos(self.X_t[2]) * self.map.info.resolution, 2, False )
                for j in range(0, int(self.X_size[1] / self.map.info.resolution)): # Iterate over X axis in front
                    self.updateCell(self.X_t[0] + j * math.cos(self.X_t[2]) * self.map.info.resolution + i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] + j * math.sin(self.X_t[2]) * self.map.info.resolution - i * math.cos(self.X_t[2]) * self.map.info.resolution, 2, False )
                    self.updateCell(self.X_t[0] + j * math.cos(self.X_t[2]) * self.map.info.resolution - i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] + j * math.sin(self.X_t[2]) * self.map.info.resolution + i * math.cos(self.X_t[2]) * self.map.info.resolution, 2, False )
                # Front to update in case of collision
                self.updateCell(self.X_t[0] + self.X_size[1] * math.cos(self.X_t[2]) - i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] + self.X_size[1] * math.sin(self.X_t[2]) + i * math.cos(self.X_t[2]) * self.map.info.resolution, 5, True )
                self.updateCell(self.X_t[0] + self.X_size[1] * math.cos(self.X_t[2]) + i * math.sin(self.X_t[2]) * self.map.info.resolution , self.X_t[1] + self.X_size[1] * math.sin(self.X_t[2]) - i * math.cos(self.X_t[2]) * self.map.info.resolution, 5, True )


        # Define set of topics to publish
        if(self.test and self.ros):
            self.map_pub.publish(self.map)

        if(self.test and self.print):
            print("Update")

    # Update the probability of a cell using Bayes Theorem
    def updateCell(self, x, y, weight, front):
        cell = int ( self.map.info.width * int( self.map.info.height/2 + y / self.map.info.resolution ) + int( self.map.info.width/2 + x / self.map.info.resolution ) )

        # Don't update cells at -1
        if ( self.map.data[int(cell)] == -1 ) :
            return

        if(weight == 0):
            self.map.data[int(cell)] = -1
            return

        if(self.collision and front):
            if(self.test and self.print):
                print("occupied")
            # Update with a value of 50 + weight in case of Collision
            event = 50 + weight
        else:
            if(self.test and self.print):
                print("empty")
            # Update with a value of 50 - weight in case of no Collision
            event = 50 - weight
        # Update the knowledge using the formula * 99 + 1 (to get values from 1 to 100)
        self.map.data[int(cell)] = int( (self.map.data[int(cell)] * event ) / ( ( self.map.data[int(cell)] * event ) + ( ( 100 - self.map.data[int(cell)] ) * ( 100 - event ) ) ) * 99 + 1 )



    def UpdateMap_GT(self):
        if(self.boundary): # Boundary setting
            # Pose
            self.updateCell_GT(self.X_g[0], self.X_g[1], 0, False)
            # Iterate over Y axis (symmetric)
            for i in range(0, int( - self.Y_size[0] / self.map_gt.info.resolution)):
                for j in range(0, int( - self.X_size[0] / self.map_gt.info.resolution)): # Iterate over X axis on the back
                    self.updateCell_GT(self.X_g[0] - j * math.cos(self.X_g[2]) * self.map_gt.info.resolution + i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] - j * math.sin(self.X_g[2]) * self.map_gt.info.resolution - i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 0, False )
                    self.updateCell_GT(self.X_g[0] - j * math.cos(self.X_g[2]) * self.map_gt.info.resolution - i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] - j * math.sin(self.X_g[2]) * self.map_gt.info.resolution + i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 0, False )
                for j in range(0, int(self.X_size[1] / self.map_gt.info.resolution)): # Iterate over X axis in front
                    self.updateCell_GT(self.X_g[0] + j * math.cos(self.X_g[2]) * self.map_gt.info.resolution + i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] + j * math.sin(self.X_g[2]) * self.map_gt.info.resolution - i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 0, False )
                    self.updateCell_GT(self.X_g[0] + j * math.cos(self.X_g[2]) * self.map_gt.info.resolution - i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] + j * math.sin(self.X_g[2]) * self.map_gt.info.resolution + i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 0, False )

            if(self.test and self.print):
                print("Boundary")

        else: # Collision detection
            # Pose
            self.updateCell_GT(self.X_g[0], self.X_g[1], -2, False) # minus to remove the overlapping of the following cycles
            # Iterate over Y axis (symmetric)
            for i in range(0, int( - self.Y_size[0] / self.map_gt.info.resolution)):
                for j in range(0, int( - self.X_size[0] / self.map_gt.info.resolution)): # Iterate over X axis on the back
                    self.updateCell_GT(self.X_g[0] - j * math.cos(self.X_g[2]) * self.map_gt.info.resolution + i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] - j * math.sin(self.X_g[2]) * self.map_gt.info.resolution - i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 2, False )
                    self.updateCell_GT(self.X_g[0] - j * math.cos(self.X_g[2]) * self.map_gt.info.resolution - i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] - j * math.sin(self.X_g[2]) * self.map_gt.info.resolution + i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 2, False )
                for j in range(0, int(self.X_size[1] / self.map_gt.info.resolution)): # Iterate over X axis in front
                    self.updateCell_GT(self.X_g[0] + j * math.cos(self.X_g[2]) * self.map_gt.info.resolution + i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] + j * math.sin(self.X_g[2]) * self.map_gt.info.resolution - i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 2, False )
                    self.updateCell_GT(self.X_g[0] + j * math.cos(self.X_g[2]) * self.map_gt.info.resolution - i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] + j * math.sin(self.X_g[2]) * self.map_gt.info.resolution + i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 2, False )
                # Front to update in case of collision
                self.updateCell_GT(self.X_g[0] + self.X_size[1] * math.cos(self.X_g[2]) - i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] + self.X_size[1] * math.sin(self.X_g[2]) + i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 5, True )
                self.updateCell_GT(self.X_g[0] + self.X_size[1] * math.cos(self.X_g[2]) + i * math.sin(self.X_g[2]) * self.map_gt.info.resolution , self.X_g[1] + self.X_size[1] * math.sin(self.X_g[2]) - i * math.cos(self.X_g[2]) * self.map_gt.info.resolution, 5, True )


        # Define set of topics to publish
        if(self.test and self.ros):
            self.map_gt_pub.publish(self.map_gt)

        if(self.test and self.print):
            print("Update")

    # Update the probability of a cell using Bayes Theorem
    def updateCell_GT(self, x, y, weight, front):
        cell = int ( self.map_gt.info.width * int( self.map_gt.info.height/2 + y / self.map_gt.info.resolution ) + int( self.map_gt.info.width/2 + x / self.map_gt.info.resolution ) )

        # Don't update cells at -1
        if ( self.map_gt.data[int(cell)] == -1 ) :
            return

        if(weight == 0):
            self.map_gt.data[int(cell)] = -1
            return

        if(self.collision and front):
            if(self.test and self.print):
                print("occupied")
            # Update with a value of 50 + weight in case of Collision
            event = 50 + weight
        else:
            if(self.test and self.print):
                print("empty")
            # Update with a value of 50 - weight in case of no Collision
            event = 50 - weight
        # Update the knowledge using the formula * 99 + 1 (to get values from 1 to 100)
        self.map_gt.data[int(cell)] = int( (self.map_gt.data[int(cell)] * event ) / ( ( self.map_gt.data[int(cell)] * event ) + ( ( 100 - self.map_gt.data[int(cell)] ) * ( 100 - event ) ) ) * 99 + 1 )



    def groundTruthCheck(self, section, error):

        MAP = MAP_GT = np.array([])

        if(section == "global"):
            MAP = np.array(self.map.data).flatten()
            MAP_GT = np.array(self.map_gt.data).flatten()

        if(section == "boundary"):
            count = 0
            for cell in self.map.data:
                if ( self.map.data[int(count)] == -1 or self.map_gt.data[int(count)] == -1):
                    MAP = np.append(MAP, self.map.data[int(count)])
                    MAP_GT = np.append(MAP_GT, self.map_gt.data[int(count)])
                count += 1


        if(section == "collision"):
            count = 0
            for cell in self.map.data:
                if ( (self.map.data[int(count)] < self.maxObject and self.map.data[int(count)] > -1) or (self.map_gt.data[int(count)] < self.maxObject  and self.map.data[int(count)] > -1) ):
                    MAP = np.append(MAP, self.map.data[int(count)])
                    MAP_GT = np.append(MAP_GT, self.map_gt.data[int(count)])
                count += 1

        Error = 0.0
        if(len(MAP) and len(MAP_GT)):
            if(error == "RMSE"):
                MSE = np.square(np.subtract(MAP_GT, MAP)).mean()
                Error = math.sqrt(MSE)
            if(error == "MAE"):
                Error = np.absolute(np.subtract(MAP_GT, MAP)).mean()
            if(error == "Baron"):
                Error = self.BaronCrossCorr(MAP_GT, MAP)
            if(error == "Pearson"):
                Error = self.PearsonCorr(MAP_GT, MAP)
        return Error

    def BaronCrossCorr(self, MAP_GT, MAP):

        MapxMapGT = np.multiply(MAP, MAP_GT)

        MeanMap = np.mean(MAP)
        MeanMapGT = np.mean(MAP_GT)
        MeanMapxMapGT = np.mean(MapxMapGT)

        StdMap = np.std(MAP)
        StdMapGT = np.std(MAP_GT)

        Baron = ( MeanMapxMapGT + MeanMap * MeanMapGT ) / ( StdMap * StdMapGT )

        return Baron

    def PearsonCorr(self, MAP_GT, MAP):


        MeanMap = np.mean(MAP)
        MeanMapGT = np.mean(MAP_GT)

        num = np.sum( (MAP - MeanMap) * (MAP_GT - MeanMapGT) )

        den = np.sqrt( np.sum( np.square( MAP - MeanMap) ) ) * np.sqrt( np.sum( np.square( MAP_GT - MeanMapGT ) ) )

        Pearson = num / den

        return Pearson


    def groundTruthF1(self, section):

        TP = 0
        TN = 0
        FP = 0
        FN = 0

        if(section == "boundary"):
            count = 0
            for cell in self.map.data:
                if ( self.map.data[int(count)] == -1):
                    if ( self.map_gt.data[int(count)] == -1 ):
                        TP += 1
                    else:
                        FP += 1
                else:
                    if ( self.map_gt.data[int(count)] == -1 ):
                        TN += 1
                    else:
                        FN += 1
                count += 1


        if(section == "collision"):
            count = 0
            for cell in self.map.data:
                if ( (self.map.data[int(count)] < self.maxObject and self.map.data[int(count)] > -1)):
                    if ( (self.map_gt.data[int(count)] < self.maxObject  and self.map.data[int(count)] > -1) ):
                        TP += 1
                    else:
                        FP += 1
                else:
                    if ( (self.map_gt.data[int(count)] < self.maxObject  and self.map.data[int(count)] > -1) ):
                        TN += 1
                    else:
                        FN += 1
                count += 1

        Precision = TP / ( TP + FP )
        Recall = TP / ( TP + FN )
        print("TP: " + str(TP) + " - TN: " + str(TN) + " - FP: " + str(FP) + " - FN: " + str(FN) + " - Precision: " + str(Precision) + " - Recall: " + str(Recall))
        F1 = 2 * ( Precision * Recall ) / ( Precision + Recall )
        return F1



if __name__ == '__main__':

    print("Start CollisionOccupancyMap")

    # Initialise the CollisionOccupancyMap
    boundary = True
    collision = CollisionOccupancyMap(boundary)

    try:
        # Continuosly try to get updates and run Prediction and Update (Constant dt)
        hertz = 5 # Update frequency [Hz]
        rate = rospy.Rate(hertz)

        #rospy.spin()
        while not rospy.is_shutdown():
            # Update the map with the desired rate
            collision.UpdateMap()
            collision.UpdateMap_GT()
            rate.sleep()

    except rospy.ROSInterruptException:
        # Wait for the user input to terminate the program
        input("Press any key to terminate the program\n")
        pass

    print("RMSE Global: " + str(collision.groundTruthCheck("global", "RMSE")))
    print("RMSE Boundary: " + str(collision.groundTruthCheck("boundary", "RMSE")))
    print("RMSE Collisions: " + str(collision.groundTruthCheck("collision", "RMSE")))

    print("MAE Global: " + str(collision.groundTruthCheck("global", "MAE")))
    print("MAE Boundary: " + str(collision.groundTruthCheck("boundary", "MAE")))
    print("MAE Collisions: " + str(collision.groundTruthCheck("collision", "MAE")))

    print("Baron Global: " + str(collision.groundTruthCheck("global", "Baron")))
    print("Baron Boundary: " + str(collision.groundTruthCheck("boundary", "Baron")))
    print("Baron Collisions: " + str(collision.groundTruthCheck("collision", "Baron")))

    print("Pearson Global: " + str(collision.groundTruthCheck("global", "Pearson")))
    print("Pearson Boundary: " + str(collision.groundTruthCheck("boundary", "Pearson")))
    print("Pearson Collisions: " + str(collision.groundTruthCheck("collision", "Pearson")))

    print("F1 Boundary: " + str(collision.groundTruthF1("boundary")))
    print("F1 Collisions: " + str(collision.groundTruthF1("collision")))


    # Print bye message
    print("THE END")
