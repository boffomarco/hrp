#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import NavSatFix

import pymap3d as pm

class PoseCheckerGPS():

    def __init__(self):
        print("Initialising PoseCheckerGPS")

        rospy.init_node("PoseCheckerGPS", anonymous=True)

        self.pose_gps_Check_pub = rospy.Publisher('PoseCheckGPS', Pose, queue_size=20)
        self.pose_enu_Check_pub = rospy.Publisher('PoseCheckENU', Pose, queue_size=20)

        rospy.Subscriber('GPSfix', NavSatFix, self.NavSat2PoseGPS)
        rospy.Subscriber('GPSfix', NavSatFix, self.NavSat2PoseENU)

        self.pose_broadcaster = tf.TransformBroadcaster()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.lat_mean = 59.398410
        self.long_mean = 17.952600


    def NavSat2PoseGPS(self, GPSfix):

        self.current_time = GPSfix.header.stamp

        # print("Status: " + str(GPSfix.status.status) + " - Service: " + str(GPSfix.status.service) ) 5 - 1

        self.x = ( GPSfix.latitude - self.lat_mean ) * 40008000 / 360
        self.y = ( ( GPSfix.longitude - self.long_mean ) * 40075160 / 360 ) * math.cos(GPSfix.latitude)

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.pose_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # publish the message
        self.pose_gps_Check_pub.publish(pose)

        last_time = self.current_time



    def NavSat2PoseENU(self, GPSfix):

        self.current_time = GPSfix.header.stamp

        # print("Status: " + str(GPSfix.status.status) + " - Service: " + str(GPSfix.status.service) ) 5 - 1

        gps_e , gps_n, gps_u = pm.geodetic2enu(GPSfix.latitude,GPSfix.longitude,0,self.lat_mean,self.long_mean,0)

        self.z_x = - gps_e
        self.z_y = gps_n

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.pose_broadcaster.sendTransform(
            (self.z_x, self.z_x, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the pose message over ROS
        pose = Pose(Point(self.z_x, self.z_y, 0.), Quaternion(*odom_quat))

        # publish the message
        self.pose_enu_Check_pub.publish(pose)

        last_time = self.current_time



if __name__ == '__main__':

    try:

        poseCheckerGPS = PoseCheckerGPS()

        rospy.spin()
        #odomCheck.run()

    except rospy.ROSInterruptException:
        pass
