#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from am_driver.msg import WheelEncoder

class PoseCheckerExact():

    def __init__(self):
        print("Initialising PoseCheckerExact")

        rospy.init_node("PoseCheckerExact", anonymous=True)

        self.pose_Check_pub = rospy.Publisher('PoseCheckExact', Pose, queue_size=20)

        rospy.Subscriber('wheel_encoder', WheelEncoder, self.Enc2PoseExact)

        self.pose_broadcaster = tf.TransformBroadcaster()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()


    def Enc2PoseExact(self, wheel_encoder): # http://www.diag.uniroma1.it//~oriolo/amr/slides/Localization1_Slides.pdf
        self.current_time = wheel_encoder.header.stamp

        delta_d  = ( wheel_encoder.rwheel + wheel_encoder.lwheel ) / 2

        delta_th = ( wheel_encoder.rwheel - wheel_encoder.lwheel ) / 0.4645

        if(delta_th):
            delta_x = delta_d / delta_th * ( math.sin( self.th + delta_th ) - math.sin( self.th ) )
            delta_y = - ( delta_d / delta_th * ( math.cos( self.th + delta_th ) - math.cos( self.th ) ) )
        else:
            delta_x = delta_d / 0.000001 * ( math.sin( self.th + delta_th ) - math.sin( self.th ) )
            delta_y = - ( delta_d / 0.000001 * ( math.cos( self.th + delta_th ) - math.cos( self.th ) ) )

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

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
        self.pose_Check_pub.publish(pose)

        self.last_time = self.current_time



if __name__ == '__main__':

    try:

        poseCheckerExact = PoseCheckerExact()

        rospy.spin()
        #odomCheck.run()

    except rospy.ROSInterruptException:
        pass
