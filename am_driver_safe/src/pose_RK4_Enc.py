#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from am_driver.msg import WheelEncoder

class PoseEncCheckRK4():

    def __init__(self):
        print("Initialising PoseEncCheckRK4")

        rospy.init_node("PoseEncCheckRK4", anonymous=True)

        self.pose_enc_Check_pub = rospy.Publisher('PoseEncCheckRK4', Pose, queue_size=20)

        rospy.Subscriber('wheel_encoder', WheelEncoder, self.Enc2PoseRK4)

        self.pose_broadcaster = tf.TransformBroadcaster()

        self.xEnc = 0
        self.yEnc = 0
        self.thEnc = 0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.leftPulses = 0
        self.lastLeftPulses = 0

        self.rightPulses = 0
        self.lastRightPulses = 0

        # Automower parameters
        self.base_width = 0.464500
        self.wheel_diameter = 0.245
        self.wheel_pulses_per_turn = 349
        self.wheel_meter_per_tick = (2.0 * math.pi * self.wheel_diameter / 2.0) / self.wheel_pulses_per_turn

    def Enc2PoseRK4(self, wheel_encoder): # https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html

        self.current_time = wheel_encoder.header.stamp
        dt = (self.current_time - self.last_time).to_sec()

        self.leftPulses = wheel_encoder.rwheelAccum
        self.rightPulses = wheel_encoder.lwheelAccum

        if(self.lastLeftPulses and self.lastRightPulses):
            deltaLeftPulses = self.leftPulses - self.lastLeftPulses
            deltaRightPulses = self.rightPulses - self.lastRightPulses

            leftDist  = - deltaLeftPulses * self.wheel_meter_per_tick
            rightDist = deltaRightPulses * self.wheel_meter_per_tick

            delta_d = ( rightDist + leftDist ) / 2

            delta_th = ( rightDist - leftDist ) / self.base_width

            k_00 = delta_d * math.cos( self.thEnc )
            k_01 = delta_d * math.sin( self.thEnc )

            k_10 = delta_d * math.cos( self.thEnc + ( delta_th / 2 ) )
            k_11 = delta_d * math.sin( self.thEnc + ( delta_th / 2 ))

            k_20 = delta_d * math.cos( self.thEnc + ( delta_th / 2 ))
            k_21 = delta_d * math.sin( self.thEnc + ( delta_th / 2 ))

            k_30 = delta_d * math.cos( self.thEnc + ( delta_th ))
            k_31 = delta_d * math.sin( self.thEnc + ( delta_th ))

            delta_x = 1/6 * ( k_00 + 2 * ( k_10 + k_20 ) + k_30 )
            delta_y = 1/6 * ( k_01 + 2 * ( k_11 + k_21 ) + k_31 )

            self.xEnc -= delta_x
            self.yEnc -= delta_y
            self.thEnc += delta_th

            #print(" x " + str(self.xEnc) + " - y " + str(self.yEnc) + " - yaw " + str(self.thEnc))

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.thEnc * 360 / math.pi )

            # first, we'll publish the transform over tf
            self.pose_broadcaster.sendTransform(
                (self.xEnc, self.yEnc, 0.),
                odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the pose message over ROS
            poseEnc = Pose(Point(self.xEnc, self.yEnc , 0.), Quaternion(*odom_quat))

            # publish the message
            self.pose_enc_Check_pub.publish(poseEnc)

        # Store prev values
        self.last_time = self.current_time
        self.lastLeftPulses = self.leftPulses
        self.lastRightPulses = self.rightPulses





if __name__ == '__main__':

    try:

        poseEncCheckRK4 = PoseEncCheckRK4()

        rospy.spin()
        #odomCheck.run()

    except rospy.ROSInterruptException:
        pass
