#!/usr/bin/python3

import time
import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from sensor_msgs.msg import NavSatFix

from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.GPS import *


# Start GPS node
rospy.init_node("gps_pub", anonymous=True,  log_level=rospy.DEBUG)

# ROS elements
global_pose_pub = rospy.Publisher("fix", NavSatFix, queue_size=10)
global_hv_pub = rospy.Publisher("hv", Vector3, queue_size=10)


def onPositionChange(self, latitude, longitude, altitude):
	fix = NavSatFix()
	fix.header.stamp = rospy.Time.now()
	fix.header.frame_id = "odom"
	fix.latitude = latitude
	fix.longitude = longitude
	fix.altitude = altitude
	global_pose_pub.publish(fix)

def onHeadingChange(self, heading, velocity):
	hv = Vector3()
	hv.x = heading
	hv.y = velocity
	global_hv_pub.publish(hv)

def onError(self, code, description):
	rospy.logerr("Code: %s", ErrorEventCode.getName(code))
	rospy.logerr("Description: %s", str(description))
	rospy.logerr("----------")


def gps_pub_node():

	# Fetch the parameters
	device_serial_number = rospy.get_param('~device_serial_number')

	# Create Phidget channels
	gps = GPS()

	# Set addressing parameters to specify which channel to open
	gps.setDeviceSerialNumber(device_serial_number)

	# Assign any event handlers you need before calling open so that no events are missed.
	gps.setOnPositionChangeHandler(onPositionChange)
	gps.setOnHeadingChangeHandler(onHeadingChange)

	gps.setOnErrorHandler(onError)

	# Open your Phidgets and wait for attachment
	gps.open()

	print("Initialised GPS " + str(device_serial_number))

	rospy.spin()

	# Close your Phidgets once the program is done.
	gps.close()


if __name__ == '__main__':

	print("Start Phidgets GPS Node")

	try:

		gps_pub_node()

	except rospy.ROSInterruptException:

		pass


	print("End Phidgets GPS Node")

