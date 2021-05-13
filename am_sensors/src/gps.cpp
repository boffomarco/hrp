#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "phidget22.h"
#include "gps.h"

void CCONV positionChangeHandler(PhidgetGPSHandle ch, void *ctx, double latitude, double longitude, double altitude) {

    ros::Publisher* nmea_pub_fix = (ros::Publisher*)ctx;

    sensor_msgs::NavSatFixPtr fix(new sensor_msgs::NavSatFix);

    fix->header.stamp = ros::Time::now();
    fix->header.frame_id = "odom";

    fix->status.status = 0;
    fix->status.service = 1;

    fix->latitude = latitude;
    fix->longitude = longitude;
    fix->altitude = altitude;

    for (int i = 0; i < 9; ++i) {
        fix->position_covariance[i] = 0;
    }
    fix->position_covariance_type = 1;

	PhidgetGPS_NMEAData NMEAData;

	PhidgetReturnCode res = PhidgetGPS_getNMEAData(ch, &NMEAData);
	if (res == EPHIDGET_OK){

        fix->position_covariance[0] = NMEAData.GSA.mode;
        fix->position_covariance[1] = NMEAData.GSA.horizDilution;
        fix->position_covariance[2] = NMEAData.GSA.fixType;
        fix->position_covariance[3] = NMEAData.GSA.posnDilution;
        //fix->position_covariance[4] = NMEAData.GSA.satUsed;
        fix->position_covariance[5] = NMEAData.GSA.vertDilution;
        fix->position_covariance[6] = NMEAData.GGA.numSatellites;
        fix->position_covariance[7] = NMEAData.GGA.horizontalDilution;
        fix->position_covariance[8] = NMEAData.GGA.heightOfGeoid;

    }

    (*nmea_pub_fix).publish(fix);
}


void CCONV headingChangeHandler(PhidgetGPSHandle ch, void *ctx, double heading, double velocity) {

    ros::Publisher* nmea_pub_hv = (ros::Publisher*)ctx;

    sensor_msgs::NavSatFixPtr hv(new sensor_msgs::NavSatFix);

    hv->header.stamp = ros::Time::now();
    hv->header.frame_id = "odom";

    hv->status.status = 0;
    hv->status.service = 1;

    for (int i = 0; i < 9; ++i) {
        hv->position_covariance[i] = 0;
    }
    hv->position_covariance_type = 1;


	PhidgetGPS_NMEAData NMEAData;

	PhidgetReturnCode res = PhidgetGPS_getNMEAData(ch, &NMEAData);
	if (res == EPHIDGET_OK){

        hv->latitude = NMEAData.GGA.latitude;
        hv->longitude = NMEAData.GGA.longitude;
        hv->altitude = NMEAData.GGA.altitude;

        hv->position_covariance[0] = NMEAData.RMC.heading;
        hv->position_covariance[1] = NMEAData.RMC.magneticVariation;
        hv->position_covariance[2] = NMEAData.RMC.mode;
        hv->position_covariance[3] = NMEAData.RMC.status;
        hv->position_covariance[4] = NMEAData.VTG.magneticHeading;
        hv->position_covariance[5] = NMEAData.VTG.mode;
        hv->position_covariance[6] = NMEAData.VTG.speed;
        hv->position_covariance[7] = NMEAData.VTG.speedKnots;
        hv->position_covariance[8] = NMEAData.VTG.trueHeading;

    }

    (* nmea_pub_hv).publish(hv);

}


int main(int argc, char** argv){

    ROS_INFO("Start Phidgets GPS ");
	ros::init(argc, argv, "GPS");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

    ros::Publisher nmea_pub_fix = nh.advertise<sensor_msgs::NavSatFix>("NMEA_fix", 200);
    ros::Publisher nmea_pub_hv  = nh.advertise<sensor_msgs::NavSatFix>("NMEA_hv", 200);

    int device_serial_number;
	if (!nh_private.getParam("device_serial_number", device_serial_number))
		device_serial_number = 0;

	PhidgetReturnCode res;
	PhidgetGPSHandle ch;

	PhidgetGPS_create(&ch);

	res = Phidget_setDeviceSerialNumber((PhidgetHandle)ch, device_serial_number);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error


	res = PhidgetGPS_setOnPositionChangeHandler(ch, positionChangeHandler, &nmea_pub_fix);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

    res = PhidgetGPS_setOnHeadingChangeHandler(ch, headingChangeHandler, &nmea_pub_hv);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error


	res = Phidget_openWaitForAttachment((PhidgetHandle)ch, PHIDGET_TIMEOUT_DEFAULT);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

    ROS_INFO("Run GPS");

	ros::spin(); // Do work, wait for events, etc.

	PhidgetGPS_delete(&ch);

	return 0;
}
