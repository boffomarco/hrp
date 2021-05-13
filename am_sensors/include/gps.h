#ifndef AM_SENSORS_GPS_H
#define AM_SENSORS_GPS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "phidget22.h"




    void CCONV positionChangeHandler(PhidgetGPSHandle ch, void *ctx, double latitude, double longitude, double altitude);

    void CCONV headingChangeHandler(PhidgetGPSHandle ch, void *ctx, double heading, double velocity);

#endif // AM_SENSORS_GPS_H

