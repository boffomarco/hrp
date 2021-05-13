#ifndef AM_SENSORS_GPS_PUB_H
#define AM_SENSORS_GPS_PUB_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "phidget22.h"

namespace am_sensors_gps{

    class GPS_pub
    {
        public:
            GPS_pub(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
            virtual ~GPS_pub();

        private:

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher gps_pub_fix;
        ros::Publisher nmea_pub_fix;
        ros::Publisher gps_pub_hv;


        PhidgetReturnCode res;
        PhidgetGPSHandle ch;

        bool init();

        void CCONV positionChangeHandler(PhidgetGPSHandle ch, void *ctx, double latitude, double longitude, double altitude);

        void CCONV headingChangeHandler(PhidgetGPSHandle ch, void *ctx, double heading, double velocity);

    };

} // namespace am_sensors_gps

#endif // AM_SENSORS_GPS_PROCESS_H

