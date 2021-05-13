#include <stdlib.h>
#include "phidget22.h"

#include "gps_pub.h"


namespace am_sensors_gps{

    GPS_pub::GPS_pub(
        const ros::NodeHandle& nh,
        const ros::NodeHandle& nh_private):
        nh_(nh),
        nh_private_(nh_private)
        {
            ROS_INFO("Starting GPS_pub.");

            if(init())
                ROS_INFO("Finished initialization of GPS_pub");
            else
                ROS_INFO("Error during initialization of GPS_pub");
        }

    GPS_pub::~GPS_pub()
    {
	    //PhidgetGPS_delete(&ch);
        ROS_INFO("Destorying GPS_pub.");

    }

    bool GPS_pub::init(){
        gps_pub_fix  = nh_.advertise<sensor_msgs::NavSatFix>("GPS_fix", 200);
        nmea_pub_fix = nh_.advertise<sensor_msgs::NavSatFix>("NMEA_fix", 200);
        gps_pub_fix  = nh_.advertise<geometry_msgs::Vector3>("NMEA_hv", 200);

        //PhidgetGPS_create(&ch);

        bool ret = true;
        /*
        try
        {
            if(ret){
                res = PhidgetGPS_setOnPositionChangeHandler(ch, &GPS_pub::positionChangeHandler, NULL);
                if (res != EPHIDGET_OK)
                    ret = false; // Exit in error
            }

            if(ret){
                res = PhidgetGPS_setOnHeadingChangeHandler(ch, &GPS_pub::headingChangeHandler, NULL);
                if (res != EPHIDGET_OK)
                    ret = false; // Exit in error
            }

            if(ret){
                res = Phidget_open((PhidgetHandle)ch);
                if (res != EPHIDGET_OK)
                    ret = false; // Exit in error
            }
        }
        catch (const Phidget22Error &err)
        {
            ROS_ERROR("Spatial: %s", err.what());
            throw;
        }*/

        return ret;

    }



    void CCONV GPS_pub::positionChangeHandler(PhidgetGPSHandle ch, void *ctx, double latitude, double longitude, double altitude) {

        PhidgetReturnCode res;
        PhidgetGPS_NMEAData NMEAData;

        ros::Time now = ros::Time::now();

        //printf("Latidute: \t\t%lf \n", latitude);
        //printf("Longitude: \t\t%lf \n", longitude);
        //printf("Altitude: \t\t%lf \n", altitude);
        /*
        res = PhidgetGPS_getNMEAData(ch, &NMEAData);
        if (res == EPHIDGET_OK){
            printf("GGA-------\n");
            printf("Latidute: \t\t%lf \n", NMEAData.GGA.latitude);
            printf("Longitude: \t\t%lf \n", NMEAData.GGA.longitude);
            printf("Altitude: \t\t%lf \n", NMEAData.GGA.altitude);
            printf("numSatellites: \t\t%d \n", NMEAData.GGA.numSatellites);
            printf("horizontalDilution: \t%lf \n", NMEAData.GGA.horizontalDilution);
            printf("heightOfGeoid: \t\t%lf \n", NMEAData.GGA.heightOfGeoid);
            printf("GSA-------\n");
            printf("mode: \t\t\t%d \n", NMEAData.GSA.mode);
            printf("horizDilution: \t\t%lf \n", NMEAData.GSA.horizDilution);
            printf("fixType: \t\t%d \n", NMEAData.GSA.fixType);
            printf("posnDilution: \t\t%lf \n", NMEAData.GSA.posnDilution);
            printf("satUsed: \t\t%d \n", NMEAData.GSA.satUsed);
            printf("vertDilution: \t\t%lf \n", NMEAData.GSA.vertDilution);
            printf("RMC-------\n");
            printf("heading: \t\t%lf \n", NMEAData.RMC.heading);
            printf("latitude: \t\t%lf \n", NMEAData.RMC.latitude);
            printf("longitude: \t\t%lf \n", NMEAData.RMC.longitude);
            printf("magneticVariation: \t%lf \n", NMEAData.RMC.magneticVariation);
            printf("mode: \t\t\t%d \n", NMEAData.RMC.mode);
            printf("speedKnots: \t\t%lf \n", NMEAData.RMC.speedKnots);
            printf("status: \t\t%d \n", NMEAData.RMC.status);
            printf("VTG-------\n");
            printf("magneticHeading: \t%lf \n", NMEAData.VTG.magneticHeading);
            printf("mode: \t\t\t%d \n", NMEAData.VTG.mode);
            printf("speed: \t\t\t%lf \n", NMEAData.VTG.speed);
            printf("speedKnots: \t\t%lf \n", NMEAData.VTG.speedKnots);
            printf("trueHeading: \t\t%lf \n", NMEAData.VTG.trueHeading);
            printf("----------\n");
        }*/
        sleep(1);
    }

    void CCONV GPS_pub::headingChangeHandler(PhidgetGPSHandle ch, void *ctx, double heading, double velocity) {


        ros::Time now = ros::Time::now();

        PhidgetReturnCode res;
        PhidgetGPS_NMEAData NMEAData;

        printf("heading: \t\t%lf \n", heading);
        printf("velocity: \t\t%lf \n", velocity);
        /*
        res = PhidgetGPS_getNMEAData(ch, &NMEAData);
        if (res == EPHIDGET_OK){
            printf("----------\n");
        }
        */
    }




} // namespace am_sensors_gps




int main(int argc, char** argv)
{
  	ROS_INFO("Start Phidgets GPS Publisher!!");
	ros::init(argc, argv, "GPS_pub");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	am_sensors_gps::GPS_pub gps_instance(nh, nh_private);
  	ROS_INFO("Run GPS_pub!!");
	ros::spin();
	return 0;
}
