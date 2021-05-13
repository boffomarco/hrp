#include <stdlib.h>
#include <stdio.h>
#include "phidget22.h"
#include <unistd.h>

void CCONV positionChangeHandler(PhidgetGPSHandle ch, void *ctx, double latitude, double longitude, double altitude) {

	PhidgetReturnCode res;
	PhidgetGPS_NMEAData NMEAData;


	//printf("Latidute: \t\t%lf \n", latitude);
	//printf("Longitude: \t\t%lf \n", longitude);
	//printf("Altitude: \t\t%lf \n", altitude);

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

        for (int i = 0; i < 12; ++i) {
            printf("satUsed: \t\t%d \n", NMEAData.GSA.satUsed[i]);
        }
        //printf("satUsed: \t\t%d \n", NMEAData.GSA.satUsed);
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
    }
    sleep(1);
}

void CCONV headingChangeHandler(PhidgetGPSHandle ch, void *ctx, double heading, double velocity) {

	PhidgetReturnCode res;
	PhidgetGPS_NMEAData NMEAData;

    printf("heading: \t\t%lf \n", heading);
    printf("velocity: \t\t%lf \n", velocity);

	res = PhidgetGPS_getNMEAData(ch, &NMEAData);
	if (res == EPHIDGET_OK){
        printf("----------\n");
    }

}



int main(int argc, char** argv){

	PhidgetReturnCode res;
	PhidgetGPSHandle ch;

	PhidgetGPS_create(&ch);

	res = PhidgetGPS_setOnPositionChangeHandler(ch, positionChangeHandler, NULL);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

    res = PhidgetGPS_setOnHeadingChangeHandler(ch, headingChangeHandler, NULL);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error


	res = Phidget_open((PhidgetHandle)ch);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

    int count = 0;
	while (1) {
        // Do work, wait for events, etc.
        sleep(10); // Do work, wait for events, etc.
        printf("-%d\n-", ++count);
	}

	PhidgetGPS_delete(&ch);

	return 0;
}
