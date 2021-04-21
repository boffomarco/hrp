#include <stdlib.h>
#include <stdio.h>
#include "phidget22.h"


void CCONV spatialDataHandler(PhidgetSpatialHandle ch, void *ctx, const double acceleration[3], const double angularRate[3], const double magneticField[3], double timestamp) {
	double* TimeStamp = (double*)ctx;
	double diff = (timestamp - (*TimeStamp));
	// Work with event data...
	printf("Acceleration: \t%lf  |  %lf  |  %lf\n", acceleration[0], acceleration[1], acceleration[2]);
	printf("AngularRate: \t%lf  |  %lf  |  %lf\n", angularRate[0], angularRate[1], angularRate[2]);
	printf("MagneticField: \t%lf  |  %lf  |  %lf\n", magneticField[0], magneticField[1], magneticField[2]);
	printf("Timestamp: %lf and difference: %lf \n", timestamp, diff);
	printf("----------\n");
	(*TimeStamp) = timestamp;
}

int main() {
	PhidgetReturnCode res;
	PhidgetSpatialHandle ch;

	PhidgetSpatial_create(&ch);


	double TimeStamp;
	res = PhidgetSpatial_setOnSpatialDataHandler(ch, spatialDataHandler, &TimeStamp);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

	res = Phidget_openWaitForAttachment((PhidgetHandle)ch, PHIDGET_TIMEOUT_DEFAULT);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

	res = Phidget_setDataInterval((PhidgetHandle)ch, 4);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error


	uint32_t dataInterval;
	res = Phidget_getDataInterval((PhidgetHandle)ch, &dataInterval);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error
	printf("data interval %d \n",dataInterval);


	Phidget_SpatialAlgorithm algorithm;
	res = PhidgetSpatial_getAlgorithm(ch, &algorithm);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error
	printf("algorithm %d \n",algorithm);


	int timestamp = 0;
	while (1) { ; }

	PhidgetSpatial_delete(&ch);

	return 0;
}
