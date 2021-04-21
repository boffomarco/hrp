#include <stdlib.h>
#include "phidget22.h"
#include <stdio.h>


int main() {
	PhidgetReturnCode res;
	PhidgetAccelerometerHandle ch;

	PhidgetAccelerometer_create(&ch);

	res = Phidget_openWaitForAttachment((PhidgetHandle)ch, PHIDGET_TIMEOUT_DEFAULT);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

	res = PhidgetAccelerometer_setDataInterval(ch, 10);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error


	uint32_t dataInterval;
	res = PhidgetAccelerometer_getDataInterval(ch, &dataInterval);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error
	printf("interval %d \n",dataInterval);


	double timestamp;
	double prevTimestamp = 0;
	double acceleration[3];
	for(int i = 0; i < 100; i++){
		res = PhidgetAccelerometer_getTimestamp(ch, &timestamp);
		if (res != EPHIDGET_OK)
			return 1; // Exit in error
		res = PhidgetAccelerometer_getAcceleration(ch, &acceleration);
		if (res != EPHIDGET_OK)
			return 1; // Exit in error


		printf("%d\t", i);
		printf("x %f - y %f - z %f \t %f \n",acceleration[0], acceleration[1], acceleration[2], timestamp - prevTimestamp);

		prevTimestamp = timestamp;
	}

	double minAcceleration[3];
	res = PhidgetAccelerometer_getMinAcceleration(ch, &minAcceleration);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

	printf("min x %f - y %f - z %f \n",minAcceleration[0], minAcceleration[1], minAcceleration[2]);


	double maxAcceleration[3];
	res = PhidgetAccelerometer_getMaxAcceleration(ch, &maxAcceleration);
	if (res != EPHIDGET_OK)
		return 1; // Exit in error

	printf("MAX x %f - y %f - z %f \n",maxAcceleration[0], maxAcceleration[1], maxAcceleration[2]);



	PhidgetAccelerometer_delete(&ch);

	return 0;
}
