#ifndef _KOBUKISENSOR_H
#define _KOBUKISENSOR_H
#include <stdbool.h>
#include <stdint.h>

#include "kobukiSensorTypes.h"
void kobukiParseSensorPacket(
	const uint8_t * packet,
	KobukiSensors_t * sensors
);

#endif
