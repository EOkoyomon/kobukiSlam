#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kobuki_library.h"
#include "kobuki_uart.h"
#include "kobukiSensor.h"
#include "kobukiSensorTypes.h"

// Request sensor data and wait for response
int32_t kobukiSensorPoll(KobukiSensors_t* const	sensors){

	int32_t status = 0;

	// initialize communications buffer
    // We know that the maximum size of the packet is less than 140 based on documentation
	uint8_t packet[140] = {0};

	status = kobuki_uart_recv(packet);

    if (status < 0) {
        return status;
    }

	// parse response
	kobukiParseSensorPacket(packet, sensors);

	return status;
}


