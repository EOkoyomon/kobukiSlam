#include "kobuki_library.h"
#include "kobuki_uart.h"
#include "kobukiSensor.h"
#include "kobukiSensorTypes.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Initializes Kobuki Library. Called before library functions. */
bool kobukiLibraryInit(void) {
	return (kobuki_uart_init() >= 0);
}

/* Request sensor data and wait for response. */
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

/* Checks for the state change of a button press on any of the Kobuki buttons. */
bool isButtonPressed(KobukiSensors_t* sensors) {
  // save previous states of buttons
  static bool previous_B0 = false;
  static bool previous_B1 = false;
  static bool previous_B2 = false;

  bool result = false;

  // check B0
  bool current_B0 = sensors->buttons.B0;
  if (current_B0 && previous_B0 != current_B0) {
    result = true;
  }
  previous_B0 = current_B0;

  // check B1
  bool current_B1 = sensors->buttons.B1;
  if (current_B1 && previous_B1 != current_B1) {
    result = true;
  }
  previous_B1 = current_B1;

  // check B2
  bool current_B2 = sensors->buttons.B2;
  if (current_B2 && previous_B2 != current_B2) {
    result = true;
  }
  previous_B2 = current_B2;

  return result;
}

int32_t kobukiDriveDirect(int16_t leftWheelSpeed, int16_t rightWheelSpeed){
	int32_t CmdSpeed;
	int32_t CmdRadius;

	if (abs(rightWheelSpeed) > abs(leftWheelSpeed)) {
	    CmdSpeed = rightWheelSpeed;
	} else {
	    CmdSpeed = leftWheelSpeed;
	}

	if (rightWheelSpeed == leftWheelSpeed) {
	    CmdRadius = 0;  // Special case 0 commands Kobuki travel with infinite radius.
	} else {
	    CmdRadius = (rightWheelSpeed + leftWheelSpeed) / (2.0 * (rightWheelSpeed - leftWheelSpeed) / 123.0);  // The value 123 was determined experimentally to work, and is approximately 1/2 the wheelbase in mm.
	    CmdRadius = round(CmdRadius);
	    //if the above statement overflows a signed 16 bit value, set CmdRadius=0 for infinite radius.
	    if (CmdRadius>32767) CmdRadius=0;
	    if (CmdRadius<-32768) CmdRadius=0;
	    if (CmdRadius==0) CmdRadius=1;  // Avoid special case 0 unless want infinite radius.
	}

	if (CmdRadius == 1){
		CmdSpeed = CmdSpeed * -1;
	}

	int32_t status = kobukiDriveRadius(CmdRadius, CmdSpeed);


	return status;
}

int32_t kobukiDriveRadius(int16_t radius, int16_t speed){
    uint8_t payload[6];

    payload[0] = 0x01;
    payload[1] = 0x04;
    memcpy(payload+2, &speed, 2);
    memcpy(payload+4, &radius, 2);

    return kobuki_uart_send(payload, 6);
}

int32_t kobukiSetControllerDefault(void) {
    uint8_t payload[15] = {0};

    payload[0] = 0x0D; // PID type 13
    payload[1] = 0x0D; // 13 byte PID length
    payload[2] = 0x00; // Default gain

    return kobuki_uart_send(payload, 15);
}

int32_t kobukiSetControllerUser(uint32_t Kp, uint32_t Ki, uint32_t Kd){
    uint8_t payload[15] = {0};

    payload[0] = 0x0D; // PID type 13
    payload[1] = 0x0D; // 13 byte PID length
    payload[2] = 0x01; // User gain

    //The values passed in are multiplied by 1000 to be represented by integers
    // P = Kp/1000
    // I = Ki/1000;
    // D = Kd/1000;
    memcpy(payload + 3, &Kp, 4);
    memcpy(payload + 7, &Ki, 4);
    memcpy(payload + 11, &Kd, 4);

    return kobuki_uart_send(payload, 15);
}

// Play a sound of f = 1/(frequency * 0.00000275) with duration
/*int32_t kobukiPlaySound(uint32_t frequency_hz, uint8_t duration_ms) {
    uint16_t use_f = (uint16_t)(1.0/(frequency_hz * 0.00000275));
    uint8_t payload[5] = {0};
    payload[0] = 0x03;
    payload[1] = 0x03;
    memcpy(payload + 2, &use_f, 2);
    payload[4] = duration_ms;
    return kobuki_uart_send(payload, 5);
}*/

// Play a predefined sound from the above sound types
int32_t kobukiPlaySoundSequence(KobukiSound_t sound) {
    uint8_t payload[3] = {0};

    payload[0] = 0x04;
    payload[1] = 0x01;
    payload[2] = (uint8_t)sound;

    return kobuki_uart_send(payload, 3);
}

// Request hardware version, firmware version and unique ID on the next data packet
int32_t kobukiRequestInformation(void) {
    uint8_t payload[4] = {0};

    payload[0] = 0x09;
    payload[1] = 0x02;
    payload[2] = 0x08 | 0x02 | 0x01;
    payload[3] = 0x00;

    return kobuki_uart_send(payload, 4);
}
// Control Output and LEDs on the Robot
// The four least significant bits of outputs controls outputs 0-3
// The four least significant bits of leds controls leds 0-3
// All power channels are enabled by this command to prevent
// accidental power down of the controller board
/*int32_t kobukiGeneralOutput(uint8_t outputs, uint8_t leds) {
    uint8_t payload[4] = {0};
    uint16_t general_output = 0;
    //general_output = (leds & 0x0F << 8) | (0x0F << 4) | (outputs & 0x0F);
    payload[0] = 0x0C;
    payload[1] = 0x02;
    memcpy(payload + 2, &general_output, 2);
    return kobuki_uart_send(payload, 4);
}*/
