// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../control_library/kobuki_library.h"
#include "../control_library/kobukiSensorTypes.h"

typedef enum {
  OFF,
  DRIVING,
} robot_state_t;


int main(void) {
	
	if (!kobukiLibraryInit()) {
		printf("Error initializing the Kobuki Library\n");
		exit(1);
	}

	printf("Kobuki Library Initiated\n");
	
	// configure initial state
	robot_state_t state = OFF;
	KobukiSensors_t sensors = {0};
	const int sleep_interval_in_ms = 10;
	
	// loop forever, running state machine
	while (1) {
		// usleep takesleep in microseconds
		usleep(sleep_interval_in_ms * 1000);

		// read sensors from robot
		if (kobukiSensorPoll(&sensors) < 0) continue;
	
	
		// handle states
		switch(state) {
			
			case OFF: {
				// transition logic
				if (isButtonPressed(&sensors)) {
					printf("Button pressed.\n");
					state = DRIVING;
				} else {
					// perform state-specific actions here
					kobukiDriveDirect(0, 0);
					state = OFF;
				}
				
				break; // each case needs to end with break!
			}

			case DRIVING: {
				// transition logic
				if (isButtonPressed(&sensors) || sensors.bumps_wheelDrops.bumpCenter) {
					printf("Button pressed.\n");
					state = OFF;
				} else {
					// perform state-specific actions here
					kobukiDriveDirect(200, 200);
					state = DRIVING;
				}
				
				break; // each case needs to end with break!
			}

      // add other cases here

		}
	}
}


