// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "kobuki_library.h"
#include "kobukiSensorTypes.h"

typedef enum {
  OFF,
  DRIVING,
} robot_state_t;


int main(void) {
	
	kobukiLibraryInit();
	printf("Kobuki Library Initiated\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
	const int sleep_interval_in_ms = 100;

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

		// usleep takesleep in microseconds
    usleep(sleep_interval_in_ms * 1000);

    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (isButtonPressed(&sensors)) {
          state = DRIVING;
        } else {
          // perform state-specific actions here
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
        // transition logic
        if (isButtonPressed(&sensors)) {
          state = OFF;
        } else {
          // perform state-specific actions here
          state = DRIVING;
        }
        break; // each case needs to end with break!
      }

      // add other cases here

    }
  }
}


