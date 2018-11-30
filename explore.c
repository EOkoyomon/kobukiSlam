// Robot exploration to find object

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "control_library/kobuki_library.h"
#include "control_library/kobukiSensorTypes.h"

#include <Python.h>

typedef enum {
  OFF,
  DRIVE_STRAIGHT,
  ROTATING,
  ROTATE_LEFT,
  ROTATE_RIGHT, 
  APPROACH, 
  RETURN
} robot_state_t;

typedef struct route 
{
  float distance;
  float rotate_angle;
  struct route *next;
} route_t;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;
	float result;

	if (previous_encoder < current_encoder) {
		result = CONVERSION * (0xFFFF - current_encoder + previous_encoder);
	} else {
		result = (previous_encoder - current_encoder) * CONVERSION;
	}
	
	if (result > 5) {
		return 0;
	}
	return result;
}

static float measure_distance_reverse(uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;

	float result;

	if (previous_encoder > current_encoder) {
		result = CONVERSION * (0xFFFF - previous_encoder + current_encoder);
	} else {
		result = (current_encoder - previous_encoder) * CONVERSION;
	}
	
	if (result > 5) {
		return 0;
	}
	return result;
}

PyObject* myModuleString = PyString_FromString((char*)"track_yellow");
PyObject* myModule = PyImport_Import(myModuleString);
PyObject* detect_left = PyObject_GetAttrString(myModule,(char*)"duck_detect_left");
PyObject* detect_right = PyObject_GetAttrString(myModule,(char*)"duck_detect_right");
PyObject* centered = PyObject_GetAttrString(myModule,(char*)"duck_centered");
PyObject* distance = PyObject_GetAttrString(myModule,(char*)"duck_distance");


static bool duck_detect_left(char *img_path) {
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(detect_left, args)
  double result = PyFloat_AsDouble(myResult);
}

static bool duck_detect_right(char *img_path) {
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(detect_right, args)
  double result = PyFloat_AsDouble(myResult);
}

static bool duck_centered(char *img_path) {
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(centered, args)
  double result = PyFloat_AsDouble(myResult);
}

static float duck_dist(char *img_path) {
  // once the duck is centered, get the distance
  PyObject* py_msg = PyString_FromString(img_path);
  PyObject* args = PyTuple_Pack(1,py_msg);
  PyObject* myResult = PyObject_CallObject(distance, args)
  double result = PyFloat_AsDouble(myResult);
 
}

/* Must be freed after. */
static route_t* get_return_directions(void) {

}

int main(void) {

	if (!kobukiLibraryInit()) {
		printf("Error initializing the Kobuki Library\n");
		exit(1);
	}

	printf("Kobuki Library Initiated\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

	bool started_rotation = false;
  clock_t start_time = 0;
	float target_rotation_time = kobukiTimeToReachAngleRight(90);
  float distance_traveled = 0;
  uint16_t old_encoder = 0;
  uint16_t new_encoder = 0;
  route_t * directions;

  // loop forever, running state machine
	const int sleep_interval_in_ms = 10;
  while (1) {
		// usleep takesleep in microseconds
		usleep(sleep_interval_in_ms * 1000);

		// read sensors from robot
		if (kobukiSensorPoll(&sensors) < 0) continue;

    // handle states
    switch(state) {

      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVE_STRAIGHT;
          new_encoder = sensors.leftWheelEncoder;
        } else {
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVE_STRAIGHT: {

        if (is_button_pressed(&sensors) ) {
          state = OFF;

        } else if (sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight) {
          state = ROTATING;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else if (duck_detect_left()) {
          state = ROTATE_LEFT;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else if (duck_detect_right()) {
          state = ROTATE_RIGHT;
          kobukiDriveDirect(0, 0);
          total_rotated = 0;

        } else {
          state = DRIVE_STRAIGHT;
          kobukiDriveDirect(100, 100);
        }

        break;
      }

      case ROTATING: {

				if (!started_rotation) {
					start_time = clock();
					started_rotation = true;
				}

        if (is_button_pressed(&sensors)) {
          state = OFF;

        } else if (duck_detect_left()) {
          state = ROTATE_LEFT;
          kobukiDriveDirect(0, 0);
          started_rotation = false;

        } else if (duck_detect_right()) {
          state = ROTATE_RIGHT;
          kobukiDriveDirect(0, 0);
          started_rotation = false;

        } else if (((float)(start_time - clock()) / (CLOCKS_PER_SEC*1000)) < target_rotation_time) {
          kobukiTurnRightFixed();

        } else {
          state = DRIVE_STRAIGHT;
          kobukiDriveDirect(0, 0);
          started_rotation = false;
        }

        break;
      }


      case ROTATE_LEFT: {
        // transition logic
        
        if (is_button_pressed(&sensors) ) {
          state = OFF;

        } else if (duck_centered()) {
          kobukiDriveDirect(0, 0);
        	state = APPROACH;

        } else {
          kobukiDriveDirect(-10, 10);
          state = ROTATE_LEFT;
        }

        break; // each case needs to end with break!
      }

      case ROTATE_RIGHT: {
        // transition logic
        
        if (is_button_pressed(&sensors) ) {
          state = OFF;

        } else if (duck_centered()) {
          kobukiDriveDirect(0, 0);
          state = APPROACH;

        } else {
          kobukiDriveDirect(10, -10);
          state = ROTATE_RIGHT;
        }

        break; // each case needs to end with break!
      }

      case APPROACH: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (duck_dist <= 0.1) {
          state = RETURN;
          directions = get_return_directions();
          distance_traveled = 0;
          total_rotated = 0;
        } else {
          kobukiDriveDirect(50, 50);
          state = APPROACH;
        }
      }

      case RETURN: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (directions == NULL) {
          state = OFF;
        } else {

          if (abs(directions->distance - distance_traveled) >= 0.1) {
            kobukiDriveDirect(50, 50);
            old_encoder = new_encoder;
            new_encoder = sensors.leftWheelEncoder;
            distance_traveled += measure_distance(old_encoder, new_encoder);
          } else if (abs(directions->rotate_angle - total_rotated) >= 0.1) {
            kobukiDriveDirect(10, -10);
            total_rotated = ;
          } else {
            directions = directions->next;
          }

          state = RETURN;
        }
      }
      


      // add other cases here

    }
  }
}

