// Robot exploration to find object

#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "control_library/kobuki_library.h"
#include "control_library/kobukiSensorTypes.h"

#include <Python.h>
#include <signal.h>

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

PyObject* myModuleString;
PyObject* myModule;
PyObject* pDict;
PyObject* detect_left;
PyObject* detect_right;
PyObject* centered;
PyObject* distance;


static bool duck_detect_left() {
  // PyObject* py_msg = PyString_FromString(img_path);
  // PyObject* args = PyTuple_Pack(1,py_msg);
  return false;
  PyObject* myResult = PyObject_CallObject(detect_left, NULL);
  double result = PyFloat_AsDouble(myResult);
  return result;
}

static bool duck_detect_right() {
  // PyObject* py_msg = PyString_FromString(img_path);
  // PyObject* args = PyTuple_Pack(1,py_msg);
  return false;
  PyObject* myResult = PyObject_CallObject(detect_right, NULL);
  double result = PyFloat_AsDouble(myResult);
  return result;
}

static bool duck_centered() {
  // PyObject* py_msg = PyString_FromString(img_path);
  // PyObject* args = PyTuple_Pack(1,py_msg);
  return false;
  PyObject* myResult = PyObject_CallObject(centered, NULL);
  double result = PyFloat_AsDouble(myResult);
  return result;
}

static float duck_dist() {
  // once the duck is centered, get the distance
  // PyObject* py_msg = PyString_FromString(img_path);
  // PyObject* args = PyTuple_Pack(1,py_msg);
  return 0;
  PyObject* myResult = PyObject_CallObject(distance, NULL);
  double result = PyFloat_AsDouble(myResult);
  return result;
}

/* Must be freed after. */
static route_t* get_return_directions(void) {

}

void intHandler(int sig) {
    Py_Exit(0);
    exit(0);
}

int main(void) { // to start the kinect recorder, lets try putting the function in track_yellow and starting it by calling a python function, and when we get a SIGINT, handle it in the python function by killing the recorder

    signal(SIGINT, intHandler);
    
    if (!kobukiLibraryInit()) {
        printf("Error initializing the Kobuki Library\n");
        exit(1);
    }

    printf("Kobuki Library Initiated\n");
    
//	int pid = fork();
//
//	if (pid == 0) {
//    		printf("kinect capture started\n");
//		int fd = open("/dev/null", O_RDWR);
//		dup2(fd, 1);
//		dup2(fd, 2);
//
//		char* argv[2];
//		argv[0] = "./kinect_record";
//		argv[1] = NULL;
//		execv("./kinect_record", argv);
//		exit(1);
//    	// system("./kinect_record 2> /dev/null > /dev/null &");
//	}
	

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
    
    Py_Initialize();
    PyRun_SimpleString((char*)"import sys");
    PyRun_SimpleString("sys.path.insert(0, '.')");
    
    myModuleString = PyString_FromString((char*)"track_yellow");
    myModule = PyImport_Import(myModuleString);

    pDict = PyModule_GetDict(myModule); //printf("hello\n");
    detect_left = PyDict_GetItemString(pDict, (char*)"duck_detect_left");
    detect_left = PyObject_GetAttrString(myModule,(char*)"duck_detect_left");
    detect_right = PyObject_GetAttrString(myModule,(char*)"duck_detect_right");
    centered = PyObject_GetAttrString(myModule,(char*)"duck_centered");
    distance = PyObject_GetAttrString(myModule,(char*)"duck_distance");

    // loop forever, running state machine
    const int sleep_interval_in_ms = 10;
    while (1) {
	// printf("STATE: %d\n", state);

        // usleep takesleep in microseconds
        usleep(sleep_interval_in_ms * 1000);

        // read sensors from robot
        if (kobukiSensorPoll(&sensors) < 0) continue;

        // handle states
        switch(state) {

        case OFF: {
            // transition logic
            if (isButtonPressed(&sensors)) {
		printf("driving\n");
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

            if (isButtonPressed(&sensors) ) {
                state = OFF;

            } else if (sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight) {
           	printf("rotating\n");
     		state = ROTATING;
                kobukiDriveDirect(0, 0);
                //  total_rotated = 0;

            } else if (duck_centered()) {
		printf("approaching\n");
                state = APPROACH;
                kobukiDriveDirect(0, 0);

            } else if (duck_detect_left()) {
		printf("rotate left\n");
                state = ROTATE_LEFT;
                kobukiDriveDirect(0, 0);
                // total_rotated = 0;

            } else if (duck_detect_right()) {
		printf("rotate right\n");
                state = ROTATE_RIGHT;
                kobukiDriveDirect(0, 0);
                // total_rotated = 0;

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

            if (isButtonPressed(&sensors)) {
                state = OFF;

            } else if (duck_centered()) {
		printf("approaching\n");
                state = APPROACH;
                kobukiDriveDirect(0, 0);
                started_rotation = false;

            } else if (duck_detect_left()) {
		printf("rotate left\n");
                state = ROTATE_LEFT;
                kobukiDriveDirect(0, 0);
                started_rotation = false;

            } else if (duck_detect_right()) {
		printf("rotate right\n");
                state = ROTATE_RIGHT;
                kobukiDriveDirect(0, 0);
                started_rotation = false;

            } else if (((float)(clock() - start_time) / (CLOCKS_PER_SEC*1000)) < target_rotation_time) {
                kobukiTurnRightFixed();

            } else {
		printf("driving straight\n");
                state = DRIVE_STRAIGHT;
                kobukiDriveDirect(0, 0);
                started_rotation = false;
            }

            break;
        }


        case ROTATE_LEFT: {
            // transition logic
            
            if (isButtonPressed(&sensors) ) {
                state = OFF;

            } else if (duck_centered()) {
		printf("approaching\n");
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
            
            if (isButtonPressed(&sensors) ) {
                state = OFF;

            } else if (duck_centered()) {
		printf("approaching\n");
                kobukiDriveDirect(0, 0);
                state = APPROACH;

            } else {
                kobukiDriveDirect(10, -10);
                state = ROTATE_RIGHT;
            }

            break; // each case needs to end with break!
        }

        case APPROACH: {
            if (isButtonPressed(&sensors)) {
                state = OFF;
            } else if (duck_dist() <= 0.1) {
		printf("gets duck dist\n");
		kobukiDriveDirect(0, 0);
                state = RETURN;
                directions = get_return_directions();
		printf("got directions\n");
                distance_traveled = 0;
                // total_rotated = 0;
            } else {
                kobukiDriveDirect(50, 50);
                state = APPROACH;
            }
		break;
        }

        case RETURN: {

	/*
            if (isButtonPressed(&sensors)) {
                state = OFF;
            } else if (directions == NULL) {
                state = OFF;
            } else {

            if (abs(directions->distance - distance_traveled) >= 0.1) {
                kobukiDriveDirect(50, 50);
                old_encoder = new_encoder;
                new_encoder = sensors.leftWheelEncoder;
                distance_traveled += measure_distance(old_encoder, new_encoder);
            } else if (abs(directions->rotate_angle) >= 0.1) {
                kobukiDriveDirect(10, -10);
                // total_rotated = ;
            } else {
                directions = directions->next;
            }

                state = RETURN;
            }
	*/
	printf("returning\n");
	state = OFF;
	break;
        }
        
        

      // add other cases here

    }
  }
  
  Py_Finalize();
}

