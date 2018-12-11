// Robot exploration to find object

#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

#include "control_library/kobuki_library.h"
#include "control_library/kobukiSensorTypes.h"

#include <signal.h>

#define PORT 8080

typedef enum {
	OFF,
	DRIVE_STRAIGHT,
	ROTATING,
	ROTATE_LEFT,
	ROTATE_RIGHT, 
	APPROACH, 
	GET_RETURN, 
	RETURN
} robot_state_t;

typedef struct route 
{
	float rotate_angle;
	float distance;
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


// Returns true on success. Fills in server and client socket file descriptors.
static bool start_instruction_server(int* server_fd, int* client_fd) {
	
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);

	// Creating socket file descriptor 
	if ((*server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		printf("Error initializing server socket\t%s\n", strerror(errno));

		return false;
	}

	// Forcefully attaching socket to the port 8080 Part 1
	if (setsockopt(*server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
		printf("Error initializing desired port - part 1:\t%s\n", strerror(errno));
		return false;
	}

	memset(&address, 0, addrlen);
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080 Part 2
	if (bind(*server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
		printf("Error initializing desired port - part 2\t%s\n", strerror(errno));

		return false;
	}

	// Listen for 3 connections, just because..
	if (listen(*server_fd, 3) < 0) {
		printf("Error initializing server listening\t%s\n", strerror(errno));

		return false;
	}

	printf("Waiting for client computer to connect\n");

	if ((*client_fd = accept(*server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
		printf("Error initializing connection to client for instruction\t%s\n", strerror(errno));
;
		return false;
	}	

	printf("Successfully connected to client computer for instructions\n");
	return true;
}


// Returns false if there was an error other than there just being no info
// Info on select in http://beej.us/guide/bgnet/html/single/bgnet.html#blocking
// Returns in the values for the variables passed in
static bool read_new_instruction(int client_fd, int* duck_detect_left, int* duck_detect_center, int* duck_detect_right) {
	const int expected_bytes = 3*sizeof(int);
	*duck_detect_left = 0, *duck_detect_center = 0, *duck_detect_right = 0;
	char buffer[expected_bytes];
	memset(buffer, 0, expected_bytes);

	struct timeval tv;
	fd_set readfds;
	int nbytes;

	// Wait 5ms before timing out and returning
	tv.tv_sec = 0;
	tv.tv_usec = 5000;

	FD_ZERO(&readfds);
	FD_SET(client_fd, &readfds);

	// Don't care about writefds and exceptfds:
	select(client_fd+1, &readfds, NULL, NULL, &tv);

	if (FD_ISSET(client_fd, &readfds)) {
		if ((nbytes = recv(client_fd, buffer, expected_bytes, 0)) <= 0) {
			printf("Error reading from client. Connection closed.\n");
			close(client_fd);
			return false;
		}
		if (nbytes == expected_bytes) {
			// printf("Got expected\n");
			*duck_detect_left = *((int *) buffer);
			*duck_detect_center = *(((int *) buffer) + 1);
			*duck_detect_right = *(((int *) buffer) + 2);
			return true;
		}
		printf("Did not get expected\n");
	}

	return true;

}

static bool requestInstructions(int client_fd) {
	const int expected_bytes = sizeof(int);
	const int signal = 249;

	char *buffer = (char *) &signal;
	int nbytes;
	struct timeval tv;
	fd_set writefds;

	// Wait 5ms before timing out and returning
	tv.tv_sec = 0;
	tv.tv_usec = 5000;

	FD_ZERO(&writefds);
	FD_SET(client_fd, &writefds);

	// Don't care about writefds and exceptfds:
	select(client_fd+1, NULL, &writefds, NULL, &tv);

	if (FD_ISSET(client_fd, &writefds)) {
		if ((nbytes = send(client_fd, buffer, expected_bytes, 0)) <= 0) {
			printf("Error reading from client. Connection closed.\n");
			close(client_fd);
			return false;
		}
   
		// Will just try again 
		if (nbytes != expected_bytes) {
			printf("Did not send expected\n");
			return true;
		}
	}
	return true;
}

/* Must be memory from head_instruction must be freed after. */
static bool readReturnSequence(int client_fd, route_t** head_instruction) {
	const int expected_bytes = 2*sizeof(float);
	char buffer[expected_bytes];
	memset(buffer, 0, expected_bytes);
	int n;
	route_t **head = head_instruction;

	struct timeval tv;
	fd_set readfds;
	int nbytes;

	// Wait 5ms before timing out and returning
	tv.tv_sec = 0;
	tv.tv_usec = 5000;

	FD_ZERO(&readfds);
	FD_SET(client_fd, &readfds);

	// Don't care about writefds and exceptfds:
	select(client_fd+1, &readfds, NULL, NULL, &tv);

	if (FD_ISSET(client_fd, &readfds)) {

		// Wait for start sequence
		do {
			nbytes = recv(client_fd, buffer, sizeof(int), 0);
			if (nbytes <= 0) {
				printf("Error reading from client. Connection closed.\n");
				close(client_fd);
				return false;
			}
			// Will try again
			if (nbytes != sizeof(int)) {
				*head_instruction = NULL;
				return true;
			}
		} while (*((int *) buffer) != 249);

		// Get the number (n) of instructions we expect
		if ((nbytes = recv(client_fd, buffer, sizeof(int), 0)) <= 0) {
			printf("Error reading from client. Connection closed.\n");
			close(client_fd);
			return false;
		}
		// Will try again
		if (nbytes != sizeof(int)) {
			*head_instruction = NULL;
			return true;
		}
		n = *((int *) buffer);

		// Get n instructions in a loop
		for (int i = 0; i < n; i+=1) {

			if ((nbytes = recv(client_fd, buffer, expected_bytes, 0)) <= 0) {
				printf("Error reading from client. Connection closed.\n");
				close(client_fd);
				return false;
			}
			// If do not receive right number, just exit and try again
			if (nbytes != expected_bytes) {
				*head_instruction = NULL;
				return true;
			}
			*head = malloc(sizeof(route_t));
			(*head)->rotate_angle = *((float *) buffer);
			(*head)->distance = *(((float *) buffer)+ 1);
			(*head)->next = NULL;
			// The address of the next route_t
			head = &((*head)->next);
		}
	}

	return true;
}


int main(void) { // to start the kinect recorder, lets try putting the function in track_yellow and starting it by calling a python function, and when we get a SIGINT, handle it in the python function by killing the recorder

	if (!kobukiLibraryInit()) {
		printf("Error initializing the Kobuki Library\n");
		exit(1);
	}

	printf("Kobuki Library Initiated\n");
	
	int server_fd, client_fd;

	start_instruction_server(&server_fd, &client_fd);

	// configure initial state
	robot_state_t state = OFF;
	KobukiSensors_t sensors = {0};
	

	bool started_rotation = false;
	clock_t start_time = 0;
	float target_rotation_time = kobukiTimeToReachAngle(90);
	float distance_traveled = 0;
	uint16_t old_encoder = 0;
	uint16_t new_encoder = 0;
	route_t *next_instr_ptr = NULL, *terminator = NULL;

	int duck_detect_left;
	int duck_detect_center;
	int duck_detect_right;

	int i = 0;
	// loop forever, running state machine
	const int sleep_interval_in_ms = 10;
	while (1) {
		// printf("STATE: %d\n", state);

		// usleep takesleep in microseconds
		usleep(sleep_interval_in_ms * 1000);

		// read sensors from robot
		if (kobukiSensorPoll(&sensors) < 0) continue;

		if (state != GET_RETURN && state != RETURN) {
			if (!read_new_instruction(client_fd, &duck_detect_left,
							&duck_detect_center, &duck_detect_right)) {
				// Break for now if cannot get instructions
				goto end;
			}
		}
		// i++;
		if (duck_detect_left || duck_detect_center || duck_detect_right) {
			// printf("\nNetwork reads: %d\n", i);
			printf("Duck_left:\t%d\nDuck_center:\t%d\nDuck_right:\t%d\n", duck_detect_left, duck_detect_center, duck_detect_right);
		}

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
					//	total_rotated = 0;

				} else if (duck_detect_center) {
					printf("approaching\n");
					state = APPROACH;
					kobukiDriveDirect(0, 0);

				} else if (duck_detect_left) {
					printf("rotate left\n");
					state = ROTATE_LEFT;
					kobukiDriveDirect(0, 0);
					// total_rotated = 0;

				} else if (duck_detect_right) {
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

				} else if (duck_detect_center) {
					printf("approaching\n");
					state = APPROACH;
					kobukiDriveDirect(0, 0);
					started_rotation = false;

				} else if (duck_detect_left) {
					printf("rotate left\n");
					state = ROTATE_LEFT;
					kobukiDriveDirect(0, 0);
					started_rotation = false;

				} else if (duck_detect_right) {
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

				} else if (duck_detect_center) {
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

				} else if (duck_detect_center) {
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

				} else if (sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight) {
					kobukiDriveDirect(0, 0);
					kobukiPlaySoundSequence(kobukiCleaningStart);
					printf("Waiting for return instructions\n");
					state = GET_RETURN;

				} else {
					kobukiDriveDirect(50, 50);
					state = APPROACH;
				}

				break;
			}

			case GET_RETURN: {
				
				if (isButtonPressed(&sensors)) {
					state = OFF;

				} else {

					if (i < 10) {
						if (!requestInstructions(client_fd)) {
							goto end;
						}
						i++;
					}	

					next_instr_ptr = NULL;
					if (!readReturnSequence(client_fd, &next_instr_ptr)) {
						goto end;
					}

					if (next_instr_ptr != NULL) {
						distance_traveled = 0;
						started_rotation = false;
						printf("returning\n");
						state = RETURN;

					} else {
						state = GET_RETURN;
					}

				}

				break;
			}
			
			case RETURN: {

				if (isButtonPressed(&sensors)) {
					state = OFF;

				} else if (next_instr_ptr == NULL) {
					kobukiPlaySoundSequence(kobukiCleaningEnd);
					started_rotation = false;
					target_rotation_time = kobukiTimeToReachAngle(90);
					printf("YAY, WE DID IT\n");
					state = OFF;

				} else {
					
					if (!started_rotation) {
						started_rotation = true;
						target_rotation_time = kobukiTimeToReachAngle(fabsf(next_instr_ptr->rotate_angle));
						start_time = clock();
						
					}
					
					if (((float)(clock() - start_time) / (CLOCKS_PER_SEC*1000)) < target_rotation_time) {
						if (next_instr_ptr->rotate_angle < 0) {
							// Rotating right
							kobukiTurnRightFixed();
							
						} else {
							// Rotating left
							kobukiTurnLeftFixed();
						}

					} else if (fabsf(next_instr_ptr->distance - distance_traveled) >= 0.1) {
						kobukiDriveDirect(50, 50);
						old_encoder = new_encoder;
						new_encoder = sensors.leftWheelEncoder;
						distance_traveled += measure_distance(old_encoder, new_encoder);

					} else {
						kobukiDriveDirect(0, 0);
						distance_traveled = 0;
						terminator = next_instr_ptr;
						next_instr_ptr = next_instr_ptr->next;
						started_rotation = false;
						free(terminator);
						state = RETURN;
					}
				}
				
				break;				
			}
			// add other cases here

			default: {
				break;
			}

		}
	}
	
	end:
	close(client_fd);
	close(server_fd);
	
}

