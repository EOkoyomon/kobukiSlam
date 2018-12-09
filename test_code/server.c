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

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

#define PORT 8080

// Returns true on success. Fills in server and client socket file descriptors.
bool start_instruction_server(int* server_fd, int* client_fd) {
	
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

	printf("Successfully connected to client computer for instructions");
	return true;
}

// Returns false if there was an error other than there just being no info
// Info on select in http://beej.us/guide/bgnet/html/single/bgnet.html#blocking
// Returns in the values for the variables passed in
bool read_new_instruction(int client_fd, int* duck_detect_left, int* duck_detect_center, int* duck_detect_right, float* duck_dist) {
	const int expected_bytes = 4*sizeof(int);
	*duck_detect_left = 0, *duck_detect_center = 0, *duck_detect_right = 0, *duck_dist = 0;
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
			printf("Got expected\n");
			*duck_detect_left = *((int *) buffer);
			*duck_detect_center = *(((int *) buffer) + 1);
			*duck_detect_right = *(((int *) buffer) + 2);
			*duck_dist = *( (float*) (((int *) buffer) + 3) );
			return true;
		}
		printf("Did not get expected\n");
	}

	return true;

}



int main(void) {

	int server_fd, client_fd;

	start_instruction_server(&server_fd, &client_fd);

	int duck_detect_left;
	int duck_detect_center;
	int duck_detect_right;
	float duck_dist;

	const int sleep_interval_in_ms = 10;
	while (1) {

		// usleep takesleep in microseconds
		usleep(sleep_interval_in_ms * 1000);

		if (!read_new_instruction(client_fd, &duck_detect_left,
						&duck_detect_center, &duck_detect_right, &duck_dist)) {
			// Break for now if cannot get instructions
			break;
		}

		printf("Duck detected left?: %d\n", (duck_detect_left != 0));
		printf("Duck detected center?: %d\n", (duck_detect_center != 0));
		printf("Duck detected right?: %d\n", (duck_detect_right != 0));
		printf("Duck dist: %f\n", duck_dist);
	}

	close(client_fd);
	close(server_fd);
	
}

