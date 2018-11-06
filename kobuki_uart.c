#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "kobuki_uart.h"

/* 
https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3b_1p2_reduced.pdf
https://www.jameco.com/Jameco/workshop/circuitnotes/raspberry-pi-circuit-note.html

Pin 8 (GPIO14) is set to UART0_TXD - Transmit line
Pin 10 (GPIO15) is set to UART0_RXD - Receive line

*/

/* Serial file on Raspberry Pi Model 3. */
const char[] serial_filepath = "/dev/ttys0";
int uart_fd = -1;

/* Returns < 0 on error. */
int kobuki_uart_init(void) {

	/*
	O_RDWR - Open for reading and writing.
	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	O_NDELAY - Enables nonblocking mode. When set, read or write requests on the file can return immediately with a failure status instead of blocking.
	*/
	uart_fd = open(serial_filepath, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart_fd == -1) {
		printf("ERROR - cannot open serial port\n");
		return -1;
	}

	/* CONFIGURE THE UART
	The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
		Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
		CSIZE:- CS5, CS6, CS7, CS8
		CLOCAL - Ignore modem status lines
		CREAD - Enable receiver
		IGNPAR = Ignore characters with parity errors
		ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
		PARENB - Parity enable
		PARODD - Odd parity (else even)	
	*/


	/* For Kobuki - Baud rate: 115200 BPS, Data bit: 8 bit, Stop bit: 1 bit, No Parity. */	
	struct termios options;
	tcpgetattr(uart_fd, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &options);

	return 1;
}

void kobuki_uart_close(void) {
	close(uart_fd);
}


static uint8_t checksum_create(uint8_t* data, uint8_t len) {
	uint8_t cs = 0;

	for (int i = 0; i < len; i++) {
		cs ^= data[i];
	}
	
	return cs;
}


/* Returns number of bytes sent or < 0 on error. */
int kobuki_uart_send(uint8_t* payload, uint8_t len) {
	uint8_t writeData[256] = {0};

	writeData[0] = 0xAA;
  writeData[1] = 0x55;
  writeData[2] = len;
  memcpy(writeData + 3, payload, len);
	writeData[3+len] = checksum_create(writeData + 2, len + 1);

	if (uart_fd != -1) {
		int count = write(uart_fd, writeData, len+4);
		if (count < 0) {
			printf("ERROR - failed to transmit on uart\n");
		}
		return count;
	}

	return -1;
}


/* Returns number of bytes read or < 0 on error. */
int kobuki_uart_recv(uint8_t* buffer) {

	typedef enum {
			wait_until_HDR,
			read_length,
			read_payload,
			read_checksum
	} state_type;

	state_type state = wait_until_HDR;

	uint8_t byteBuffer;
	int status = 0;
	uint8_t payloadSize = 0;
	uint8_t calcuatedCS;

	status = fsync(uart_fd);

	int num_checksum_failures = 0;
  uint8_t p_index = 0;

	while (1) {

		status = read(uart_fd, &packetBuffer[p_index], 1);
		if (status == 0) {
			continue;
		} elif (status < 0) {
			printf("ERROR - received error while reading from uart");
			return status;
		}

		switch(state) {
			case wait_until_HDR: {
				p_index++;

				if (p_index == 2 && packetBuffer[0]==0xAA && packetBuffer[1]==0x55) {
					state = read_length;
				} else if (p_index == 2) {
					p_index = 0;
					state = wait_until_HDR;
				} else {
					state = wait_until_HDR;
				}

				break;
			}

			case read_length: {
				payloadSize = packetBuffer[p_index];
				byteBuffer = payloadSize;
				p_index++;
				state = read_payload;

				break;
			}

			case read_payload: {
				p_index++;
				if (p_index > payloadSize+3) {
					state = read_checksum;
				}

				break;
			}

			case read_checksum: {
				calcuatedCS = checksum_create(packetBuffer + 2, payloadSize + 1);
				byteBuffer = packetBuffer[payloadSize+3];
				if (calcuatedCS == byteBuffer) {
					num_checksum_failures = 0;
					return payloadSize + 3;
				} else {
					state = wait_until_HDR;
					p_index = 0;
					if (num_checksum_failures == 3) {
						printf("ERROR - checksum did not match data 4 times");
						return -1500;
					}
					num_checksum_failures++;
				}

				break;
				}

				default:
					break;
		}

	}
	return status;

}

