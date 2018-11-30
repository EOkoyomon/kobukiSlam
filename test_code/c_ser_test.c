#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

/* 
https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3b_1p2_reduced.pdf
https://www.jameco.com/Jameco/workshop/circuitnotes/raspberry-pi-circuit-note.html

Pin 8 (GPIO14) is set to UART0_TXD - Transmit line
Pin 10 (GPIO15) is set to UART0_RXD - Receive line

*/

/* Serial file on Raspberry Pi Model 3. */
const char serial_filepath[] = "/dev/serial0";
int uart_fd = -1;

/* Returns < 0 on error. */
int setup(void) {

	/*
	O_RDWR - Open for reading and writing.
	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	O_NDELAY - Enables nonblocking mode. When set, read or write requests on the file can return immediately with a failure status instead of blocking.
	*/
	uart_fd = open(serial_filepath, O_RDWR | O_NOCTTY); // | O_NDELAY);

	if (uart_fd == -1) {
		printf("ERROR - cannot open serial port\n\t%s\n", strerror(errno));
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
	tcgetattr(uart_fd, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &options);

	return 1;
}

void teardown(void) {
	close(uart_fd);
}

/* Returns number of bytes sent or < 0 on error. */
int uart_write(void) {
	uint8_t writeData[20] = {0};
	uint8_t *p = &writeData[0];

	*p++ = 'h';
	*p++ = 'e';
	*p++ = 'l';
	*p++ = 'l';
	*p++ = 'o';
	*p++ = '\0';

	if (uart_fd != -1) {
		int count = write(uart_fd, writeData, 6);
		if (count < 0) {
			printf("ERROR - failed to transmit on uart\n\t%s\n", strerror(errno));
		}

		fsync(uart_fd);
		return count;
	}

	return -1;
}


/* Returns number of bytes read or < 0 on error. */
int uart_read(char* c) {

	fsync(uart_fd);

	if (uart_fd != -1) {
		int count = read(uart_fd, c, 1);
		if (count < 0) {
			printf("ERROR - failed to transmit on uart\n\t%s\n", strerror(errno));
		}

		fsync(uart_fd);
		return count;
	}

	return -1;

}

int main(void) {
	if (setup() < 0) return 1;

	char buff[32];
	char *c = buff;

	while (1) {
		sleep(1);
	
		buff[0] = 'f';
		buff[1] = 'f';
		buff[2] = 'f';
		buff[3] = 'f';
		buff[4] = 'f';

		uart_write();
		c = buff;
		while (uart_read(c) > 0 && *c != '\0') {
			printf("%c", *c);
			c++;
		}

		printf("\n");
	}
}
