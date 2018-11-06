#ifndef _KOBUKI_UART_H
#define _KOBUKI_UART_H


/* Must call before using uart functions. Returns < 0 on error. */
int kobuki_uart_init(void);

/* Must call as exiting module. */
void kobuki_uart_close(void);

/* Takes in pointer to data to send as well as length of data.
Returns number of bytes sent or < 0 on error. */
int kobuki_uart_send(uint8_t* payload, uint8_t len);

/* Takes in pointer to receive buffer of where to put read data.
Buffer must be at least 140 bytes because maximum size of the packet is less than 140 based on documentation.
Returns number of bytes read or < 0 on error. */
int kobuki_uart_recv(uint8_t* buffer);

#endif
