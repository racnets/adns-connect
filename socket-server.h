/*
 * socket-server.h
 */

#ifndef SOCKET_SERVER_H_
#define SOCKET_SERVER_H_
#include <stdint.h>

typedef enum {
	SUCCESS,
	FAIL
} result_t;	

result_t socket_server_wait_for_client(void);
result_t socket_server_close(void);
result_t client_socket_close(void);
result_t socket_server_init(void);
result_t socket_server_receive(char **val, int *len);
result_t socket_server_send(char *val, int len);

#endif /* SOCKET_SERVER_H_ */
