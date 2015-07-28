#include <sys/types.h>
#include <sys/socket.h>	// socket(), connect(), send(), recv()
#include <netinet/in.h>	// sockaddr_in
#include <arpa/inet.h>	// htons(), inet_aton()
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "socket-server.h"

#define BUF 1024

int created_socket;
int new_socket;
struct sockaddr_in address = {0};
socklen_t addrlen;
char *buffer;

result_t socket_server_init(void) {
	// create socket
	if ((created_socket = socket(AF_INET, SOCK_STREAM, 0)) > 0)
		printf ("\tsocket successfully created\n");

	const int y = 1;
  	setsockopt(created_socket, SOL_SOCKET, SO_REUSEADDR, &y, sizeof(int));
	
	// bind socket to port
	address.sin_family = AF_INET;
	address.sin_port = htons(15000);
	address.sin_addr.s_addr = INADDR_ANY;	
	if (bind(created_socket, (struct sockaddr *) &address, sizeof (address)) != 0) {
		printf( "\tport is blocked!\n");
		return FAIL;
	}  
	
	// listen
	listen(created_socket, 3);
	
	addrlen = sizeof(struct sockaddr_in);
	buffer = malloc(BUF);
	
	return SUCCESS;
}	

result_t socket_server_wait_for_client(void) {
	new_socket = accept(created_socket, (struct sockaddr *) &address, &addrlen);
	if (new_socket > 0) {
		printf ("\tclient(%s) connected...\n", inet_ntoa(address.sin_addr));
		return SUCCESS;
	} else return FAIL;
}

result_t socket_server_receive(char **val, int *len) {			
	ssize_t size;
	size = recv(new_socket, buffer, BUF-1, 0);
	if( size > 0) {
		buffer[size] = '\0';
		printf ("message received: %s(%d)\n", buffer,size);
	}
	
	*len = size;
	*val = malloc(size);
	memcpy(*val, buffer, size);
	return SUCCESS;
}

result_t socket_server_send(char *val, int len) {			
	//~ printf("send message: ");
	//~ fgets(buffer, BUF, stdin);
	send(new_socket, val, len, 0);

	return SUCCESS;
}

result_t socket_server_close(void) {
	if (new_socket) close(new_socket);
	if (created_socket) close(created_socket);
		
	return SUCCESS;
}

result_t client_socket_close(void) {
	if (new_socket) close(new_socket);
		
	return SUCCESS;
}
