#define _POSIX_C_SOURCE 1 /* ctime_r() */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <time.h>

#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>




#define DEFAULT_SERVER_PORT    4567




static void handle_client(int sockfd);




int main(void)
{
	int listen_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (-1 == listen_socket) {
		fprintf(stderr, "[EE] socket() failed: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
	fprintf(stderr, "[II] Socket successfully created\n");


	struct sockaddr_in serveraddr;
	memset(&serveraddr, 0, sizeof (serveraddr));
	serveraddr.sin_family = AF_INET; /* Server's address is an Internet address. */
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Let the system figure out our address. */
	serveraddr.sin_port = htons(DEFAULT_SERVER_PORT); /* Port to listen on. */


	/* Associate listen socket with a port. */
	if (0 != bind(listen_socket, (struct sockaddr *) &serveraddr, sizeof (serveraddr))) {
		fprintf(stderr, "[EE] bind() failed: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}


	/* Start listening for new connections. Socket becomes ready
	   to accept new connections. */
	if (0 != listen(listen_socket, 1)) {
		fprintf(stderr, "[EE] listen() failed: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}


	while (1) {
		/* Accept connection from client. */
		struct sockaddr_in client;
		socklen_t len = sizeof (client);
		int conn_socket = accept(listen_socket, (struct sockaddr *) &client, &len);
		if (conn_socket < 0) {
			fprintf(stderr, "[EE] accept() failed: %s\n", strerror(errno));
			exit(EXIT_FAILURE);
		}
		fprintf(stderr, "[II] New connection from client accepted\n");


		handle_client(conn_socket);
		shutdown(conn_socket, SHUT_RDWR);
		close(conn_socket);
	}

	shutdown(listen_socket, SHUT_RDWR);
	close(listen_socket);
}




void handle_client(int sockfd)
{
	while (1) {
		time_t now = 0;
		int n = recvfrom(sockfd, &now, sizeof (now), MSG_WAITALL, NULL, 0);
		if (n < 0) {
			fprintf(stderr, "[EE] read() error: %s\n", strerror(errno));
			continue;
		} else if (n == 0) {
			fprintf(stderr, "[EE] read() returns zero\n");
			break;
		} else {
			char buffer[32] = { 0 }; /* man ctime: "stores the string in a user-supplied buffer which should have room for at least 26 bytes." */
			fprintf(stderr, "[II] Current time is: '%s'\n", ctime_r(&now, buffer));
		}
	}
}
