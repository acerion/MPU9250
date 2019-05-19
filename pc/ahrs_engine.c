#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>

#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>




#include "../MPU9250/MPU9250.h"




#define DEFAULT_SERVER_PORT    4567

#ifndef M_PI
#define M_PI 3.14159265359
#endif




typedef struct {
	float yaw;
	float pitch;
	float roll;

	/* Linear acceleration (acceleration with gravity component subtracted). */
	float lin_ax;
	float lin_ay;
	float lin_az;
} ahrs_t;




// Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
static const float local_declination = 13.8;
static float g_quaternions[4] = { 1.0f, 0.0f, 0.0f, 0.0f };




/* Read IMU datasets from network socket and process them further. */
static void handle_incoming_imu_datasets_stream(int sockfd);

/* Calculate new values of quaternions from given IMU @dataset. */
static void update_quaternions(float * quaternions, const imu_dataset_t * dataset, float filter_time_delta_s);

/* Use given @quaternions and current IMU @dataset to calculate
   current yaw/pitch/roll values and put them in @ahrs. */
static void calculate_ahrs_from_quaternions(float * quaternions, const imu_dataset_t * dataset, ahrs_t * ahrs);

/* Print contents of @ahrs to @file. */
static void print_ahrs(FILE * file, ahrs_t * ahrs);




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


		handle_incoming_imu_datasets_stream(conn_socket);
		shutdown(conn_socket, SHUT_RDWR);
		close(conn_socket);
	}

	shutdown(listen_socket, SHUT_RDWR);
	close(listen_socket);

	exit(EXIT_SUCCESS);
}




void handle_incoming_imu_datasets_stream(int sockfd)
{
	static uint32_t timestamp_previous_us = 0;

	while (1) {
		imu_dataset_t imu_dataset;
		int n = recvfrom(sockfd, &imu_dataset, sizeof (imu_dataset_t), MSG_WAITALL, NULL, 0);
		if (n < 0) {
			fprintf(stderr, "[EE] read() error: %s\n", strerror(errno));
			continue;
		} else if (n == 0) {
			fprintf(stderr, "[EE] read() returns zero\n");
			break;
		} else {
			const uint32_t delta_t_us = imu_dataset.timestamp_us - timestamp_previous_us; /* [microseconds] */
			const float delta_t_s = delta_t_us / 1000000.0; /* [seconds] */
#if 0
			fprintf(stderr, "[II] IMU dataset: "
				"counter: %12lu "
				"time: %6lu [us]/%5.1f [Hz] "
				"temp: %6.2f\n",
				(long unsigned) imu_dataset.counter,
				(long unsigned) delta_t_us,
				(1.0 / (delta_t_s)),
				imu_dataset.imu_temperature);
#endif
#if 1
			ahrs_t ahrs = { 0 };
			update_quaternions(g_quaternions, &imu_dataset, delta_t_s);
			calculate_ahrs_from_quaternions(g_quaternions, &imu_dataset, &ahrs);
			print_ahrs(stderr, &ahrs);
#endif
			timestamp_previous_us = imu_dataset.timestamp_us;
		}
	}
}




void print_ahrs(FILE * file, ahrs_t * ahrs)
{
	fprintf(file, "yaw/pitch/roll = %12.6f   %12.6f   %12.6f\n", ahrs->yaw, ahrs->pitch, ahrs->roll);
}




void calculate_ahrs_from_quaternions(float * q, const imu_dataset_t * dataset, ahrs_t * ahrs)
{
}




void update_quaternions(float * quaternions, const imu_dataset_t * dataset, float filter_time_delta_s)
{
}
