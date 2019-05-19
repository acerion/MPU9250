#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>
#include <unistd.h>

#include <errno.h>
#include <getopt.h>

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>




#include "../MPU9250/MPU9250.h"



/* https://www.cmrr.umn.edu/~strupp/serial.html */




#define STATE_TEXT     0
#define STATE_DATA     1




struct {
	struct received {
		uint8_t header[HEADER_SIZE];

		data_t data_a;
		data_t data_b;
	} __attribute__((packed)) data;

	size_t i;
	int state;
	uint32_t previous_counter;
} receiver;




static uint8_t expected_header[HEADER_SIZE] = { HEADER_BEGIN, HEADER_BYTE_1, HEADER_BYTE_2, HEADER_BYTE_3, HEADER_BYTE_4, HEADER_BYTE_5 };
static char file_name[64] = "/dev/ttyUSB0";




static int configure_fd(int fd);
static void handle_byte(uint8_t c);
static void handle_received_data(struct received * data);
static void print_data_locally(data_t * data, char id);
static bool is_checksum_valid(data_t * data);




static void close_socket(void);
static int create_socket(const char * server_address, int server_port);
static void send_to_ahrs_engine(int sock, const uint8_t * data, size_t size);

static bool g_send_to_ahrs_engine = false;
static int g_ahrs_engine_socket = 0;
static char * g_ahrs_engine_address = "127.0.0.1";
static int g_ahrs_engine_port = 4567;




int main(int argc, char ** argv)
{
	int opt;
	while ((opt = getopt(argc, argv, "eh")) != -1) {
		switch (opt) {
		case 'e':
			g_send_to_ahrs_engine = true;
			break;
		case 'h':
		default:
			fprintf(stderr, "Usage: %s [-e]\n", argv[0]);
			fprintf(stderr, "        -e: send data to AHRS engine over network\n");
			exit(EXIT_FAILURE);
		}
	}

	if (g_send_to_ahrs_engine) {
		g_ahrs_engine_socket = create_socket(g_ahrs_engine_address, g_ahrs_engine_port);
		if (g_ahrs_engine_socket == -1) {
			exit(EXIT_FAILURE);
		}
		atexit(close_socket);
	}



	int fd = open(file_name, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "[EE] Can't open '%s': %s\n", file_name, strerror(errno));
		exit(EXIT_FAILURE);
	}

	configure_fd(fd);

	do {
		uint8_t c;
		const int n_read = read(fd, &c, 1);
		if (n_read > 0) {
			switch (receiver.state) {
			case STATE_DATA:
				handle_byte(c);
				break;
			case STATE_TEXT:
			default:
				if (c == HEADER_BEGIN) {
					handle_byte(c);
				} else {
					fprintf(stderr, "%c", c);
				}
				break;
			}
		} else if (n_read < 0) {
			fprintf(stderr, "[EE] read() error: %d/ %s\n", n_read, strerror(errno));
		} else {  /* n_read == 0 */
			fprintf(stderr, "[EE] read() timeout\n");
		}
	} while (1);

	exit(EXIT_SUCCESS);
}




void handle_byte(uint8_t c)
{
	if (receiver.state == STATE_TEXT) {
		if (HEADER_BEGIN != c) {
			fprintf(stderr, "[EE] Expected beginning of header, received 0x%02x\n", c);
			return;
		}
	}
	receiver.state = STATE_DATA;

	uint8_t * bytes = (uint8_t *) &receiver.data;
	const size_t data_size = sizeof (receiver.data);

	bytes[receiver.i] = c;
	receiver.i++;

	if (receiver.i == HEADER_SIZE) {
		if (0 != memcmp(receiver.data.header, expected_header, HEADER_SIZE)) {
			fprintf(stderr, "[EE] Failed to capture header: %x %x %x %x %x %x, going back to TEXT state\n",
				receiver.data.header[0],
				receiver.data.header[1],
				receiver.data.header[2],
				receiver.data.header[3],
				receiver.data.header[4],
				receiver.data.header[5]);

			/* Reset receiver. */
			receiver.state = STATE_TEXT;
			receiver.i = 0;
			memset(receiver.data.header, 0, HEADER_SIZE);
		} else {
			/* Received correct header, start receiving data. */
		}

	} else if (receiver.i == data_size) {
		handle_received_data(&receiver.data);

		/* Reset receiver. */
		receiver.i = 0;
		receiver.state = STATE_TEXT;
		memset(&receiver.data, 0, data_size);
	} else {
		; /* Continue receiving header or data. */
	}

	return;
}




void handle_received_data(struct received * received)
{
	if (0 != memcmp(&received->data_a, &received->data_b, sizeof (data_t))) {
		fprintf(stderr, "[EE] Data contents mismatch\n");
	}
	if (received->data_a.checksum != received->data_b.checksum) {
		fprintf(stderr, "[EE] Checksum bytes mismatch (checksum A = 0x%02x, checksum B = 0x%02x)\n",
			received->data_a.checksum, received->data_b.checksum);
	}


	data_t * data = NULL;
	char id = ' ';
	bool counter_ok;

	if (is_checksum_valid(&received->data_a)) {
		data = &received->data_a;
		id = 'a';
		goto label_valid_data;
	}
	fprintf(stderr, "[EE] Checksum of data A can't be verified\n");

	if (is_checksum_valid(&received->data_b)) {
		data = &received->data_b;
		id = 'b';
		goto label_valid_data;
	}
	fprintf(stderr, "[EE] Checksum of data B can't be verified\n");

	return;


 label_valid_data:

	counter_ok = (data->counter == receiver.previous_counter + 1);
	if (!counter_ok) {
		fprintf(stderr, "[EE] Missing %d packets\n", data->counter - (receiver.previous_counter + 1));
	}
	receiver.previous_counter = data->counter;



	print_data_locally(data, id);



	if (g_send_to_ahrs_engine) {
		time_t now = time(0);
		send_to_ahrs_engine(g_ahrs_engine_socket, (uint8_t *) &now, sizeof (now));
	}


	return;
}




bool is_checksum_valid(data_t * data)
{
	uint8_t calculated = 0x00;
	const uint8_t * bytes = (uint8_t *) data;

	for (size_t i = 0; i < sizeof (data_t) - 1; i++) { /* -1: don't include checksum byte. */
		calculated ^= bytes[i];
	}

	if (calculated != data->checksum) {
		fprintf(stderr, "[EE] Checksum mismatch: calculated 0x%02x != received 0x%02x\n", calculated, data->checksum);
		return false;
	} else {
		//fprintf(stderr, "[II] Checksum match: calculated 0x%02x == received 0x%02x\n", calculated, data->checksum);
		return true;
	}
}




void print_data_locally(data_t * data, char id)
{
	static uint32_t timestamp_previous = 0;
	const uint32_t delta = data->timestamp - timestamp_previous;


	fprintf(stderr, "Data %c,  "
		"counter: %12lu  "
		"time: %6lu [us]/%5.1f [Hz]  | "
		"acc:  %12.6f  %12.6f  %12.6f  | "
		"gyro: %11.6f  %11.6f  %11.6f  | "
		"mag (%s):  %11.6f  %11.6f  %11.6f  | "
		"temp: %6.2f\n",
		id,
		(long unsigned) data->counter,
		(long unsigned) delta,
		(1.0 / (delta / 1000000.0)),
		1000.0 * data->ax, 1000.0 * data->ay, 1000.0 * data->az,
		data->gx, data->gy, data->gz,
		data->new_mag_data_ready ? "new" : "old", data->mx, data->my, data->mz,
		data->imu_temperature);

	timestamp_previous = data->timestamp;
}




int configure_fd(int fd)
{
	struct termios tty;
	memset(&tty, 0, sizeof (tty));

	if (0 > tcgetattr(fd, &tty)) {
		fprintf(stderr, "[EE] tcgetattr(): %s\n", strerror(errno));
		return -1;
	}

	const speed_t speed = B115200;
	cfsetospeed(&tty, (speed_t) speed);
	cfsetispeed(&tty, (speed_t) speed);

	/* Enable the receiver and set local mode. */
	tty.c_cflag |= (CLOCAL | CREAD);

	/* 8N1: no parity, 1 stop bit, 8 bits data. */
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	/* Disable hardware flow control. */
	//tty.c_cflag &= ~CRTSCTS;

	/* Local options: raw mode. */
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* Input options: no software flow control. */
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	/* Output options: raw output (do we really need output options in this program?). */
	tty.c_oflag &= ~OPOST;

	/* Minimum number of characters to read. */
	tty.c_cc[VMIN] = 1;

	/* Time to wait for first character. */
	tty.c_cc[VTIME] = 1;

	if (0 != tcsetattr(fd, TCSANOW, &tty)) {
		fprintf(stderr, "[EE] tcsetattr(): %s\n", strerror(errno));
		return -1;
	}

	return 0;
}




void close_socket(void)
{
	if (0 != g_ahrs_engine_socket) {
		shutdown(g_ahrs_engine_socket, SHUT_RDWR);
		close(g_ahrs_engine_socket);
		g_ahrs_engine_socket = 0;
	}
}




int create_socket(const char * server_address, int server_port)
{
	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (-1 == sock) {
		fprintf(stderr, "[EE] socket() failed: %s\n", strerror(errno));
		return -1;
	}


	struct sockaddr_in servaddr;
	memset(&servaddr, 0, sizeof (servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(server_address);
	servaddr.sin_port = htons(server_port);


	if (0 != connect(sock, (struct sockaddr *) &servaddr, sizeof (servaddr))) {
		fprintf(stderr, "[EE] connect() failed: %s\n", strerror(errno));
		return -1;
	}
	fprintf(stderr, "[II] Connected to server\n");


	return sock;
}




void send_to_ahrs_engine(int sock, const uint8_t * buffer, size_t size)
{
	if (sock <= 0) {
		fprintf(stderr, "[EE] Invalid socket\n");
		return;
	}

	const int n = write(sock, buffer, size);
	if (n < 0) {
		fprintf(stderr, "[EE] write() failed: %s\n", strerror(errno));
	} else if (n == 0) {
		fprintf(stderr, "[NN] Can't write to socket\n");
	} else {
		;
	}
}
