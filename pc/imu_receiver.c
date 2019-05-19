#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>




#include "../MPU9250/MPU9250.h"




/* https://www.cmrr.umn.edu/~strupp/serial.html */




#define STATE_TEXT     0
#define STATE_DATA     1




typedef struct {
	int state;
	size_t i;
	uint32_t previous_counter;

	struct received {
		uint8_t header[HEADER_SIZE];

		imu_dataset_t dataset_a;
		imu_dataset_t dataset_b;
	} __attribute__((packed)) received;
} receiver;

static receiver g_receiver;




static uint8_t expected_header[HEADER_SIZE] = { HEADER_BEGIN, HEADER_BYTE_1, HEADER_BYTE_2, HEADER_BYTE_3, HEADER_BYTE_4, HEADER_BYTE_5 };

static char g_serial_file_full_path[64] = "/dev/ttyUSB0";
static int g_serial_fd = 0;



static void close_serial_fd(void);
static int get_serial_fd(const char * full_path);
static void handle_byte(receiver * rec, uint8_t c);
static void handle_received_data(receiver * rec);
static void print_dataset_locally(imu_dataset_t * dataset, char id);
static bool is_checksum_valid(imu_dataset_t * dataset);
static void reset_receiver(receiver * rec);




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


	g_serial_fd = get_serial_fd(g_serial_file_full_path);
	if (g_serial_fd <= 0) {
		fprintf(stderr, "[EE] Can't open serial line file '%s'\n", g_serial_file_full_path);
		exit(EXIT_FAILURE);
	}
	atexit(close_serial_fd);


	do {
		uint8_t c;
		const int n_read = read(g_serial_fd, &c, 1);
		if (n_read > 0) {
			switch (g_receiver.state) {
			case STATE_DATA:
				handle_byte(&g_receiver, c);
				break;
			case STATE_TEXT:
			default:
				if (c == HEADER_BEGIN) {
					handle_byte(&g_receiver, c);
				} else {
					fprintf(stderr, "%c", c);
				}
				break;
			}
		} else if (n_read < 0) {
			fprintf(stderr, "[EE] read() error: %d/ %s\n", n_read, strerror(errno));
		} else {  /* n_read == 0 */
			fprintf(stderr, "[EE] read() timeout\n");
			if (0 != access(g_serial_file_full_path, F_OK)) {
				fprintf(stderr, "[EE] Serial line file no longer exists\n");
				goto loop_err;
			}
			sleep(1); /* To avoid fast looping on error. */
		}
	} while (1);


	exit(EXIT_SUCCESS);

 loop_err:
	exit(EXIT_FAILURE);
}




void handle_byte(receiver * rec, uint8_t c)
{
	if (rec->state == STATE_TEXT) {
		if (HEADER_BEGIN != c) {
			fprintf(stderr, "[EE] Expected beginning of header, received 0x%02x\n", c);
			return;
		}
	}
	rec->state = STATE_DATA;

	uint8_t * bytes = (uint8_t *) &rec->received;
	const size_t data_size = sizeof (rec->received);

	bytes[rec->i] = c;
	rec->i++;

	if (rec->i == HEADER_SIZE) {
		if (0 != memcmp(rec->received.header, expected_header, HEADER_SIZE)) {
			fprintf(stderr, "[EE] Failed to capture header: %x %x %x %x %x %x, going back to TEXT state\n",
				rec->received.header[0],
				rec->received.header[1],
				rec->received.header[2],
				rec->received.header[3],
				rec->received.header[4],
				rec->received.header[5]);

			reset_receiver(rec);
		} else {
			/* Received correct header, start receiving data. */
		}

	} else if (rec->i == data_size) {
		/* Full set of data (one packet) received. */
		handle_received_data(rec);
		reset_receiver(rec);
	} else {
		; /* Continue receiving header or data. */
	}

	return;
}




void reset_receiver(receiver * rec)
{
	rec->state = STATE_TEXT;
	rec->i = 0;
	/* Don't touch ::previous_counter. */
	memset(&rec->received, 0, sizeof (rec->received));
}




void handle_received_data(receiver * rec)
{
	if (0 != memcmp(&rec->received.dataset_a, &rec->received.dataset_b, sizeof (imu_dataset_t))) {
		fprintf(stderr, "[EE] Dataset contents mismatch\n");
	}
	if (rec->received.dataset_a.checksum != rec->received.dataset_b.checksum) {
		fprintf(stderr, "[EE] Checksum bytes mismatch (checksum A = 0x%02x, checksum B = 0x%02x)\n",
			rec->received.dataset_a.checksum, rec->received.dataset_b.checksum);
	}


	imu_dataset_t * dataset = NULL;
	char id = ' ';
	bool counter_ok;

	if (is_checksum_valid(&rec->received.dataset_a)) {
		dataset = &rec->received.dataset_a;
		id = 'a';
		goto label_valid_data;
	}
	fprintf(stderr, "[EE] Checksum of dataset A can't be verified\n");

	if (is_checksum_valid(&rec->received.dataset_b)) {
		dataset = &rec->received.dataset_b;
		id = 'b';
		goto label_valid_data;
	}
	fprintf(stderr, "[EE] Checksum of dataset B can't be verified\n");

	return;


 label_valid_data:

	counter_ok = (dataset->counter == rec->previous_counter + 1);
	if (!counter_ok) {
		fprintf(stderr, "[EE] Missing %d packets\n", dataset->counter - (rec->previous_counter + 1));
	}
	rec->previous_counter = dataset->counter;



	print_dataset_locally(dataset, id);



	if (g_send_to_ahrs_engine) {
		send_to_ahrs_engine(g_ahrs_engine_socket, (uint8_t *) dataset, sizeof (imu_dataset_t));
	}


	return;
}




bool is_checksum_valid(imu_dataset_t * dataset)
{
	uint8_t calculated = 0x00;
	const uint8_t * bytes = (uint8_t *) dataset;

	for (size_t i = 0; i < sizeof (imu_dataset_t) - 1; i++) { /* -1: don't include checksum byte. */
		calculated ^= bytes[i];
	}

	if (calculated != dataset->checksum) {
		fprintf(stderr, "[EE] Checksum mismatch: calculated 0x%02x != received 0x%02x\n", calculated, dataset->checksum);
		return false;
	} else {
		//fprintf(stderr, "[II] Checksum match: calculated 0x%02x == received 0x%02x\n", calculated, dataset->checksum);
		return true;
	}
}




void print_dataset_locally(imu_dataset_t * dataset, char id)
{
	static uint32_t timestamp_previous = 0;
	const uint32_t delta = dataset->timestamp - timestamp_previous;


	fprintf(stderr, "Dataset %c, "
		"counter: %12lu  "
		"time: %6lu [us]/%5.1f [Hz] | "
		"acc:  %12.6f  %12.6f  %12.6f | "
		"gyro: %11.6f  %11.6f  %11.6f | "
		"mag (%s):  %11.6f  %11.6f  %11.6f | "
		"temp: %6.2f\n",
		id,
		(long unsigned) dataset->counter,
		(long unsigned) delta,
		(1.0 / (delta / 1000000.0)),
		1000.0 * dataset->ax, 1000.0 * dataset->ay, 1000.0 * dataset->az,
		dataset->gx, dataset->gy, dataset->gz,
		dataset->new_mag_data_ready ? "new" : "old", dataset->mx, dataset->my, dataset->mz,
		dataset->imu_temperature);

	timestamp_previous = dataset->timestamp;
}




int get_serial_fd(const char * full_path)
{
	int fd = open(full_path, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "[EE] Can't open '%s': %s\n", full_path, strerror(errno));
		return -1;
	}

	struct termios tty;
	memset(&tty, 0, sizeof (tty));

	if (tcgetattr(fd, &tty) < 0) {
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

	return fd;
}




void close_serial_fd(void)
{
	if (0 != g_serial_fd) {
		close(g_serial_fd);
		g_serial_fd = 0;
	}
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
