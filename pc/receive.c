#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>
#include <unistd.h>

#include <errno.h>

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>




/* https://www.cmrr.umn.edu/~strupp/serial.html */





#define STATE_TEXT     0
#define STATE_DATA     1

#define HEADER_BEGIN   0x17
#define HEADER_SIZE       6


typedef struct {
	float ax;
	float ay;
	float az;

	float gx;
	float gy;
	float gz;

	float mx;
	float my;
	float mz;

	float temperature;

	uint8_t new_mag_data_ready;
	uint32_t timestamp; /* [microseconds] */
} __attribute__((packed)) payload_t;


struct {
	struct receiver_data {
		uint8_t header[HEADER_SIZE];

		payload_t payload_a;
		uint8_t checksum_a;

		payload_t payload_b;
		uint8_t checksum_b;
	} __attribute__((packed)) data;

	size_t i;
	int state;
} receiver;




static uint8_t expected_header[HEADER_SIZE] = { HEADER_BEGIN, 0x6b, 0x61, 0x6d, 0x69, 0x6c };
static char file_name[64] = "/dev/ttyUSB0";




static int configure_fd(int fd);
static void handle_binary(uint8_t c);
static void handle_payloads(void);
static void print_payload(char p, payload_t * payload);
static bool is_checksum_valid(payload_t * payload, uint8_t checksum);




int main(void)
{
	int fd = open(file_name, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "[EE] Can't open '%s': %s\n", file_name, strerror(errno));
		return -1;
	}

	configure_fd(fd);

	do {
		uint8_t c;
		const int n_read = read(fd, &c, 1);
		if (n_read > 0) {
			switch (receiver.state) {
			case STATE_DATA:
				handle_binary(c);
				break;
			case STATE_TEXT:
			default:
				if (c == HEADER_BEGIN) {
					handle_binary(c);
				} else {
					//fprintf(stderr, "%c", c);
				}
				break;
			}
		} else if (n_read < 0) {
			fprintf(stderr, "[EE] read() error: %d/ %s\n", n_read, strerror(errno));
		} else {  /* n_read == 0 */
			fprintf(stderr, "[EE] read() timeout\n");
		}
	} while (1);

	return 0;
}




void handle_binary(uint8_t c)
{
	if (receiver.state == STATE_TEXT) {
		if (HEADER_BEGIN != c) {
			fprintf(stderr, "[EE] Expected beginning of header, received 0x%02x\n", c);
			return;
		}
	}
	receiver.state = STATE_DATA;

	uint8_t * data = (uint8_t *) &receiver.data;
	const size_t data_size = sizeof (receiver.data);

	data[receiver.i] = c;
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
			/* Received correct header, start receiving payload. */
		}

	} else if (receiver.i == data_size) {
		handle_payloads();

		/* Reset receiver. */
		receiver.i = 0;
		receiver.state = STATE_TEXT;
		memset(&receiver.data, 0, data_size);
	} else {
		; /* Continue receiving header or payload. */
	}

	return;
}




void handle_payloads(void)
{
	if (0 != memcmp(&receiver.data.payload_a, &receiver.data.payload_b, sizeof (payload_t))) {
		fprintf(stderr, "[EE] Payload contents mismatch\n");
	}
	if (receiver.data.checksum_a != receiver.data.checksum_b) {
		fprintf(stderr, "[EE] Checksum bytes mismatch (checksum A = 0x%02x, checksum B = 0x%02x)\n",
			receiver.data.checksum_a, receiver.data.checksum_b);
	}

	if (is_checksum_valid(&receiver.data.payload_a, receiver.data.checksum_a)) {
		print_payload('a', &receiver.data.payload_a);
		return;
	}
	fprintf(stderr, "[EE] Checksum of payload A can't be verified\n");

	if (is_checksum_valid(&receiver.data.payload_b, receiver.data.checksum_b)) {
		print_payload('b', &receiver.data.payload_b);
		return;
	}
	fprintf(stderr, "[EE] Checksum of payload B can't be verified\n");
}




bool is_checksum_valid(payload_t * payload, uint8_t received)
{
	uint8_t calculated = 0x00;
	const uint8_t * data = (uint8_t *) payload;

	for (size_t i = 0; i < sizeof (payload_t); i++) {
		calculated ^= data[i];
	}

	if (calculated != received) {
		fprintf(stderr, "[EE] Checksum mismatch: calculated 0x%02x != received 0x%02x\n", calculated, received);
		return false;
	} else {
		//fprintf(stderr, "[II] Checksum match: calculated 0x%02x == received 0x%02x\n", calculated, received);
		return true;
	}
}




void print_payload(char id, payload_t * p)
{
	static uint32_t timestamp_previous = 0;
	const uint32_t delta = p->timestamp - timestamp_previous;

	fprintf(stderr, "payload %c  "
		"time: %15lu/%6lu [us]/%5.1f [Hz]  | "
		"acc:  %12.6f  %12.6f  %12.6f  | "
		"gyro: %11.6f  %11.6f  %11.6f  | "
		"mag (%s):  %11.6f  %11.6f  %11.6f  | "
		"temp: %6.2f\n",
		id,
		(long unsigned) p->timestamp, (long unsigned) delta,
		(1.0 / (delta / 1000000.0)),
		1000.0 * p->ax, 1000.0 * p->ay, 1000.0 * p->az,
		p->gx, p->gy, p->gz,
		p->new_mag_data_ready ? "new" : "old", p->mx, p->my, p->mz,
		p->temperature);

	timestamp_previous = p->timestamp;
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
