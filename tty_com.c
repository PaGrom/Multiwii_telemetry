#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "tty_com.h"

telem telemetry;

int set_interface_attribs(int fd, int speed, int parity) {

	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		printf("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // ignore break signal
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("error %d from tcsetattr", errno);
		return -1;
	}

	return 0;
}

void set_blocking(int fd, int should_block) {

	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		printf("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf("error %d setting term attributes", errno);
}

int read_ax(char *buf) {
	return ((buf[6])<<8) | (buf[5]&0xff);
}

int read_ay(char *buf) {
	return ((buf[8])<<8) | (buf[7]&0xff);
}

int read_az(char *buf) {
	return ((buf[10])<<8) | (buf[9]&0xff);
}

int read_gx(char *buf) {
	return ((buf[12])<<8) | (buf[11]&0xff);
}

int read_gy(char *buf) {
	return ((buf[14])<<8) | (buf[13]&0xff);
}

int read_gz(char *buf) {
	return ((buf[16])<<8) | (buf[15]&0xff);
}

int read_magx(char *buf) {
	return ((buf[18])<<8) | (buf[17]&0xff);
}

int read_magy(char *buf) {
	return ((buf[20])<<8) | (buf[19]&0xff);
}

int read_magz(char *buf) {
	return ((buf[22])<<8) | (buf[21]&0xff);
}

float read_alt(char *buf) {
	return ((buf[7])<<16) | ((buf[6])<<8) | (buf[5]&0xff);
}

int read_head(char *buf) {
	return ((buf[11])<<16) | ((buf[10])<<8) | (buf[9]&0xff);
}

void read_acc_gyro_mag(int fd) {
	
	int size_buf = 24;
	char buf[size_buf];
	char ReqBuff[6];

	ReqBuff[0] = (char)('$');
	ReqBuff[1] = (char)('M');
	ReqBuff[2] = (char)('<');
	ReqBuff[3] = 0;
	ReqBuff[4] = 102;
	ReqBuff[5] = 102;

	write(fd, ReqBuff, 6);

	int n = 0;
	while (n != size_buf) {
		n += read(fd, &buf[n], size_buf - n);
		if (n == -1)
			break;
		// printf(" n = %d\n", n);
	}
	
	if (buf[0] == (char)'$') {

		// printf("%s %d\n", buf, (int)strlen(buf));
		// int i;
		// for (i = 0; i < n; i++) {
		// 	printf("%d\t%d\n", i, buf[i]&0xFF);
		// }

		// printf("\n");
		
		telemetry.ax = read_ax(buf);
		telemetry.ay = read_ay(buf);
		telemetry.az = read_az(buf);
		printf("ax = %d\n", telemetry.ax);
		printf("ay = %d\n", telemetry.ay);
		printf("az = %d\n", telemetry.az);

		printf("\n");

		telemetry.gx = read_gx(buf);
		telemetry.gy = read_gy(buf);
		telemetry.gz = read_gz(buf);
		printf("gx = %d\n", telemetry.gx);
		printf("gy = %d\n", telemetry.gy);
		printf("gz = %d\n", telemetry.gz);

		printf("\n");

		telemetry.magx = read_magx(buf);
		telemetry.magy = read_magy(buf);
		telemetry.magz = read_magz(buf);
		printf("magx = %d\n", telemetry.magx);
		printf("magy = %d\n", telemetry.magy);
		printf("magz = %d\n", telemetry.magz);

		printf("\n");
	}
}

void read_altitude(int fd) {

	int size_buf = 10;
	char buf[size_buf];
	char ReqBuff[6];

	ReqBuff[0] = (char)('$');
	ReqBuff[1] = (char)('M');
	ReqBuff[2] = (char)('<');
	ReqBuff[3] = 0;
	ReqBuff[4] = 109;
	ReqBuff[5] = 109;

	write(fd, ReqBuff, 6);

	int n = 0;
	while (n != size_buf) {
		n += read(fd, &buf[n], size_buf - n);
		if (n == -1)
			break;
		// printf(" n = %d\n", n);
	}
	
	if (buf[0] == (char)'$') {
		// printf("%s %d\n", buf, (int)strlen(buf));
		// int i;
		// for (i = 0; i < n; i++) {
		// 	printf("%d\t%d\n", i, buf[i]&0xFF);
		// }
		telemetry.alt = read_alt(buf) / 100;
		printf("alt = %f\n", telemetry.alt);
	}
}

void read_attitude(int fd) {

	int size_buf = 14;
	char buf[size_buf];
	char ReqBuff[6];

	ReqBuff[0] = (char)('$');
	ReqBuff[1] = (char)('M');
	ReqBuff[2] = (char)('<');
	ReqBuff[3] = 0;
	ReqBuff[4] = 108;
	ReqBuff[5] = 108;

	write(fd, ReqBuff, 6);
	// sleep(1);

	int n = 0;
	while (n != size_buf) {
		n += read(fd, &buf[n], size_buf - n);
		if (n == -1)
			break;
		// printf(" n = %d\n", n);
	}
	
	if (buf[0] == (char)'$') {
		// printf("%s %d\n", buf, (int)strlen(buf));
		// int i;
		// for (i = 0; i < n; i++) {
		// 	printf("%d\t%d\n", i, buf[i]&0xFF);
		// }
		telemetry.head = read_head(buf);
		printf("head = %d\n", telemetry.head);
	}
}

int main(int argc, char const *argv[]) {

	if (argc < 2) {
		printf("Please start with %s /dev/ttyS1 (for example)\n", argv[0]);
		return 0;
	}

	int fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return 0;
	}

	set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking(fd, 0);                // set no blocking

	while (1) {
		read_acc_gyro_mag(fd);
		read_altitude(fd);
		read_attitude(fd);
	}

	return 0;
}
