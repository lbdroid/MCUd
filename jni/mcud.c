#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int doHeartbeat = 1;
int run = 1;

int mcu_fd;

static pthread_mutex_t mcuwritelock;
//TODO mutex for other ports

int set_interface_attribs (int fd, int speed, int parity){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0){
		printf ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	// 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;				// disable break processing
	tty.c_lflag = 0;				// no signaling chars, no echo,
							// no canonical processing
	tty.c_oflag = 0;				// no remapping, no delays
	tty.c_cc[VMIN]  = 0;				// read doesn't block
	tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);		// shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);		// ignore modem controls,
							// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);		// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0){
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0){
		printf ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	printf ("error %d setting term attributes", errno);
}

void write_mcu(unsigned char* data, int len){
	pthread_mutex_lock(&mcuwritelock);
	write (mcu_fd, data, len);
	pthread_mutex_unlock(&mcuwritelock);
}

void *read_mcu(void * args){
	char buf[1024];
	int n;
	while (run){
		n = read(mcu_fd, buf, sizeof(buf));
		if (n > 0){
			printf("read %d bytes\n", n);
		}
		usleep(100);
	}
	return 0;
}

int main(int argc, char ** argv){
	unsigned char heartbeat[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x55, 0xfd};
	char *mcuportname = "/dev/ttyS0";
	pthread_t mcu_reader;

	mcu_fd = open (mcuportname, O_RDWR | O_NOCTTY | O_SYNC);
	if (mcu_fd < 0){
		printf ("error %d opening %s: %s", errno, mcuportname, strerror (errno));
		return -1;
	}

	set_interface_attribs (mcu_fd, B38400, 0);	// set speed to 38,400 bps, 8n1 (no parity)
	set_blocking (mcu_fd, 0);			// set no blocking
	// TODO: Should this be non-blocking reads? We know the precise packet format for the data that is
	// coming in, so we should be able to perform blocking reads based on the actual length of packets.
	// 0x8855XXXXDATAYY where XXXX is the length of DATA, and YY is a parity byte. So we should be able
	// to read the first 4 bytes, maybe 1 byte at a time for first 2 bytes, then 2 bytes length, then
	// the rest.
	//
	// set_blocking(mcu_fd, 1);
	// read(mcu_fd, buf, 1);
	// if (buff[0] != 0x88) bail;
	// read(mcu_fd, buf, 1);
	// if (buff[0] != 0x55) bail;
	// read(mcu_fd, buf, 2);
	// len = buf[0]<<8+buf[1];
	// read(mcu_fd, buf, len+1); // don't forget to add the parity bit!!

	if (pthread_create(&mcu_reader, NULL, read_mcu, NULL) != 0) return -1;
	pthread_detach(mcu_reader);

	while (run){
		if (doHeartbeat) write_mcu (heartbeat, 8);
		sleep(1);
	}
}

