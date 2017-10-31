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
		printf ("error %d from tcgetattr\n", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	// 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
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
		printf ("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

int set_blocking (int fd, int should_block){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0){
		printf ("error %d from tggetattr\n", errno);
		return -1;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout or inter-byte timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0){
		printf ("error %d setting term attributes\n", errno);
		return -1;
	}
	return 0;
}

void write_mcu(unsigned char* data, int len){
	pthread_mutex_lock(&mcuwritelock);
	write (mcu_fd, data, len);
	pthread_mutex_unlock(&mcuwritelock);
}

void dump_packet(unsigned char* data, int len){
	int i;
	printf("0x");
	for (i=0; i<len; i++) printf("%02X", data[i]);
	printf("\n");
}

void process_mcu_aflag(unsigned char value){
	//TODO
}

void key_press(unsigned char keycode){
	//TODO
}

void process_mcu_swi(unsigned char* data, int len){
	//TODO
}

void process_mcu_radio(unsigned char* data, int len){
	//TODO
}

void process_mcu_main(unsigned char* data, int len){
	//TODO
	switch(data[2]){
		case 0x88: // MCU On
			break;
		case 0x89: // MCU shutdown sequence
			switch(data[3]){
				case 0x53:
				case 0x54:
				case 0x55:
					break;
			}
			break;
		case 0x45: // Radio power on/off
			break;
		case 0x00: // MCU/ACC power state
			switch(data[3]){
				case 0x00: // MCU OFF
				case 0x01: // MCU ON
				case 0x21: // Must request MCU data
				case 0x31: // ACC ON
				case 0x32: // ACC OFF
					break;
			}
		case 0x07:
			key_press(0x0);
			break;
		case 0x0d: // Radio band
		case 0x10: // Buttons?
		case 0x11: // More buttons
		case 0x12: // Headlights OFF
		case 0x13: // Headlights ON
		case 0x21: // More buttons
		case 0x23: // Reverse ON/OFF
		case 0x24: // ebrake ON/OFF
		case 0x60: // RDS ON
			break;
	}
}

void process_mcu(unsigned char* data, int len){
	switch(data[0]){
		case 0xc0:
		case 0xc3:
		case 0xc4:
			process_mcu_swi(data, len);
			break;
		case 0x01:
			switch(data[1]){
				case 0xd3:
				case 0x03:
					process_mcu_radio(data, len);
					break;
				case 0x00:
					process_mcu_main(data, len);
					break;
				case 0x07:
				case 0x10:
				case 0x38:
					process_mcu_swi(data, len);
					break;
				default:
					printf ("MCU READ Unhandled Packet: ");
					dump_packet(data, len);
			}
			break;
		case 0x06:
			//TODO if (data[1] == 0x20) resetArmLaterCmd(data[2]);
			break;
		case 0x0a:
			process_mcu_aflag(data[1]);
			break;
		case 0x21:
		case 0x50:
		case 0x51:
			process_mcu_radio(data, len);
			break;
		default:
			printf ("MCU READ Unhandled Packet: ");
			dump_packet(data, len);			
	}
}

void *read_mcu(void * args){
	unsigned char buf[1024];
	int n;
	int size;
	int i;
	int cs;
	while (run){
		// First 2 bytes: header 0x8855
		// Second 2 bytes: data length
		// Next N bytes: data
		// Last byte: checksum of length through data

		n = read(mcu_fd, buf, 1);
		if (n != 1 || buf[0] != 0x88) continue;

		n = read(mcu_fd, buf+1, 1);
		if (n != 1 || buf[1] != 0x55) continue;

		n = read(mcu_fd, buf+2, 2);
		if (n != 2) continue;

		size = (((int)buf[2])<<8) + buf[3];
		n = read(mcu_fd, buf+4, size+1);
		if (n < size+1) continue;

		cs = 0;
		for (i=2; i<size+4; i++){
			cs ^= buf[i];
		}
		if (cs != buf[size+4]) continue;

		printf("read %d bytes, checksum valid\n", size+5);
		process_mcu(&buf[2], size);
	}
	return 0;
}

int main(int argc, char ** argv){
	unsigned char heartbeat[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x55, 0xfd};
	char *mcuportname = "/dev/ttyS0";
	pthread_t mcu_reader;

	mcu_fd = open (mcuportname, O_RDWR | O_NOCTTY | O_SYNC);
	if (mcu_fd < 0){
		printf ("error %d opening %s: %s\n", errno, mcuportname, strerror (errno));
		return -1;
	}

	if (set_interface_attribs (mcu_fd, B38400, 0) < 0) return -1;	// set speed to 38,400 bps, 8n1 (no parity)
	if (set_blocking (mcu_fd, 1) < 0) return -1;			// set blocking reads

	if (pthread_create(&mcu_reader, NULL, read_mcu, NULL) != 0) return -1;
	pthread_detach(mcu_reader);

	while (run){
		if (doHeartbeat) write_mcu (heartbeat, 8);
		sleep(1);
	}
}

