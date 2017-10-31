#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define USB_MODE_DEVICE 0
#define USB_MODE_HOST 1

unsigned char heartbeat[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x55, 0xfd};
unsigned char sleep1[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x5f, 0xf7};
unsigned char sleep2[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x61, 0xc9};
unsigned char sleep3[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x62, 0xca};

int do_heartbeat = 1;
int run = 1;

int sleeptick = 0;
int resume_wifi_on_wake = 0;
int resume_bt_on_wake = 0;
int mcu_on = 1;
int acc_on = 1;

int mcu_fd;
int bd_fd;
int amp_fd;

static pthread_mutex_t mcuwritelock;
static pthread_mutex_t bdwritelock;
static pthread_mutex_t ampwritelock;

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

void set_usb_mode(int mode){
	//TODO
}

int pack_frame(unsigned char* data, unsigned char* buffer, int len){
	int i;
	unsigned char cs = 0;

	if (sizeof(buffer) < len+5) return -1;

	buffer[0] = 0x88;
	buffer[1] = 0x55;
	buffer[2] = len >> 8;
	buffer[3] = len & 0xff;
	cs = buffer[2]^buffer[3];
	
	for (i=0; i<len; i++){
		buffer[i+4] = data[i];
		cs ^= data[i];
	}

	buffer[len+4] = cs;
	return 1;
}

void request_mcu_data(){
	unsigned char cmd1[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x60, 0xc8};
	unsigned char cmd2[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0x00, 0x02};
	write_mcu(cmd1, 8);
	write_mcu(cmd2, 8);
}

void set_mcu_on(int on){
	if (mcu_on != on){
		mcu_on = on;
		if (on == 1){
			request_mcu_data();
			set_usb_mode(USB_MODE_HOST);
		} else
			set_usb_mode(USB_MODE_DEVICE);
	}
}

void set_acc_on(int on){
	if (acc_on != on){
		acc_on = on;
		//TODO ToolsJni.cmd_29_acc_state_to_bsp(on == 0 ? 0 : 1);
		//TODO bluetooth on/off below should create a thread with a delay to avoid rapid on/off during ignition

		if (on == 1){
			if (resume_bt_on_wake == 1) system("service call bluetooth_manager 6"); // turn bluetooth ON
			system("am broadcast -a tk.rabidbeaver.maincontroller.ACC_ON");
		} else {
			if (system("dumpsys bluetooth_manager | busybox grep \"^  state:\" | busybox grep ON") == 0){
				resume_bt_on_wake = 1;
				system("service call bluetooth_manager 8"); // turn bluetooth OFF
			} else
				resume_bt_on_wake = 0;
			system("am broadcast -a tk.rabidbeaver.maincontroller.ACC_OFF");
		}
	}
}

void set_headlights_on(int on){
	//TODO
}

void set_ebrake_on(int on){
	//TODO
}

void set_reverse_on(int on){
	//TODO
}

void process_mcu_main(unsigned char* data, int len){
	switch(data[2]){
		case 0x88: // MCU On
			sleeptick = 0;
			if (resume_wifi_on_wake == 1){
				system("svc wifi enable");
				resume_wifi_on_wake = 0;
			}
			break;
		case 0x89: // MCU shutdown sequence
			switch(data[3]){
				case 0x53:
					do_heartbeat = 0;
					sleeptick++;
					if (sleeptick == 1){
						if (system("dumpsys wifi | busybox grep \"^Wi-Fi is enabled\"") == 0){ // returns 0 when wifi is enabled, 256 when disabled.
							// wifi is enabled. Store this and disable.
							resume_wifi_on_wake = 1;
							system("svc wifi disable");
						} else
							resume_wifi_on_wake = 0;
					}
					if (sleeptick > 4) write_mcu(sleep1, 8);
					break;
				case 0x54:
					sleeptick = 0;
					write_mcu(sleep2, 8);
				case 0x55:
					sleep(1);
					write_mcu(sleep3, 8);
					system("input keyevent 26");
					break;
			}
			break;
		case 0x45: // Radio power on/off
			break;
		case 0x00: // MCU/ACC power state
			switch(data[3]){
				case 0x00: // MCU OFF
					set_mcu_on(0);
					return;
				case 0x01: // MCU ON
					set_mcu_on(1);
					do_heartbeat = 1;
					return;
				case 0x21: // Must request MCU data
					request_mcu_data();
					return;
				case 0x31: // ACC ON
					set_acc_on(1);
					return;
				case 0x32: // ACC OFF
					set_acc_on(0);
					break;
			}
		case 0x07:
			key_press(0x0);
			break;
		case 0x0d: // Radio band TODO
		case 0x10: // Buttons? TODO
		case 0x11: // More buttons TODO
			break;
		case 0x12: // Headlights OFF
			set_headlights_on(0);
			break;
		case 0x13: // Headlights ON
			set_headlights_on(1);
			break;
		case 0x21: // More buttons TODO
			break;
		case 0x23: // Reverse ON/OFF
			if (data[3] == 0x02) set_reverse_on(0);
			else if (data[3] == 0x03) set_reverse_on(1);
			break;
		case 0x24: // ebrake ON/OFF
			set_ebrake_on(data[3] & 1);
			break;
		case 0x60: // RDS ON TODO
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

/* Interfacing requirements;
 *
 * Hardware: MCU (serial /dev/ttyS0), BD37033 (i2c /dev/i2c-4), AMP (serial /dev/ttyS1)
 * HALs: audio, car, lights, power, radio
 *
 * For HALs, will create unix domain sockets in /dev/car/, each named for their particular HAL,
 * except for the car HAL, which will be named "main". Example: /dev/car/main
 *
 * Threads: Require one reader thread for each device and each HAL. In addition, this main thread
 * will perform heartbeat and maintenance.
 *
 */

	while (run){
		if (do_heartbeat) write_mcu (heartbeat, 8);
		sleep(1);
	}
}

