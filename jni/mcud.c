#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/un.h>

#define USB_MODE_DEVICE 0
#define USB_MODE_HOST 1

unsigned char heartbeat[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x55, 0xfd};
unsigned char sleep1[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x5f, 0xf7};
unsigned char sleep2[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x61, 0xc9};
unsigned char sleep3[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x62, 0xca};

unsigned char oxaflag = 0x00;
unsigned char KEY_ADC[50] = {0x0};
unsigned char SCAN_ADC[6] = {0x0};
unsigned char swi_detect = 0x0;
unsigned char swi_adc = 0x0;

int do_heartbeat = 1;
int run = 1;

int sleeptick = 0;
int resume_wifi_on_wake = 0;
int resume_bt_on_wake = 0;
int mcu_on = 1;
int acc_on = 1;
int headlights_on = 0;
int ebrake_on = 0;
int reverse_on = 0;

int mcu_fd;
int bd_fd;
int amp_fd;

int key_nohal_fd;
int key_nohal_cl_fd;
int audio_hal_fd;
int audio_hal_cl_fd;
int car_hal_fd;
int car_hal_cl_fd;
int lights_hal_fd;
int lights_hal_cl_fd;
int power_hal_fd;
int power_hal_cl_fd;
int radio_hal_fd;
int radio_hal_cl_fd;

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
	tty.c_cc[VMIN]  = 0xff;				// read blocks for min 255 bytes
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

int create_socket(char *path){
	int fd;
	struct sockaddr_un sun;

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) return -1;
	if (fcntl(fd, F_SETFD, FD_CLOEXEC)) {
		close(fd);
		return -1;
	}

	memset(&sun, 0, sizeof(sun));
	sun.sun_family = AF_LOCAL;
	memset(sun.sun_path, 0, sizeof(sun.sun_path));
	snprintf(sun.sun_path, sizeof(sun.sun_path), "%s", path);

	unlink(sun.sun_path);

	if (bind(fd, (struct sockaddr*)&sun, sizeof(sun)) < 0 || listen(fd, 1) < 0){
		close(fd);
		return -1;
	}

	return fd;
}

void dump_packet(unsigned char* data, int len){
	int i;
	printf("0x");
	for (i=0; i<len; i++) printf("%02x", data[i]);
	printf("\n");
}

void write_mcu(unsigned char* data, int len){
	pthread_mutex_lock(&mcuwritelock);
	write (mcu_fd, data, len);
	printf("Writing packet:    ");
	dump_packet(data, len);
	pthread_mutex_unlock(&mcuwritelock);
}

void key_press(unsigned char keycode){
	char cmd[512];
	sprintf(cmd, "am broadcast -a tk.rabidbeaver.mcureceiver.MCU_KEY -ei %d", keycode);
	printf("KEY PRESSED: %02x\n",keycode);
	system(cmd);
	//TODO something better than this...
}

void process_mcu_key(unsigned char* data, int len){
	//TODO
	printf("UNIMPLEMENTED panel key: ");
	dump_packet(data, len);
}

void process_mcu_swi(unsigned char* data, int len){
	printf("swi key: ");
	dump_packet(data, len);
	switch(data[1]){
		case 0x00:
			switch(data[2]){
				case 0x07: // 0x010007
					key_press(0x12);
					return;
				case 0x10: // 0x010010
					if (data[3] == 0x07){
						key_press(0x0d);
						return;
					}
					if (data[3] == 0x20){
						key_press(0x0e);
						return;
					}
			}
			break;
		case 0x07: // 0x0107
			key_press(0xc);
			return;
		case 0x10: // onHandleSteer
			switch(data[2]){
				case 0xff:
					swi_detect = data[3];
					return;
				case 0x00:
					swi_adc = data[3];
					return;
				case 0x70:
					key_press(data[3]);
					return;
			}
			if (0x01 <= data[2] && data[2] <= 0x0d) KEY_ADC[data[2]-1] = data[3];
			if (0x61 <= data[2] && data[2] <= 0x6b) KEY_ADC[data[2]-0x51] = data[3];
			if (0x51 <= data[2] && data[2] <= 0x56) SCAN_ADC[data[2]-0x51] = data[3];
			return;
	}
	printf("UNIMPLEMENTED key: ");
	dump_packet(data, len);
}

void process_mcu_radio(unsigned char* data, int len){
	//TODO
	printf("UNIMPLEMENTED process_radio_mcu: ");
	dump_packet(data, len);
}

void set_usb_mode(int mode){
	int fd = open("/sys/kernel/debug/intel_otg/mode", O_RDWR);
	if (fd != -1){
		write(fd, (mode==USB_MODE_HOST)?"1":"0", 1);
		close(fd);
	}
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

void *set_bt_off_delayed(void * args){
	sleep(10);
	if (acc_on == 0){
		if (system("dumpsys bluetooth_manager | busybox grep \"^  state:\" | busybox grep ON") == 0){
			resume_bt_on_wake = 1;
			system("service call bluetooth_manager 8"); // turn bluetooth OFF
		} else
			resume_bt_on_wake = 0;
	}
	return 0;
}

void set_acc_on(int on){
	int fd;
	pthread_t bt_off_thread;
	if (acc_on != on){
		acc_on = on;
		//I'm pretty sure that cmd_29_acc_state_to_bsp just writes "0" or "1" to /sys/fytver/acc_on
		fd = open("/sys/fytver/acc_on", O_RDWR);
		if (fd != -1){
			write(fd, on?"1":"0", 1);
			close(fd);
		}

		if (on == 1){
			if (resume_bt_on_wake == 1) system("service call bluetooth_manager 6"); // turn bluetooth ON
			system("am broadcast -a tk.rabidbeaver.maincontroller.ACC_ON");
		} else {
			// Typically, you get into the car, turn the key to run position, which turns on ACC, then
			// turn to the START position, which turns off ACC, then release when engine starts, turning
			// back on ACC. To avoid the ON-OFF-ON, we launch a thread with a 10 second delay to shut down
			// the bluetooth. If the ACC is turned OFF after the 10 second delay, THEN we turn off BT.
			if (pthread_create(&bt_off_thread, NULL, set_bt_off_delayed, NULL) == 0) pthread_detach(bt_off_thread);
			system("am broadcast -a tk.rabidbeaver.maincontroller.ACC_OFF");
		}
	}
	//TODO send to HAL when available
}

void set_headlights_on(int on){
	headlights_on = on;
	if (on == 1) system("am broadcast -a tk.rabidbeaver.maincontroller.HEADLIGHTS_ON");
	else system("am broadcast -a tk.rabidbeaver.maincontroller.HEADLIGHTS_OFF");
	//TODO send to HAL when available
}

void set_ebrake_on(int on){
	ebrake_on = on;
	if (on == 1) system("am broadcast -a tk.rabidbeaver.maincontroller.EBRAKE_ON");
	else system("am broadcast -a tk.rabidbeaver.maincontroller.EBRAKE_OFF");
	//TODO send to HAL when available
}

void set_reverse_on(int on){
	reverse_on = on;
	if (on == 1) system("am broadcast -a tk.rabidbeaver.maincontroller.REVERSE_ON");
	else system("am broadcast -a tk.rabidbeaver.maincontroller.REVERSE_OFF");
	//TODO send to HAL when available
}

void process_mcu_main(unsigned char* data, int len){
	switch(data[2]){
		case 0x88: // MCU On
			sleeptick = 0;
			system("settings put secure location_providers_allowed +gps");
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
						system("settings put secure location_providers_allowed -gps");
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
					break;
				case 0x55:
					sleep(1);
					write_mcu(sleep3, 8);
					system("input keyevent 26");
					break;
			}
			break;
		case 0xd3: // Radio power on/off
			process_mcu_radio(data, len);
			break;
		case 0x00: // MCU/ACC power state
			switch(data[3]){
				case 0x00: // MCU OFF
					set_mcu_on(0);
					break;
				case 0x01: // MCU ON
					set_mcu_on(1);
					do_heartbeat = 1;
					break;
				case 0x21: // Must request MCU data
					request_mcu_data();
					break;
				case 0x31: // ACC ON
					set_acc_on(1);
					break;
				case 0x32: // ACC OFF
					set_acc_on(0);
					break;
			}
			break;
		case 0x07:
			process_mcu_swi(data, len);
			break;
		case 0x0d: // Button
			process_mcu_swi(data, len);
			break;
		case 0x10: // Buttons
			process_mcu_swi(data, len);
			break;
		case 0x11: // More buttons
			process_mcu_swi(data, len);
			break;
		case 0x12: // Headlights OFF
			set_headlights_on(0);
			break;
		case 0x13: // Headlights ON
			set_headlights_on(1);
			break;
		case 0x21: // More buttons
			process_mcu_swi(data, len);
			break;
		case 0x23: // Reverse ON/OFF
			if (data[3] == 0x02) set_reverse_on(0);
			else if (data[3] == 0x03) set_reverse_on(1);
			break;
		case 0x24: // ebrake ON/OFF
			set_ebrake_on(data[3] & 1);
			break;
		case 0x60: // RDS ON
			process_mcu_radio(data, len);
			break;
	}
}

void reset_mcu_delayed(unsigned char delay){
	unsigned char payload[] = {0x88, 0x55, 0x00, 0x03, 0x06, 0x20, delay, (0x00 ^ 0x03 ^ 0x06 ^ 0x20 ^ delay)};
	write_mcu(payload, 8);
}

void process_mcu(unsigned char* data, int len){
	switch(data[0]){
		case 0xc0:
		case 0xc3:
		case 0xc4:
			process_mcu_key(data, len);
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
			if (data[1] == 0x20) reset_mcu_delayed(data[2]);
			break;
		case 0x0a:
			oxaflag = data[1];
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
		if (n < 2) continue;

		size = (((int)buf[2])<<8) + buf[3];
		n = read(mcu_fd, buf+4, size+1);
		if (n < size+1) continue;

		cs = 0;
		for (i=2; i<size+5; i++){
			cs ^= buf[i];
		}
		if (cs != buf[size+5]) continue;

		printf("Read valid packet: ");
		dump_packet(buf, size+5);
		process_mcu(&buf[4], size);
	}
	return 0;
}

void *audio_hal_read(void *args){
	int rc;
	char buf[100];
	while (run){
		if ((audio_hal_cl_fd = accept(audio_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((rc=read(audio_hal_cl_fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(audio_hal_cl_fd);
		}
	}
	return 0;
}

void *car_hal_read(void *args){
	int rc;
	char buf[100];
	while (run){
		if ((car_hal_cl_fd = accept(car_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((rc=read(car_hal_cl_fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(car_hal_cl_fd);
		}
	}
	return 0;
}

void *lights_hal_read(void *args){
	int rc;
	char buf[100];
	while (run){
		if ((lights_hal_cl_fd = accept(lights_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((rc=read(lights_hal_cl_fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(lights_hal_cl_fd);
		}
	}
	return 0;
}

void *power_hal_read(void *args){
	int rc;
	char buf[100];
	while (run){
		if ((power_hal_cl_fd = accept(power_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((rc=read(power_hal_cl_fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(power_hal_cl_fd);
		}
	}
	return 0;
}

void *radio_hal_read(void *args){
	int rc;
	char buf[100];
	while (run){
		if ((radio_hal_cl_fd = accept(radio_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((rc=read(radio_hal_cl_fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(radio_hal_cl_fd);
		}
	}
	return 0;
}

void *key_nohal_read(void * args){
	int rc;
	char buf[100];
	while (run){
		if ((key_nohal_cl_fd = accept(key_nohal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((rc=read(key_nohal_cl_fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);

			// For now, just echo the data back to the client
			write (key_nohal_cl_fd, buf, rc);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(key_nohal_cl_fd);
		}
	}
	return 0;
}

int main(int argc, char ** argv){
	char *mcuportname = "/dev/ttyS0";
	pthread_t mcu_reader;

	pthread_t key_nohal_reader;
	pthread_t audio_hal_reader;
	pthread_t car_hal_reader;
	pthread_t lights_hal_reader;
	pthread_t power_hal_reader;
	pthread_t radio_hal_reader;

	mcu_fd = open (mcuportname, O_RDWR | O_NOCTTY | O_SYNC);
	if (mcu_fd < 0){
		printf ("error %d opening %s: %s\n", errno, mcuportname, strerror (errno));
		return -1;
	}

	if (set_interface_attribs (mcu_fd, B38400, 0) < 0) return -1;	// set speed to 38,400 bps, 8n1 (no parity)

	if (pthread_create(&mcu_reader, NULL, read_mcu, NULL) != 0) return -1;
	pthread_detach(mcu_reader);

	key_nohal_fd = create_socket("/dev/car/keys");
	audio_hal_fd = create_socket("/dev/car/audio");
	car_hal_fd = create_socket("/dev/car/main");
	lights_hal_fd = create_socket("/dev/car/lights");
	power_hal_fd = create_socket("/dev/car/power");
	radio_hal_fd = create_socket("/dev/car/radio");

	if (pthread_create(&key_nohal_reader, NULL, key_nohal_read, NULL) != 0) return -1;
	pthread_detach(key_nohal_reader);

	if (pthread_create(&audio_hal_reader, NULL, audio_hal_read, NULL) != 0) return -1;
	pthread_detach(audio_hal_reader);

	if (pthread_create(&car_hal_reader, NULL, car_hal_read, NULL) != 0) return -1;
	pthread_detach(car_hal_reader);

	if (pthread_create(&lights_hal_reader, NULL, lights_hal_read, NULL) != 0) return -1;
	pthread_detach(lights_hal_reader);

	if (pthread_create(&power_hal_reader, NULL, power_hal_read, NULL) != 0) return -1;
	pthread_detach(power_hal_reader);

	if (pthread_create(&radio_hal_reader, NULL, radio_hal_read, NULL) != 0) return -1;
	pthread_detach(radio_hal_reader);

/* Interfacing requirements;
 *
 * Hardware: MCU (serial /dev/ttyS0), BD37033 (i2c /dev/i2c-4), AMP (serial /dev/ttyS1)
 * HALs: audio, car, lights, power, radio
 *
 * For HALs, will create unix domain sockets in /dev/car/, each named for their particular HAL,
 * except for the car HAL, which will be named "main". Example: /dev/car/main
 *
 * Other: There may be some controls that are not handled by a typical HAL, like programming the
 * SWI. For this we will need to create additional unix domain socket(s) and build custom
 * application to interface with it.
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

