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
#include <android/log.h>
#include <linux/i2c-dev.h>

#define USB_MODE_DEVICE "none"
#define USB_MODE_HOST "host"

unsigned char heartbeat[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x55, 0xfd};
unsigned char sleep1[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x5f, 0xf7};
unsigned char sleep2[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x61, 0xc9};
unsigned char sleep3[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0xaa, 0x62, 0xca};

unsigned char oxaflag = 0x00;
unsigned char KEY_ADC[50] = {0x0};
unsigned char SCAN_ADC[6] = {0x0};
unsigned char swi_detect = 0x0;
unsigned char swi_adc = 0x0;

unsigned char radio_power_on = 0x1;
unsigned char radio_rds_on = 0x0;
unsigned char* rds_text_packet = 0;
int rds_text_len = 0;
int radio_freq = 0;
unsigned char radio_band = 0x0;
unsigned char radio_loc = 0x0;
unsigned char radio_pty = 0x0;
unsigned char radio_rds_stat = 0x0;
unsigned char radio_area = 0x0;

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

static pthread_mutex_t radiohalwritelock;
static pthread_mutex_t keynohalwritelock;
static pthread_mutex_t carhalwritelock;
static pthread_mutex_t powerhalwritelock;

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

void dump_packet(char* label, unsigned char* data, int len){
	int i;
	char buffer[1024] = {0};
	sprintf(buffer, "%s: 0x", label);
	for (i=0; i<len; i++) sprintf(buffer+strlen(buffer), "%02x", data[i]);
	__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "%s", buffer);
}

void generate_cs(unsigned char* data, int len){
	int i;
	unsigned char cs = 0;

	for (i = 2; i < len-1; i++){
		cs ^= data[i];
	}

	data[len-1] = cs;
}

void write_mcu(unsigned char* data, int len){
	pthread_mutex_lock(&mcuwritelock);
	write (mcu_fd, data, len);
	dump_packet("WriteMCU", data, len);
	pthread_mutex_unlock(&mcuwritelock);
}

void write_bd(unsigned char* data, int len){
	pthread_mutex_lock(&bdwritelock);
	write (bd_fd, data, len);
	dump_packet("WriteBD", data, len);
	pthread_mutex_unlock(&bdwritelock);
}

void write_car_hal(unsigned char* data, int len){
	if (car_hal_cl_fd > 0){
		pthread_mutex_lock(&carhalwritelock);
		write (car_hal_cl_fd, data, len);
		pthread_mutex_unlock(&carhalwritelock);
	}
}

void write_power_hal(unsigned char* data, int len){
	if (power_hal_cl_fd > 0){
		pthread_mutex_lock(&powerhalwritelock);
		write (power_hal_cl_fd, data, len);
		pthread_mutex_unlock(&powerhalwritelock);
	}
}

void write_swi_nohal(unsigned char* data, int len){
	if (key_nohal_cl_fd > 0){
		pthread_mutex_lock(&keynohalwritelock);
		write (key_nohal_cl_fd, data, len);
		pthread_mutex_unlock(&keynohalwritelock);
	}
}

void write_radio_hal(unsigned char* data, int len){
	if (radio_hal_cl_fd > 0){
		pthread_mutex_lock(&radiohalwritelock);
		write (radio_hal_cl_fd, data, len);
		pthread_mutex_unlock(&radiohalwritelock);
	}
}

void key_press(unsigned char keycode){
	unsigned char key[] = {0xaa, 0x55, 0x02, 0x02, keycode, 0x02 ^ 0x02 ^ keycode};
	write_swi_nohal(key, 6);
}

void process_mcu_key(unsigned char* data, int len){
	//TODO
	dump_packet("UNIMPLEMENTED panel key", data, len);
}

void send_swi_nohal(unsigned char id){
	int i;
	unsigned char adc_packet[] = {0xaa, 0x55, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	switch(id){
		case 0x01: // adc scan
			for (i = 4; i < 10; i++) adc_packet[i] = SCAN_ADC[i-4];
			generate_cs(adc_packet, 11);
			write_swi_nohal(adc_packet, 11);
			return;
	}
}

void process_mcu_swi(unsigned char* data, int len){
	dump_packet("SWI key", data, len);
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
			if (0x51 <= data[2] && data[2] <= 0x56){
				SCAN_ADC[data[2]-0x51] = data[3];
				if (data[2] == 0x56) send_swi_nohal(0x01);
			}
			return;
	}
	dump_packet("UNIMPLEMENTED key", data, len);
}

void send_radio_hal(unsigned char id){
	unsigned char twobuffer[] = {0xaa, 0x55, 0x00, 0x02, 0x00, 0x00, 0x00};
	unsigned char fourbuffer[] = {0xaa, 0x55, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};

	switch(id){
		case 0x0e: // get band
			twobuffer[4] = 0x01;
			twobuffer[5] = radio_band;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
		case 0x0f: // get freq
			fourbuffer[4] = 0x02;
			fourbuffer[7] = radio_freq % 0x100;
			fourbuffer[6] = radio_freq >> 8 % 0x100;
			fourbuffer[5] = radio_freq >> 16 % 0x100;
			generate_cs(fourbuffer, 9);
			write_radio_hal(fourbuffer, 9);
			return;
		case 0x10: // get area
			twobuffer[4] = 0x03;
			twobuffer[5] = radio_area;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
		case 0x11: // get rds on
			twobuffer[4] = 0x0a;
			twobuffer[5] = radio_rds_on;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
		case 0x12: // get rds text
			write_radio_hal(rds_text_packet, rds_text_len);
			return;
		case 0x13: // get power
			twobuffer[4] = 0x09;
			twobuffer[5] = radio_power_on;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
		case 0x14: // get loc
			twobuffer[4] = 0x06;
			twobuffer[5] = radio_loc;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
		case 0x15: // get pty_id
			twobuffer[4] = 0x05;
			twobuffer[5] = radio_pty;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
		case 0x16: // get rds_stat
			twobuffer[4] = 0x04;
			twobuffer[5] = radio_rds_stat;
			generate_cs(twobuffer, 7);
			write_radio_hal(twobuffer, 7);
			return;
	}
}

void process_mcu_radio(unsigned char* data, int len){
	int i;

	switch(data[0]){
		case 0x21: // radio config store and send here, also send when requested.
			return;
		case 0x50: // rds channel text
			// This is somehow associated with stored "channels". Not sure what to do with that....
			dump_packet("RDS CHANNEL TEXT", data, len);
			return;
		case 0x51: // rds text
			dump_packet("RDS TEXT", data, len);
			if (rds_text_packet > 0) free(rds_text_packet);
			rds_text_packet = malloc(len);
			rds_text_packet[0] = 0xaa;
			rds_text_packet[1] = 0x55;
			for (i = 2; i < len; i++) rds_text_packet[i] = data[i];
			rds_text_packet[4] = 0x07;
			generate_cs(rds_text_packet, len);
			rds_text_len = len;
			send_radio_hal(0x12);
			return;
		case 0x01:
			switch(data[1]){
				case 0x00:
					switch(data[2]){
						case 0x60: // rds on/off
							if ((data[3] & 0x01) == 0x01) radio_power_on = 0x01;
							else radio_rds_on = 0x00;
							send_radio_hal(0x11);
							return;
						case 0xbb: // power on/off
							if ((data[3] & 0x01) == 0x01) radio_rds_on = 0x01;
							else radio_power_on = 0x00;
							send_radio_hal(0x13);
							return;
					}
					break;
				case 0xd3: // ps text
					dump_packet("RDS PS TEXT", data, len);
					return;
				case 0x03: // the big one
					switch(data[2]){
						case 0x80:
						case 0x82:
						case 0x00:
							break;
						case 0x01:
							radio_freq = data[3] * 10000;
							return;
						case 0x02:
							radio_freq += data[3] * 100;
							return;
						case 0x03:
							radio_freq += data[3];
							if (radio_freq > 100000){
								radio_freq -= 100000;
								return;
							}
							send_radio_hal(0x0f);
							return;
						case 0x05:
						case 0x06:
							break;
						case 0x07: // loc
							radio_loc = data[3] & 0x01;
							send_radio_hal(0x14);
							return;
						case 0x08:
						case 0x10:
						case 0x18:
							break;
						case 0x21: // pty id
							radio_pty = data[3];
							send_radio_hal(0x15);
							return;
						case 0x22: // rds stat
							radio_rds_stat = data[3];
							send_radio_hal(0x16);
							return;
						case 0x30: // area
							radio_area = data[3] - 0x01;
							send_radio_hal(0x10);
							return;
					}
					break;
			}
			break;
	}

	dump_packet("process_radio_mcu unhandled or invalid", data, len);
}

void set_usb_mode(char* mode){
	int fd = open("/sys/kernel/debug/intel_otg/mode", O_RDWR);
	if (fd != -1){
		write(fd, mode, strlen(mode));
		close(fd);
	}
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
		__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "Changing MCU Power: %d", on);
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
	unsigned char buffer[] = {0xaa, 0x55, 0x02, 0x01, 0x00, 0x00};
	int fd;
	pthread_t bt_off_thread;
	if (acc_on != on){
		acc_on = on;
		__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "Changing ACC: %d", on);

		fd = open("/sys/fytver/acc_on", O_RDWR);
		if (fd != -1){
			write(fd, on?"1":"0", 1);
			close(fd);
		}

		if (on == 1){
			if (resume_bt_on_wake == 1) system("service call bluetooth_manager 6"); // turn bluetooth ON
			system("am broadcast -a tk.rabidbeaver.maincontroller.ACC_ON");
			buffer[4] = 0x01;
		} else {
			// Typically, you get into the car, turn the key to run position, which turns on ACC, then
			// turn to the START position, which turns off ACC, then release when engine starts, turning
			// back on ACC. To avoid the ON-OFF-ON, we launch a thread with a 10 second delay to shut down
			// the bluetooth. If the ACC is turned OFF after the 10 second delay, THEN we turn off BT.
			if (pthread_create(&bt_off_thread, NULL, set_bt_off_delayed, NULL) == 0) pthread_detach(bt_off_thread);
			system("am broadcast -a tk.rabidbeaver.maincontroller.ACC_OFF");
			buffer[4] = 0x00;
		}
	}
	generate_cs(buffer, 6);
	write_car_hal(buffer, 6);
	write_power_hal(buffer, 6);
}

void set_headlights_on(int on){
	unsigned char buffer[] = {0xaa, 0x55, 0x02, 0x02, 0x00, 0x00};
	headlights_on = on;
	if (on == 1){
		system("am broadcast -a tk.rabidbeaver.maincontroller.HEADLIGHTS_ON");
		buffer[4] = 0x01;
	} else {
		system("am broadcast -a tk.rabidbeaver.maincontroller.HEADLIGHTS_OFF");
		buffer[4] = 0x00;
	}
	generate_cs(buffer, 6);
	write_car_hal(buffer, 6);
}

void set_ebrake_on(int on){
	unsigned char buffer[] = {0xaa, 0x55, 0x02, 0x03, 0x00, 0x00};
	ebrake_on = on;
	if (on == 1){
		system("am broadcast -a tk.rabidbeaver.maincontroller.EBRAKE_ON");
		buffer[4] = 0x01;
	} else {
		system("am broadcast -a tk.rabidbeaver.maincontroller.EBRAKE_OFF");
		buffer[4] = 0x00;
	}
	generate_cs(buffer, 6);
	write_car_hal(buffer, 6);
}

void set_reverse_on(int on){
	unsigned char buffer[] = {0xaa, 0x55, 0x02, 0x04, 0x00, 0x00};
	reverse_on = on;
	if (on == 1){
		system("am broadcast -a tk.rabidbeaver.maincontroller.REVERSE_ON");
		buffer[4] = 0x01;
	} else {
		system("am broadcast -a tk.rabidbeaver.maincontroller.REVERSE_OFF");
		buffer[4] = 0x00;
	}
	generate_cs(buffer, 6);
	write_car_hal(buffer, 6);
}

void process_mcu_main(unsigned char* data, int len){
	switch(data[2]){
		case 0x88: // MCU early switch. This kicks both on shutdown AND on startup.
			sleeptick = 0;
			break;
		case 0x89: // MCU shutdown sequence
			system("setprop sys.fyt.sleeping 1");
			system("setprop sys.sleep 1");
			system("setprop sys.sleeptimes 1");
			switch(data[3]){
				case 0x53:
					do_heartbeat = 0;
					sleeptick++;
					if (sleeptick == 1){
						set_usb_mode(USB_MODE_DEVICE);
						system("settings put secure location_providers_allowed -gps");
						// returns 0 when wifi is enabled, 256 when disabled.
						__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "Checking wifi state");
						if (system("dumpsys wifi | busybox grep \"^Wi-Fi is enabled\"") == 0){
							// wifi is enabled. Store this and disable.
							__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "Shutting wifi off");
							resume_wifi_on_wake = 1;
							system("svc wifi disable");
						} else
							resume_wifi_on_wake = 0;
					}
					if (sleeptick > 4) write_mcu(sleep1, 8);
					return;
				case 0x54:
					sleeptick = 0;
					write_mcu(sleep2, 8);
					return;
				case 0x55:
					sleep(1);
					write_mcu(sleep3, 8);
					system("input keyevent 26");
					return;
			}
			break;
		case 0xbb: // Radio power on/off
			process_mcu_radio(data, len);
			break;
		case 0x00: // MCU/ACC power state
			switch(data[3]){
				case 0x00: // MCU OFF
					set_mcu_on(0);
					return;
				case 0x01: // MCU ON
					set_mcu_on(1);
					do_heartbeat = 1;
					system("settings put secure location_providers_allowed +gps");
					system("setprop sys.fyt.sleeping 0");
					system("setprop sys.sleep 0");
					if (resume_wifi_on_wake == 1){
						system("svc wifi enable");
						resume_wifi_on_wake = 0;
					}
					return;
				case 0x21: // Must request MCU data
					request_mcu_data();
					return;
				case 0x31: // ACC ON
					set_acc_on(1);
					return;
				case 0x32: // ACC OFF
					set_acc_on(0);
					return;
			}
			break;
		case 0x07:
			process_mcu_swi(data, len);
			return;
		case 0x0d: // Button
			process_mcu_swi(data, len);
			return;
		case 0x10: // Buttons
			process_mcu_swi(data, len);
			return;
		case 0x11: // More buttons
			process_mcu_swi(data, len);
			return;
		case 0x12: // Headlights OFF
			set_headlights_on(0);
			return;
		case 0x13: // Headlights ON
			set_headlights_on(1);
			return;
		case 0x21: // More buttons
			process_mcu_swi(data, len);
			return;
		case 0x23: // Reverse ON/OFF
			if (data[3] == 0x02) set_reverse_on(0);
			else if (data[3] == 0x03) set_reverse_on(1);
			return;
		case 0x24: // ebrake ON/OFF
			set_ebrake_on(data[3] & 1);
			return;
		case 0x60: // RDS ON
			process_mcu_radio(data, len);
			return;
	}
	dump_packet("PROCESS MCU MAIN Unhandled Packet", data, len);
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
			return;
		case 0x01:
			switch(data[1]){
				case 0xd3:
				case 0x03:
					process_mcu_radio(data, len);
					return;
				case 0x00:
					process_mcu_main(data, len);
					return;
				case 0x07:
				case 0x10:
				case 0x38:
					process_mcu_swi(data, len);
					return;
			}
			break;
		case 0x06:
			if (data[1] == 0x20) reset_mcu_delayed(data[2]);
			return;
		case 0x0a:
			oxaflag = data[1];
			return;
		case 0x21:
		case 0x50:
		case 0x51:
			process_mcu_radio(data, len);
			return;

	}
	dump_packet("READ Unhandled Packet", data, len);
}

void *read_mcu(void * args){
	unsigned char buf[1024];
	int n;
	int size;
	int i;
	unsigned char cs;
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
		for (i=2; i<size+4; i++){
			cs ^= buf[i];
		}
		if (cs != buf[size+4]) continue;

		dump_packet("Read valid packet", buf, size+5);
		process_mcu(&buf[4], size);
	}
	return 0;
}

void process_audio_command(unsigned char* data, int length){
	int i;
	for (i=3; i<length-2; i+=2) write_bd(data+i, 2);
}

void *audio_hal_read(void *args){
	int rc, cs, i;
	unsigned char buf[100];
	while (run){
		if ((audio_hal_cl_fd = accept(audio_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		// forever read the client until the client disconnects.
		while (run){
			rc = read(audio_hal_cl_fd, buf, 1); // read for first shield byte 0xaa
			if (rc <= 0) break;
			if (buf[0] != 0xaa) continue;
			rc = read(audio_hal_cl_fd, buf+1, 1); // read for second shield byte 0x55
			if (rc <= 0) break;
			if (buf[1] != 0x55) continue;
			rc = read(audio_hal_cl_fd, buf+2, 1); // read length byte
			if (rc <= 0) break;
			rc = read(audio_hal_cl_fd, buf+3, buf[2]+1); // read the data + xor
			if (rc <= 0 || rc < buf[2]+1) break; // rc will only be < buf[2]+1 if we run into EOF
			cs = 0;
			for (i = 2; i < buf[2]+3; i++) cs ^= buf[i];
			if (cs == buf[buf[2]+3]){
				dump_packet("audio_hal_read VALID", buf, buf[2]+4);
				process_audio_command(buf, buf[2]+4);
			} else {
				dump_packet("audio_hal_read INVALID", buf, buf[2]+4);
			}
		}

		if (rc == -1) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "AUDIOHAL READ ERROR");
			close(audio_hal_cl_fd);
		} else if (rc == 0) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "AUDIOHAL EOF");
			close(audio_hal_cl_fd);
		}
		audio_hal_cl_fd = 0;
	}
	return 0;
}

void process_car_command(unsigned char* data, int length){
	// Control AMP and ANTENNA power signals?
}

void *car_hal_read(void *args){
	int rc, cs, i;
	unsigned char buf[100];
	while (run){
		if ((car_hal_cl_fd = accept(car_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		// forever read the client until the client disconnects.
		while (run){
			rc = read(car_hal_cl_fd, buf, 1); // read for first shield byte 0xaa
			if (rc <= 0) break;
			if (buf[0] != 0xaa) continue;
			rc = read(car_hal_cl_fd, buf+1, 1); // read for second shield byte 0x55
			if (rc <= 0) break;
			if (buf[1] != 0x55) continue;
			rc = read(car_hal_cl_fd, buf+2, 1); // read length byte
			if (rc <= 0) break;
			rc = read(car_hal_cl_fd, buf+3, buf[2]+1); // read the data + xor
			if (rc <= 0 || rc < buf[2]+1) break; // rc will only be < buf[2]+1 if we run into EOF
			cs = 0;
			for (i = 2; i < buf[2]+3; i++) cs ^= buf[i];
			if (cs == buf[buf[2]+3]){
				dump_packet("car_hal_read VALID", buf, buf[2]+4);
				process_car_command(buf, buf[2]+4);
			} else {
				dump_packet("car_hal_read INVALID", buf, buf[2]+4);
			}
		}

		if (rc == -1) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "CARHAL READ ERROR");
			close(car_hal_cl_fd);
		} else if (rc == 0) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "CARHAL EOF");
			close(car_hal_cl_fd);
		}
		radio_hal_cl_fd = 0;
	}
	return 0;
}

void process_lights_command(unsigned char* data, int length){
	// Likely this will mostly be for dealing with the panel button lights
}

void *lights_hal_read(void *args){
	int rc, cs, i;
	unsigned char buf[100];
	while (run){
		if ((lights_hal_cl_fd = accept(lights_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		// forever read the client until the client disconnects.
		while (run){
			rc = read(lights_hal_cl_fd, buf, 1); // read for first shield byte 0xaa
			if (rc <= 0) break;
			if (buf[0] != 0xaa) continue;
			rc = read(lights_hal_cl_fd, buf+1, 1); // read for second shield byte 0x55
			if (rc <= 0) break;
			if (buf[1] != 0x55) continue;
			rc = read(lights_hal_cl_fd, buf+2, 1); // read length byte
			if (rc <= 0) break;
			rc = read(lights_hal_cl_fd, buf+3, buf[2]+1); // read the data + xor
			if (rc <= 0 || rc < buf[2]+1) break; // rc will only be < buf[2]+1 if we run into EOF
			cs = 0;
			for (i = 2; i < buf[2]+3; i++) cs ^= buf[i];
			if (cs == buf[buf[2]+3]){
				dump_packet("lights_hal_read VALID", buf, buf[2]+4);
				process_lights_command(buf, buf[2]+4);
			} else {
				dump_packet("lights_hal_read INVALID", buf, buf[2]+4);
			}
		}

		if (rc == -1) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "LIGHTSHAL READ ERROR");
			close(lights_hal_cl_fd);
		} else if (rc == 0) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "LIGHTSHAL EOF");
			close(lights_hal_cl_fd);
		}
		lights_hal_cl_fd = 0;
	}
	return 0;
}

void process_power_command(unsigned char* data, int length){
	// We can use this to receive commands like TURN SCREEN OFF.
}

void *power_hal_read(void *args){
	int rc, cs, i;
	unsigned char buf[100];
	while (run){
		if ((power_hal_cl_fd = accept(power_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		// forever read the client until the client disconnects.
		while (run){
			rc = read(power_hal_cl_fd, buf, 1); // read for first shield byte 0xaa
			if (rc <= 0) break;
			if (buf[0] != 0xaa) continue;
			rc = read(power_hal_cl_fd, buf+1, 1); // read for second shield byte 0x55
			if (rc <= 0) break;
			if (buf[1] != 0x55) continue;
			rc = read(power_hal_cl_fd, buf+2, 1); // read length byte
			if (rc <= 0) break;
			rc = read(power_hal_cl_fd, buf+3, buf[2]+1); // read the data + xor
			if (rc <= 0 || rc < buf[2]+1) break; // rc will only be < buf[2]+1 if we run into EOF
			cs = 0;
			for (i = 2; i < buf[2]+3; i++) cs ^= buf[i];
			if (cs == buf[buf[2]+3]){
				dump_packet("power_hal_read VALID", buf, buf[2]+4);
				process_power_command(buf, buf[2]+4);
			} else {
				dump_packet("power_hal_read INVALID", buf, buf[2]+4);
			}
		}

		if (rc == -1) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "POWERHAL READ ERROR");
			close(power_hal_cl_fd);
		} else if (rc == 0) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "POWERHAL EOF");
			close(power_hal_cl_fd);
		}
		power_hal_cl_fd = 0;
	}
	return 0;
}

void process_radio_command(unsigned char* data, int length){
	unsigned char band_AM[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x1d, 0x1c};
	unsigned char band_FM[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x1a, 0x1b};
	unsigned char step_up[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x11, 0x10};
	unsigned char step_down[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x10, 0x11};
	unsigned char seek_up[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x06, 0x07};
	unsigned char seek_down[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x05, 0x04};
	unsigned char power_on[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0xbb, 0xb9};
	unsigned char power_off[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0xbc, 0xbe};
	unsigned char rds_on[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0x60, 0x62};
	unsigned char rds_off[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0x61, 0x63};
	unsigned char autosens_on[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0x9f, 0x9d};
	unsigned char autosens_off[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0x9e, 0x9f};
	unsigned char stereo[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x0b, 0x0a};
	unsigned char loc[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x0d, 0x0c};
	unsigned char rdsptyen[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x15, 0x14};
	unsigned char rdstaen[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x16, 0x17};
	unsigned char rdsafen[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x17, 0x22};

	unsigned char freq_direct[] = {0x88, 0x55, 0x00, 0x03, 0x25, 0x00, 0x00, 0x00};
	unsigned char area[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x03, 0x00, 0x00};

	switch(data[3]){
		case 0x01: // band
			if (length != 6) break;
			if (data[4] == 0x01) write_mcu(band_AM, 8);
			else write_mcu(band_FM, 8);
			radio_band = data[4];
			return;
		case 0x02: // tune direct
			if (length != 7) break;
			if (data[4] > 0x2a) data[4] = 0x2a;
			if (data[4] == 0x2a && data[5] > 0x30) data[5] = 0x30;
			freq_direct[5] = data[4];
			freq_direct[6] = data[5];
			generate_cs(freq_direct, 8);
			write_mcu(freq_direct, 8);
			return;
		case 0x03: // tune step
			if (length != 6) break;
			if (data[4] == 0x01) write_mcu(step_up, 8);
			else write_mcu(step_down, 8);
			return;
		case 0x04: // seek
			if (length != 6) break;
			if (data[4] == 0x01) write_mcu(seek_up, 8);
			else write_mcu(seek_down, 8);
			return;
		case 0x05: // power
			if (length != 6) break;
			if (data[4] == 0x01) write_mcu(power_on, 8);
			else write_mcu(power_off, 8);
			return;
		case 0x06: // rds
			if (length != 6) break;
			if (data[4] == 0x01) write_mcu(rds_on, 8);
			else write_mcu(rds_off, 8);
			return;
		case 0x07: // area
			if (length != 6 || data[4] > 4) break;
			area[6] = data[4];
			generate_cs(area, 8);
			write_mcu(area, 8);
			return;
		case 0x08: // auto sensitivity
			if (length != 6) break;
			if (data[4] == 0x01) write_mcu(autosens_on, 8);
			else write_mcu(autosens_off, 8);
			return;
		case 0x09: // stereo
			if (length != 5) break;
			write_mcu(stereo, 8);
			return;
		case 0x0a: // loc
			if (length != 5) break;
			write_mcu(loc, 8);
			return;
		case 0x0b: // rds pty enable
			if (length != 5) break;
			write_mcu(rdsptyen, 8);
			return;
		case 0x0c: // rds ta enable
			if (length != 5) break;
			write_mcu(rdstaen, 8);
			return;
		case 0x0d: // rds af enable
			if (length != 5) break;
			write_mcu(rdsafen, 8);
			return;
		case 0x0e: // get band
		case 0x0f: // get freq
		case 0x10: // get area
		case 0x11: // get rds on
		case 0x12: // get rds text
		case 0x13: // get power
		case 0x14: // get loc
		case 0x15: // get pty_id
		case 0x16: // get rds_stat
			send_radio_hal(data[3]);
			return;
		
	}
	dump_packet("process_radio_command UNKNOWN or INVALID Packet", data, length);
}

void *radio_hal_read(void *args){
	int rc, cs, i;
	unsigned char buf[100];
	while (run){
		if ((radio_hal_cl_fd = accept(radio_hal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		// forever read the client until the client disconnects.
		while (run){
			rc = read(radio_hal_cl_fd, buf, 1); // read for first shield byte 0xaa
			if (rc <= 0) break;
			if (buf[0] != 0xaa) continue;
			rc = read(radio_hal_cl_fd, buf+1, 1); // read for second shield byte 0x55
			if (rc <= 0) break;
			if (buf[1] != 0x55) continue;
			rc = read(radio_hal_cl_fd, buf+2, 1); // read length byte
			if (rc <= 0) break;
			rc = read(radio_hal_cl_fd, buf+3, buf[2]+1); // read the data + xor
			if (rc <= 0 || rc < buf[2]+1) break; // rc will only be < buf[2]+1 if we run into EOF
			cs = 0;
			for (i = 2; i < buf[2]+3; i++) cs ^= buf[i];
			if (cs == buf[buf[2]+3]){
				dump_packet("radio_hal_read VALID", buf, buf[2]+4);
				process_radio_command(buf, buf[2]+4);
			} else {
				dump_packet("radio_hal_read INVALID", buf, buf[2]+4);
			}
		}

		if (rc == -1) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "RADIOHAL READ ERROR");
			close(radio_hal_cl_fd);
		} else if (rc == 0) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "RADIOHAL EOF");
			close(radio_hal_cl_fd);
		}
		radio_hal_cl_fd = 0;
	}
	return 0;
}

void process_key_command(unsigned char* data, int length){
	int i;
	unsigned char stop_detect[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x10, 0x00, 0x12};
	unsigned char start_detect[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x10, 0x01, 0x13,
					0x88, 0x55, 0x00, 0x03, 0x01, 0x10, 0x02, 0x10};
	unsigned char clear[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x10, 0x20, 0x28};
	unsigned char save[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x10, 0x21, 0x29};

	unsigned char genkey[] = {0x88, 0x55, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00};

	unsigned char key_map[] = {0x00, 0x12, 0x13, 0x14, 0x15, 0x00, 0x00, 0x18, 0x19, 0x1a,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x81, 0x82, 0x83,
				0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a};

	char key_cmd[32] = {0};

	switch(data[3]){
		case 0x01: // enter detection mode, value = [0|1]
			if (length != 6) break;
			swi_detect = data[4];
			if (data[4] == 0x0) write_mcu(stop_detect, 8);
			else write_mcu(start_detect, 16);
			return;
		case 0x02: // clear
			if (length != 5) break;
			write_mcu(clear, 8);
			return;
		case 0x03: // save
			if (length != 5) break;
			write_mcu(save, 8);
			return;
		case 0x04: // keyAdc
			if (length != 6 || data[4] < 0x0 || data[4] >= 0x32) break;
			i = 0;
			while (i < 6 && SCAN_ADC[i] > 0xf0) i++;
			if (i > 6) break;
			genkey[5] = key_map[data[4]];
			genkey[6] = swi_adc;
			generate_cs(genkey, 8);
			write_mcu(genkey, 8);
			return;
		case 0x05: // input keyevent keycode
			if (length != 6) break;
			sprintf(key_cmd, "input keyevent %d", data[4]);
			system(key_cmd);
			return;
	}
	dump_packet("process_key_command UNKNOWN or INVALID Packet", data, length);
}

void *key_nohal_read(void * args){
	int rc, i, cs;
	unsigned char buf[100];
	// forever listen for connections
	while (run){
		if ((key_nohal_cl_fd = accept(key_nohal_fd, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		// forever read the client until the client disconnects.
		while (run){
			rc = read(key_nohal_cl_fd, buf, 1); // read for first shield byte 0xaa
			if (rc <= 0) break;
			if (buf[0] != 0xaa) continue;
			rc = read(key_nohal_cl_fd, buf+1, 1); // read for second shield byte 0x55
			if (rc <= 0) break;
			if (buf[1] != 0x55) continue;
			rc = read(key_nohal_cl_fd, buf+2, 1); // read length byte
			if (rc <= 0) break;
			rc = read(key_nohal_cl_fd, buf+3, buf[2]+1); // read the data + xor
			if (rc <= 0 || rc < buf[2]+1) break; // rc will only be < buf[2]+1 if we run into EOF
			cs = 0;
			for (i = 2; i < buf[2]+3; i++) cs ^= buf[i];
			if (cs == buf[buf[2]+3]){
				dump_packet("key_nohal_read VALID", buf, buf[2]+4);
				process_key_command(buf, buf[2]+4);
			} else {
				dump_packet("key_nohal_read INVALID", buf, buf[2]+4);
			}
		}
		if (rc == -1) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "KEYNOHAL READ ERROR");
			close(key_nohal_cl_fd);
		} else if (rc == 0) {
			__android_log_print(ANDROID_LOG_DEBUG, "MCUD", "KEYNOHAL EOF");
			close(key_nohal_cl_fd);
		}
		key_nohal_cl_fd = 0;
	}
	return 0;
}

void reset_bd37(){
	char buf[] = {0xfe, 0x81};
	if (write(bd_fd, buf, 2) != 2){
		__android_log_print(ANDROID_LOG_ERROR, "MCUD", "i2c write failure");
	}
}

int main(int argc, char ** argv){
	char *mcuportname = "/dev/ttyS0";
	char *bdportname = "/dev/i2c-4";
	int bdaddr = 0x40;
	pthread_t mcu_reader;

	pthread_t key_nohal_reader;
	pthread_t audio_hal_reader;
	pthread_t car_hal_reader;
	pthread_t lights_hal_reader;
	pthread_t power_hal_reader;
	pthread_t radio_hal_reader;

	daemon(0,0);

	system("setprop audio.hw.allow_close 1");

	mcu_fd = open (mcuportname, O_RDWR | O_NOCTTY | O_SYNC);
	if (mcu_fd < 0){
		__android_log_print(ANDROID_LOG_ERROR, "MCUD", "error %d opening %s: %s\n", errno, mcuportname, strerror (errno));
		return -1;
	}

	if (set_interface_attribs (mcu_fd, B38400, 0) < 0) return -1;	// set speed to 38,400 bps, 8n1 (no parity)

	if (pthread_create(&mcu_reader, NULL, read_mcu, NULL) != 0) return -1;
	pthread_detach(mcu_reader);

	bd_fd = open (bdportname, O_RDWR);
	if (bd_fd < 0){
		__android_log_print(ANDROID_LOG_ERROR, "MCUD", "error %d opening %s: %s\n", errno, bdportname, strerror (errno));
		return -1;
	}
	if (ioctl(bd_fd, I2C_SLAVE, bdaddr) < 0) {
		return -1;
	}

	reset_bd37();

	system ("/system/bin/mkdir /dev/car; /system/bin/chmod 755 /dev/car");

	key_nohal_fd = create_socket("/dev/car/keys");
	audio_hal_fd = create_socket("/dev/car/audio");
	car_hal_fd = create_socket("/dev/car/main");
	lights_hal_fd = create_socket("/dev/car/lights");
	power_hal_fd = create_socket("/dev/car/power");
	radio_hal_fd = create_socket("/dev/car/radio");

	system ("/system/bin/chmod 777 /dev/car/*");

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

