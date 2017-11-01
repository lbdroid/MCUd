#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

char *socket_path = "/dev/car/keys";
int fd;

void *socket_read(void * args){
	int srv_fd, rc;
	char buf[100];
	while (1){
		while ((rc=read(fd, buf, sizeof(buf))) > 0) {
			printf("read %u bytes: %.*s\n", rc, rc, buf);
		}
		if (rc == -1) {
			perror("read");
			exit(-1);
		} else if (rc == 0) {
			printf("EOF\n");
			close(srv_fd);
		}
	}
	return 0;
}

int main(int argc, char *argv[]) {
	struct sockaddr_un addr;
	pthread_t socket_reader;
	char buf[100];
	int rc;

	if (argc > 1) socket_path=argv[1];

	if ( (fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
		perror("socket error");
		exit(-1);
	}

	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	if (*socket_path == '\0') {
		*addr.sun_path = '\0';
		strncpy(addr.sun_path+1, socket_path+1, sizeof(addr.sun_path)-2);
	} else {
		strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);
	}

	if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
		perror("connect error");
		exit(-1);
	}

	if (pthread_create(&socket_reader, NULL, socket_read, NULL) != 0) return -1;
	pthread_detach(socket_reader);

	while( (rc=read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
		if (write(fd, buf, rc) != rc) {
			if (rc > 0) perror("partial write");
			else {
				perror("write error");
				exit(-1);
			}
		}
	}

	return 0;
}

