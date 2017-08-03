/***
	The UAV node looks for IrDA devices and if it finds one
	it listens for incoming connections
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <linux/types.h>
#include <linux/irda.h>

#ifndef AF_IRDA
#define AF_IRDA 23
#endif /* AF_IRDA */

/* Memory allocation for discovery */
#define DISC_MAX_DEVICES 10
#define DISC_BUF_LEN	sizeof(struct irda_device_list) + \
	sizeof(struct irda_device_info) * DISC_MAX_DEVICES

char buf[4096];

/*
 * Function echo_discover_devices (fd)
 *
 *    Try to discover some remote device(s) that we can connect to
 *
 */
int irscanf_discover_devices(int fd)
{
	struct irda_device_list *list;
	unsigned char		buf[DISC_BUF_LEN];
	socklen_t len;
	int i;

	/* Set the list to point to the correct place */
	list = (struct irda_device_list *) buf;
	len = DISC_BUF_LEN;

	if((getsockopt(fd, SOL_IRLMP, IRLMP_ENUMDEVICES, buf, &len)) ||
			(list->len <= 0)) {
		perror("getsockopt");
		printf("Didn't find any devices!\n");
		return(-1);
	}

	/* List all devices */
	for (i=0;i<list->len;i++) {
		printf("  [%d] name:  %s, daddr: 0x%08x\n",
				i + 1, list->dev[i].info, list->dev[i].daddr);

	}
	return list->dev[0].daddr;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	struct sockaddr_irda peer, self;
	int daddr = 0;
	int fd, conn_fd;
	FILE *stream;
	int hints;
	socklen_t addrlen;


	/* Create socket */
	fd = socket(AF_IRDA, SOCK_STREAM, 0);
	if (fd < 0) {
		perror("socket");
		exit(-1);
	}

	/* Set the filter used for performing discovery */
	hints = HINT_COMPUTER | HINT_PDA;
	if (setsockopt(fd, SOL_IRLMP, IRLMP_HINT_MASK_SET, &hints, 
				sizeof(hints))) 
	{
		perror("setsockopt-hints");
		exit(-1);
	}
	/* Init self */
	self.sir_family = AF_IRDA;
	strncpy(self.sir_name, "uav", 25);
	self.sir_lsap_sel = LSAP_ANY;

	if (bind(fd, (struct sockaddr*) &self, sizeof(struct sockaddr_irda))) {
		perror("bind");
		return -1;
	}

	if (listen(fd, 8)) {
		perror("listen");
		return -1;
	}

	while (ros::ok())
	{
		daddr = irscanf_discover_devices(fd);
		ros::spinOnce();

		addrlen = sizeof(struct sockaddr_irda);

		printf("Waiting for connection!\n");
		conn_fd = accept(fd, (struct sockaddr *) &peer, &addrlen);
		if (conn_fd < 0) {
			perror("accept");
			return -1;
		}
		stream = fdopen(conn_fd, "r");
		if(stream == NULL) {
			perror("fdopen");
			return -1;
		}
		printf("Connected!\n");

		do {
			if((fgets(buf, sizeof(buf), stream) == NULL) ||
					(buf[0] == 0x3))
				buf[0] = '\0';

			fwrite(buf, 1, strlen(buf), stdout);

		} while (buf[0] != '\0');
		fflush(stdout);
		fclose(stream);
		close(conn_fd);
		printf("Disconnected!\n");
		loop_rate.sleep();
	}

	return 0;
}
