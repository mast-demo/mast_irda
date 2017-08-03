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
	struct sockaddr_irda peer;
	int daddr = 0;
	int fd;
	FILE *stream;
	int hints;

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

	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		//		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
		/* Find a peer */
		daddr = irscanf_discover_devices(fd);
		if (daddr == -1)
		{
			ROS_WARN("No devices found");
		}
	}

	return 0;
}
