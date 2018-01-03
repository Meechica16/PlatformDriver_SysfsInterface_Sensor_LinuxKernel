#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>
#include <poll.h>
#include <pthread.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>
#include <linux/errno.h>

#include "buffer.h"
#include "libioctl.h"

#define DEVICE_SENSOR "/dev/sensor"


pthread_t polling_thread;
pthread_mutex_t mutex;

int stop = 0;
long my_distance;
int errno;

bufp userfifo[FIFOSIZE];

ioctl_data_t paramconfig;


/* Thread to calculate distance from object using POLL method */
void* polling_function(void* arg)
{

	int fd_sensor[10], i;
	char name[20];
	int ret;

	for(i=0;i<2;i++){
		sprintf(name, "/dev/HCSR_%d", i);

		fd_sensor[i] = open(name, O_RDWR);
		if(fd_sensor[i] == -1)
		{
			 	printf("file either does not exit or is currently used by an another user\n");
		 		exit(-1);
		}
	}

	paramconfig.arg1 = 7;
	paramconfig.arg2 = 65;
	printf("Setting the PARAMS m=%d samples, delta=%dms periodic sampling delay  on Device 1\n", paramconfig.arg1, paramconfig.arg2);
	ret = ioctl(fd_sensor[0], SET_PARAMETERS, &paramconfig);
	if(ret == -1)
		printf("ioctl failed\n");

	sleep(1);
	paramconfig.arg1 = 3;
	paramconfig.arg2 = 50;
	printf("Setting the PARAMS m=%d samples, delta=%dms periodic sampling delay for Device 2\n", paramconfig.arg1, paramconfig.arg2);
	ret = ioctl(fd_sensor[1], SET_PARAMETERS, &paramconfig);
	if(ret == -1)
		printf("ioctl failed\n");
	sleep(1);
	
	close(fd_sensor[0]);
	close(fd_sensor[1]);

	return NULL;
}

int main()
{

	int ret1;

	if (pthread_mutex_init(&mutex, NULL) != 0) 
	{
	    printf("\n mutex init failed\n");
	    return 1;
	}

	ret1 = pthread_create(&polling_thread, NULL, &polling_function, NULL);
	if (ret1 != 0)
	      printf("\ncan't create polling thread\n");
	usleep (3000);

	pthread_join (polling_thread, NULL);
	pthread_mutex_destroy(&mutex);

	return 0;
}
