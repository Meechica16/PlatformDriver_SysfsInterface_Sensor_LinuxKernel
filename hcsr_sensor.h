#ifndef __HCSR_SENSOR_H__
#define __HCSR_SENSOR_H__

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/slab.h>

#include "buffer.h"
#include "sensorchip.h"

#define FIFOSIZE		5

int NUM_HCSR_DEVICES;

struct hcsr_dev{
	struct miscdevice sensor_dev;
	schip chipdevice;
	char name[20];		/* Name of Device */
	/* Device Attributes */
	int IOpinTrig;
	int IOpinEcho;
	int m;				/* samples per measurement */
	unsigned long delta;/* sampling period */
	int enable;			/* enable(1)/disable(0) measurement */
	int distance;		/* latest distance measure */
	/* Attributes END */
	/* Trigger and Echo gpio pin numbers */
	int trig_pin;
	int echo_pin;

	int done;
	/* FIFO parameters */
	bufp fifo[FIFOSIZE];
	int head, tail;
	int bufsize;

	int runcount;
	struct hrtimer my_hrtimer;
	spinlock_t lock_kprobe;
	unsigned long flags;
	struct list_head listnode;
};

struct hcsr_platform_data{
	int devno;
};

struct hcsr_drv{
	struct sensor_chip *head;
}*hcsr_drvp;

#endif /* __HCSR_SENSOR_H__ */