#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/param.h>
#include <linux/semaphore.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>	
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/math64.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include "hcsr_sensor.h"
#include "libioctl.h"
#include "gpioconfig.h"

#define CLASS_NAME		"HCSR"
#define DRIVER_NAME		"hcsr_platform_driver"


static LIST_HEAD(device_list);
static int classinit = 1;

/* platform driver ID table used for matching with platform devices */
static const struct platform_device_id P_id_table[] = {
         { "HCSR_0", 0 },
         { "HCSR_1", 0 },
         { "HCSR_2", 0 },
         { "HCSR_3", 0 },
         { "HCSR_4", 0 },
         { "HCSR_5", 0 },
         { "HCSR_6", 0 },
         { "HCSR_7", 0 },
         { "HCSR_8", 0 },
         { "HCSR_9", 0 },
	 { },
};

/* command line paramter to set the number of HCSR-04 devices to instantiate */
int argNum;
module_param(argNum, int, S_IRUGO|S_IWUSR);

int ready;
/* variables used to calculate the distance and remove the outliers */
int min = 10000000, max = 0, sum;
int MAXRUNS;

static int local_count = 0;
static unsigned long period_ns;		/* specify the sampling period in ns */
unsigned long flags;
uint64_t start, finish;				/* stores the timestamp at rising and falling edges of the ECHO interrupt */
uint64_t stamp, sensor_cycles;
long cycles;

unsigned int irq_line;				/* stores the requested irqline */
ktime_t ktime;

static struct device *gko_device;
static struct class *gko_class;
static dev_t gko_dev;


/* time stamp counter to retrieve the CPU cycles */
uint64_t tsc(void)
{
	uint32_t a, d;
	asm volatile("rdtsc" : "=a" (a), "=d" (d));
	return (( (uint64_t)a)|( (uint64_t)d)<<32 );
}

/* interrupt handler to detect the rising and falling edge on the GPIO ECHO pin */
static irqreturn_t sensor_interrupt_handler (int irq, void *dev_id)
{
	struct hcsr_dev* hcsr_devp = (struct hcsr_dev *)dev_id;
	int avg_distance = 0;
	long my_distance;

		if (gpio_get_value(hcsr_devp->echo_pin))
		{	
			//Detect the Rising edge, note the timestamp value and set the IRQ Line to detect the FALLING edge
			start = tsc();
			irq_set_irq_type(irq, IRQF_TRIGGER_FALLING);		
		}
		else
		{
			//Detect the FALLING edge, note the timestamp value and set the IRQ Line to detect the RISING edge
			finish = tsc();
			sensor_cycles = finish - start;
			
			spin_lock_irqsave(&hcsr_devp->lock_kprobe,flags);
			cycles = (long)sensor_cycles;
			my_distance = (cycles*340)/80000;

			/* captures distance measurements upto m+1 samples */
			if(hcsr_devp->runcount < (MAXRUNS - 1) && hcsr_devp->runcount >= 0)
			{
				printk(KERN_INFO "distance = %ld\n", my_distance/100);
				if(cycles<min)
					min = cycles;
				if(cycles>max)
					max = cycles;
				sum += cycles;
				hcsr_devp->runcount++;
			}
			/* Capture the final m+2 sample and evaluate the average distance */
			if(hcsr_devp->runcount == (MAXRUNS -1))
			{
				printk(KERN_INFO "distance = %ld\n", my_distance/100);
				if(cycles<min)
					min = cycles;
				if(cycles>max)
					max = cycles;
				sum += cycles;

				sum = sum - max - min;
  				avg_distance = sum/(MAXRUNS - 2);
  				avg_distance = (avg_distance*340)/80000;

  				/* Store the average distance and corresponding timestamp in the FIFO buffer */
				hcsr_devp->fifo[hcsr_devp->head].distance = avg_distance/100;
				hcsr_devp->fifo[hcsr_devp->head].timestamp = tsc();
				hcsr_devp->distance = hcsr_devp->fifo[hcsr_devp->head].distance;

				printk("Storing Buffer at head:%d dist = %ld tsc = %lu\n", hcsr_devp->head, hcsr_devp->fifo[hcsr_devp->head].distance, (unsigned long)hcsr_devp->fifo[hcsr_devp->head].timestamp);

				hcsr_devp->head = (hcsr_devp->head + 1) % FIFOSIZE;
				hcsr_devp->bufsize = ((hcsr_devp->bufsize+1) > FIFOSIZE) ? FIFOSIZE:(hcsr_devp->bufsize+1);
				if(hcsr_devp->head == hcsr_devp->tail)
					hcsr_devp->tail = (hcsr_devp->tail + 1) % FIFOSIZE;
				
				hcsr_devp->runcount = -1;
				sum = 0;
				min = 100000000;
				max = 0;
				hcsr_devp->done =1;
			}
			spin_unlock_irqrestore(&hcsr_devp->lock_kprobe,flags);
			irq_set_irq_type(irq, IRQF_TRIGGER_RISING);
		}							
	return IRQ_HANDLED;
}

/* sensor device open function */
int sensor_open(struct inode *inode, struct file *file)
{

	const struct file_operations *fops = NULL;
	struct hcsr_dev *hcsr_devp;
	int minor = iminor(inode);

	/* find the private device struct using minor number */
	list_for_each_entry(hcsr_devp, &device_list, listnode)
	{
		if(hcsr_devp->sensor_dev.minor == minor)
		{
			fops = fops_get(hcsr_devp->sensor_dev.fops);
			break;
		}
	}
	
	/* Easy access to devp from rest of the entry points */
	file->private_data = hcsr_devp;
	printk(KERN_INFO "Opening device\n");

	return 0;
}

/* sensor device release function */
int sensor_release(struct inode *inode, struct file *file)
{

	printk(KERN_INFO "\nDevice is closing\n");

	return 0;
}

/* timer callback function to generate the trigger pulse every delta ms. 
* delta should be < 90ms in order to generate accurate measurements and collect
* exact number of samples
*/
static enum hrtimer_restart myfunc(struct hrtimer *timer)
{
	int ret_overrun;
	ktime_t currtime , interval;
	struct hcsr_dev *hcsr_devp;
	/* Get the per-device structure that contains this cdev */
	hcsr_devp = container_of(timer, struct hcsr_dev, my_hrtimer);

	currtime  = ktime_get();

	//Generate TRIGGER PULSE
	gpio_set_value_cansleep(hcsr_devp->trig_pin, 0);
	udelay(100);
	gpio_set_value_cansleep(hcsr_devp->trig_pin, 1);
	udelay(116);
	gpio_set_value_cansleep(hcsr_devp->trig_pin, 0);
 
	if (local_count < (MAXRUNS)) {

		interval = ktime_set(0,period_ns); 
	  	ret_overrun = hrtimer_forward(timer, currtime , interval);
		local_count++;
		
		return HRTIMER_RESTART;
  	}

  else{
		local_count = 0;
		return HRTIMER_NORESTART;
  	}
}

/* sensor device read function which is used to copy the distance measurement to the user space */
ssize_t sensor_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct hcsr_dev *hcsr_devp = file->private_data;
	int ret = 0;

	if(hcsr_devp->bufsize==0)
	{	/* If buffer is empty and timer is active, wait for timer to stop and then copy buffer to user */
		if(hrtimer_active(&hcsr_devp->my_hrtimer))
		{			
			//wait and copy to user
			while(!hcsr_devp->bufsize)
			{
				// printk(KERN_INFO "Timer active, waiting for buffer data\n");

			}
			ret = copy_to_user(buf, &hcsr_devp->fifo, sizeof(bufp)*FIFOSIZE);
			// printk(KERN_INFO "Copied buffer. Buff size = %d\n", hcsr_devp->bufsize);
		}
		else/* Timer not active, hence we trigger a new measurement */
		{
			//trigger new measurement
			hcsr_devp->done = 0;
		  	local_count = 0;
			hcsr_devp->runcount = 0;
			MAXRUNS = hcsr_devp->m + 2;
			period_ns = 1000000*hcsr_devp->delta;
			ktime = ktime_set( 0, period_ns );
			hrtimer_start((&hcsr_devp->my_hrtimer), ktime, HRTIMER_MODE_REL);

			printk(KERN_INFO "Triggered new measurement\n");

			//wait and copy to user
			while(!hcsr_devp->bufsize && hrtimer_active(&hcsr_devp->my_hrtimer))
			{
				// printk(KERN_INFO "Timer active, waiting for buffer data2\n");
				
			}
			
			ret = copy_to_user(buf, &hcsr_devp->fifo, sizeof(bufp)*FIFOSIZE);
			// printk(KERN_INFO "Copied buffer to user: Buff size = %d\n", hcsr_devp->bufsize);
		}
	}
	else //if buffsize!=0
	{
		//After we have detected both the RISING and FALLING edges
		while(hrtimer_active(&hcsr_devp->my_hrtimer) || (hcsr_devp->done == 0) )
		{
			// printk(KERN_INFO "Timer 2 active, waiting for buffer data\n");

		}
		ret = copy_to_user(buf, &hcsr_devp->fifo, sizeof(bufp)*FIFOSIZE);
		// printk(KERN_INFO "Copied buffer. Buff size = %d\n", hcsr_devp->bufsize);
	}
	return 0;	
}

/* sensor device write function which is used to generate the TRIGGER using the GPIO pin */
ssize_t sensor_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct hcsr_dev *hcsr_devp = file->private_data;
	int clearbuf;
	int ret;

	printk(KERN_INFO "Write for device: %s\n", hcsr_devp->name);

	ret = copy_from_user(&clearbuf, (int *)buf, sizeof(int));

	/* if a non-zero integer is received from user input then clear the buffer */
	if(clearbuf)
		printk(KERN_INFO "Clearing Buffer:%d\n", clearbuf);

	if(clearbuf)
	{
		memset(hcsr_devp->fifo, 0, FIFOSIZE*sizeof(bufp));
		hcsr_devp->head = 0;
		hcsr_devp->tail = 0;
		hcsr_devp->bufsize = 0;
		printk(KERN_INFO "\nBuffer Cleared\n");
	}

	/* If no on-going measurement then trigger a new measurement */
	if(!hrtimer_active((&hcsr_devp->my_hrtimer)))
	{
		hcsr_devp->done = 0;
		local_count = 0;
		hcsr_devp->runcount = 0;

		MAXRUNS = hcsr_devp->m + 2;
		period_ns = 1000000*hcsr_devp->delta;
		ktime = ktime_set( 0, period_ns );
		printk(KERN_INFO "Write started measurement\n");
		hrtimer_start((&hcsr_devp->my_hrtimer), ktime, HRTIMER_MODE_REL);

	return 0;
	}
	/* return with error if there is an on-going measurement */
	else{
		printk(KERN_INFO "********Write Failed\n");
		return -EINVAL;
	}
}

/* IOCTL commands to set the device attributes */
long sensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg )
{

	struct hcsr_dev *hcsr_devp = file->private_data;
	ioctl_data_t ioctl_data;
	int ret, res;

	ret = copy_from_user((void *)&ioctl_data, (void * __user)arg, sizeof(ioctl_data_t));

	switch(cmd)
	{
		case CONFIG_PINS:
		{

			if((ioctl_data.arg1 >= 0 && ioctl_data.arg1 <=19) && 
				(ioctl_data.arg2>=0 &&	(ioctl_data.arg2 <=3 || ioctl_data.arg2 == 10 || ioctl_data.arg2 == 12)))
			{
				/* configuring the trigger pin */
				hcsr_devp->trig_pin = gpio_config_pin(ioctl_data.arg1, 0, 0);
				hcsr_devp->IOpinTrig = ioctl_data.arg1;
				/* configuring the echo pin */
				hcsr_devp->echo_pin = gpio_config_pin(ioctl_data.arg2, 1, 0);
				hcsr_devp->IOpinEcho = ioctl_data.arg2;
				printk("PIN SET TRIGGER=%d  ECHO=%d\n", hcsr_devp->trig_pin, hcsr_devp->echo_pin);
			}
			else
				return -EINVAL;

			//connect the gpio pin for ECHO to the irq line to detect rising and falling edge interrupts
			if( (irq_line = gpio_to_irq(hcsr_devp->echo_pin)) < 0 )
			{
				printk(KERN_INFO "Failed to get IRQ no for the gpio pin\n");
			}
			//Request the IRQ line 
			res = request_irq(irq_line, sensor_interrupt_handler, IRQF_TRIGGER_RISING, "gpio_change_state", (void *)hcsr_devp);
			if (res < 0)
			{
				printk(KERN_INFO "Failed to request IRQ line\n");
				if (res == -EBUSY)
					ret = res;
				else
					ret = -EINVAL;
				return ret;
			}
		}
		break;
		case SET_PARAMETERS:
		{
			/* Need atleast 3 samples to compute average distance and,
			a positive sampling interval to take periodic measurements */
			if(ioctl_data.arg1<=2 || ioctl_data.arg2<=0)
				return -EINVAL;

			hcsr_devp->m = ioctl_data.arg1;
			hcsr_devp->delta = ioctl_data.arg2;
			// MAXRUNS = hcsr_devp->m + 2;
			
			printk(KERN_INFO "PARAM SET to m = %d, delta = %lu\n", hcsr_devp->m, hcsr_devp->delta);
		}
		break;	
		default:
			printk("Invalid Command\n");
	}
	return 0;
}

/* device file operations data structure */
static struct file_operations my_dev_fops = {
	.owner = THIS_MODULE, 			/* Owner */
	.open = sensor_open, 			/* Open method */
	.release = sensor_release,		/* Release method */
	.unlocked_ioctl = sensor_ioctl,	/* IOCTL method */
	.write = sensor_write, 			/* Write method */
	.read = sensor_read, 			/* Read method */
	
};

/* Functions to display the device attribute values using the sysfs interface:-
* trigger, echo, number_samples, sampling_period, distance and enable */
static ssize_t hcsr_trigger_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->IOpinTrig);
}

static ssize_t hcsr_echo_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->IOpinEcho);
}

static ssize_t hcsr_number_samples_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->m);
}

static ssize_t hcsr_sampling_period_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%lu\n", hcsr_devp->delta);
}

static ssize_t hcsr_enable_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->enable);
}

static ssize_t hcsr_distance_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->distance);
}

/* Functions to store the device attribute values using the sysfs interface:-
* trigger, echo, number_samples, sampling_period and enable */
static ssize_t hcsr_trigger_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int IOpin_trigger;

	sscanf(buf, "%d", &IOpin_trigger);
	if(IOpin_trigger>=0 && IOpin_trigger<=19)
	{
		hcsr_devp->IOpinTrig = IOpin_trigger;
		hcsr_devp->trig_pin = gpio_config_pin(IOpin_trigger, 0, 0);
	}
	else
		return -EINVAL;

	return count;
}

static ssize_t hcsr_echo_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int IOpin_echo, ret, res;

	sscanf(buf, "%d", &IOpin_echo);
	if( (IOpin_echo>=0 && IOpin_echo<=3) || IOpin_echo==10 || IOpin_echo==12)
	{
		hcsr_devp->IOpinEcho = IOpin_echo;
		hcsr_devp->echo_pin = gpio_config_pin(IOpin_echo, 1, 0);
	}
	else
		return -EINVAL;

	//connect the gpio pin for ECHO to the irq line to detect rising and falling edge interrupts
	if( (irq_line = gpio_to_irq(hcsr_devp->echo_pin)) < 0 )
	{
		printk(KERN_INFO "Failed to get IRQ no for the gpio pin\n");
	}
	//Request the IRQ line 
	res = request_irq(irq_line, sensor_interrupt_handler, IRQF_TRIGGER_RISING, "gpio_change_state", (void *)hcsr_devp);
	if (res < 0)
	{
		printk(KERN_INFO "Failed to request IRQ line\n");
		if (res == -EBUSY)
			ret = res;
		else
			ret = -EINVAL;
		return ret;
	}

	return count;
}

static ssize_t hcsr_number_samples_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int samples;

	sscanf(buf, "%d", &samples);
	hcsr_devp->m = samples;

	return count;
}

static ssize_t hcsr_sampling_period_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	unsigned long sampling_period;

	sscanf(buf, "%lu", &sampling_period);
	hcsr_devp->delta = sampling_period;

	return count;
}

static ssize_t hcsr_enable_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf,
								size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int enable;
	int ret;

	sscanf(buf, "%d", &enable);
	hcsr_devp->enable = enable;

	if( (!hrtimer_active((&hcsr_devp->my_hrtimer))) && (hcsr_devp->enable != 0) )
	{
	    // MAXRUNS = hcsr_devp->m + 2;
		hcsr_devp->done = 0;
		local_count = 0;
		hcsr_devp->runcount = 0;

		MAXRUNS = hcsr_devp->m + 2;
		period_ns = 1000000*hcsr_devp->delta;
		ktime = ktime_set( 0, period_ns );
		printk(KERN_INFO "Write started measurement\n");
		hrtimer_start((&hcsr_devp->my_hrtimer), ktime, HRTIMER_MODE_REL);

	return count;
	}

	if(hcsr_devp->enable == 0)
	{
		ret = hrtimer_cancel(&hcsr_devp->my_hrtimer);
		if(ret==0)
			printk("Timer was not active\n");
		else if(ret==1)
			printk("Timer was active and is stopped now\n");

	return count;
	}
	return count;
}
/* End of attribute functions */

/* Declare the device attributes with its functions and permissions */
static DEVICE_ATTR(trigger, S_IRWXU, hcsr_trigger_show, hcsr_trigger_store);
static DEVICE_ATTR(echo, S_IRWXU, hcsr_echo_show, hcsr_echo_store);
static DEVICE_ATTR(number_samples, S_IRWXU, hcsr_number_samples_show, hcsr_number_samples_store);
static DEVICE_ATTR(sampling_period, S_IRWXU, hcsr_sampling_period_show, hcsr_sampling_period_store);
static DEVICE_ATTR(enable, S_IRWXU, hcsr_enable_show, hcsr_enable_store);
static DEVICE_ATTR(distance, S_IRUSR, hcsr_distance_show, NULL);


/* sensor device initialisation function to initialise the gpios and request an irq line for the rising edge */
static int P_driver_probe(struct platform_device *dev_found)
{
	const char *name;

	struct sensor_chip *schip_dev;
	struct hcsr_dev *hcsr_devp;

	schip_dev = container_of(dev_found, struct sensor_chip, plf_dev);
	hcsr_devp = container_of(schip_dev, struct hcsr_dev, chipdevice);
	name = kmalloc(20, GFP_KERNEL);

	printk(KERN_ALERT "Found the device -- NAME:%s devNO: %d\n", schip_dev->name, schip_dev->dev_no);


	if(classinit==1)
	{
		/* class */
	    gko_class = class_create(THIS_MODULE, CLASS_NAME);
	    if (IS_ERR(gko_class)) {
	            printk(KERN_ERR " cant create class %s\n", CLASS_NAME);
	            // goto class_err;
	    }
	}
	classinit = 0;

	spin_lock_init(&hcsr_devp->lock_kprobe);
	hcsr_devp->chipdevice = *schip_dev;

	/* Init the misc device parameters */
	hcsr_devp->sensor_dev.minor = MISC_DYNAMIC_MINOR;
	name = hcsr_devp->chipdevice.plf_dev.name;
	hcsr_devp->sensor_dev.name = name;
	hcsr_devp->sensor_dev.fops = &my_dev_fops;
	misc_register(&hcsr_devp->sensor_dev);
	//printk("Init device %s\n", hcsr_devp->sensor_dev.name);

	/* Initialize the buffer and the fifo buffer operating parameters */
	memset(hcsr_devp->fifo, 0, FIFOSIZE*sizeof(bufp));
	hcsr_devp->head = 0;
	hcsr_devp->tail = 0;
	hcsr_devp->bufsize = 0;
	hcsr_devp->done = 0;
	hcsr_devp->runcount = 0;
	hcsr_devp->IOpinTrig = 0;
	hcsr_devp->IOpinEcho = 0;

	/* Init the hrtimer of this device and its hrtimer callback function */
	hrtimer_init(&(hcsr_devp->my_hrtimer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
 	hcsr_devp->my_hrtimer.function = &myfunc;

 	/* add this new device to a list of devices*/
 	INIT_LIST_HEAD(&hcsr_devp->listnode);
 	list_add(&hcsr_devp->listnode, &device_list);

 	/* Creating HCSR sysfs interface */
 	gko_dev = 1;

 	/* create a HCSR device under the HCSR class with the allocated 
 	device memory from misc device */
    gko_device = device_create(gko_class, NULL, gko_dev, hcsr_devp, name);
    if (IS_ERR(gko_device)) {
            printk(KERN_ERR " cant create device %s\n", name);
            // goto device_err;
    }

    device_create_file(gko_device, &dev_attr_trigger);
    device_create_file(gko_device, &dev_attr_echo);
    device_create_file(gko_device, &dev_attr_number_samples);
    device_create_file(gko_device, &dev_attr_sampling_period);
    device_create_file(gko_device, &dev_attr_enable);
    device_create_file(gko_device, &dev_attr_distance);

	hcsr_devp->chipdevice.dev_minor = hcsr_devp->sensor_dev.minor;
	printk(KERN_ALERT "Registered platform device: %s with the driver\n", hcsr_devp->sensor_dev.name);

	return 0;

}

static int P_driver_remove(struct platform_device *pdev)
{
	struct hcsr_dev *hcsr_devp;
	struct hcsr_dev *hcsr_devc;		/* used as the cursor to list for loop */
	struct sensor_chip *schip_dev;

	schip_dev = container_of(pdev, struct sensor_chip, plf_dev);
	hcsr_devp = container_of(schip_dev, struct hcsr_dev, chipdevice);

	printk("Removing Device: %s\n", hcsr_devp->chipdevice.plf_dev.name);

	list_for_each_entry(hcsr_devc, &device_list, listnode);
	{
		//printk("HCSR NAME :%s  DEV:%d  SCHIPdev:%d\n", hcsr_devp->chipdevice.plf_dev.name, hcsr_devp->chipdevice.dev_no, schip_dev->dev_no);
		if(hcsr_devp->chipdevice.dev_no == schip_dev->dev_no)
		{
			list_del(&hcsr_devp->listnode);
			
			gpio_config_pin(hcsr_devp->IOpinTrig, -1, 1);
			gpio_config_pin(hcsr_devp->IOpinEcho, -1, 1);
			free_irq(gpio_to_irq(hcsr_devp->echo_pin), (void *)hcsr_devp);

			device_destroy(gko_class, hcsr_devp->sensor_dev.minor);
			misc_deregister(&hcsr_devp->sensor_dev);
			printk("Deregistered misc device --> %s\n", schip_dev->plf_dev.name);
			kfree(hcsr_devp);
		}
	}

	if(list_empty(&device_list))
	{
		class_unregister(gko_class);
        class_destroy(gko_class);
        printk("Destroyed HCSR class\n");
	}

	printk("Device removed\n");
	return 0;
}


static struct platform_driver P_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= P_driver_probe,
	.remove		= P_driver_remove,
	.id_table	= P_id_table,
};


module_platform_driver(P_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CSE_530-TEAM_27");
MODULE_DESCRIPTION("CSE_530-Assignment2Part2");