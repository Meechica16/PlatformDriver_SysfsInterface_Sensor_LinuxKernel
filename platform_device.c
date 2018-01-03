#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "hcsr_sensor.h"

int NUM_HCSR_DEVICES;
int argNum;
struct hcsr_dev **hcsr_devpArr;

module_param(argNum, int, S_IRUGO|S_IWUSR);


static void device_release(struct device *dev)
 {
	 
 }

/**
 * register the device when module is initiated
 */

static int p_device_init(void)
{
	int ret = 0;
	int i;
	char *name = NULL, chipname[20];
	

	/* number of HCSR sensor devices to be initialized received as an argument from command line */
	NUM_HCSR_DEVICES = (int)argNum;

	hcsr_devpArr = kmalloc(sizeof(struct hcsr_dev *)*NUM_HCSR_DEVICES, GFP_KERNEL);

	hcsr_drvp = kmalloc(sizeof(struct hcsr_drv), GFP_KERNEL);
	if(!hcsr_drvp) {
		printk(KERN_INFO "Bad kmalloc for Driver container\n");
		return -ENOMEM;
	}

	for(i=0;i<NUM_HCSR_DEVICES;i++)
	{
		name = kmalloc(20, GFP_KERNEL);
		hcsr_devpArr[i] = kmalloc(sizeof(struct hcsr_dev), GFP_KERNEL);
		if(!hcsr_devpArr[i]) {
			printk(KERN_INFO "Bad kmalloc for Device memory\n");
			return -ENOMEM;
		}
		memset(hcsr_devpArr[i], 0, sizeof(struct hcsr_dev));

		sprintf(name, "HCSR_%d", i);
		sprintf(chipname, "HCSR%d", i);
	    hcsr_devpArr[i]->chipdevice.name = chipname;
	    // strncpy(hcsr_devpArr[i]->chipdevice.plf_dev.name, name, 20);
	    hcsr_devpArr[i]->chipdevice.plf_dev.name = name;
	    hcsr_devpArr[i]->chipdevice.plf_dev.id = -1;
	    hcsr_devpArr[i]->chipdevice.dev_minor = -1;
	    hcsr_devpArr[i]->chipdevice.dev_no = i+10;
	    hcsr_devpArr[i]->chipdevice.plf_dev.dev.release = device_release;
	    
		/* Register the device */
		platform_device_register(&hcsr_devpArr[i]->chipdevice.plf_dev);
	}
	i--;
	hcsr_devpArr[i]->chipdevice.next = NULL;

	while(i)
	{
		hcsr_devpArr[i-1]->chipdevice.next = &hcsr_devpArr[i]->chipdevice;
		i=i-1;
	}
	hcsr_drvp->head = &hcsr_devpArr[i]->chipdevice;

	return ret;
}

static void p_device_exit(void)
{
	int i;

	for(i=NUM_HCSR_DEVICES-1; i>=0; i--)
	{
		hcsr_devpArr[i]->chipdevice.next = NULL;
		platform_device_unregister(&hcsr_devpArr[i]->chipdevice.plf_dev);
	}
	

	hcsr_drvp->head = NULL;
	kfree(hcsr_drvp);

	printk(KERN_ALERT "Goodbye, unregister the device\n");
}

module_init(p_device_init);
module_exit(p_device_exit);
MODULE_LICENSE("GPL");