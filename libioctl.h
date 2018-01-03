#include <linux/ioctl.h>

typedef struct ioctl_data
{
	int arg1;
	int arg2;
}ioctl_data_t;

#define IOC_MAGIC 'k'
#define CONFIG_PINS _IOWR(IOC_MAGIC, 0, unsigned int)
#define SET_PARAMETERS _IOWR(IOC_MAGIC, 1, unsigned int)
