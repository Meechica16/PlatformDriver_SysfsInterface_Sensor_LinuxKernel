
typedef struct sensor_chip{
	char *name;
	struct platform_device plf_dev;
	int dev_minor;
	int dev_no;
	struct sensor_chip *next;
}schip;
