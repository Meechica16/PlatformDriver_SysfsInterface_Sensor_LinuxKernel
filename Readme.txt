/////////////////////////////////////////////////////////////
Aakanksha Budhiraja, ASU ID:- 1211210335
Kushal Anil Parmar, ASU ID:- 1207686255
/////////////////////////////////////////////////////////////
Part 2 -  Platform device driver and sysfs interface for HC-SR04
==================================================================================================================
It containes 9 files:
buffer.h
gpioconfig.h
hcsr_sensor.h
libioctl.h
sensorchip.h
kernelmain.c
platform_device.c
sensor_driver.c
sysfs_test.sh
---------------------

buffer.h 		-> Includes the structure of a FIFO buffer which stores 5 recent readings of the timestamp and distance measured.

gpioconfig.h 	-> Includes the pinmux mapping to configure the IO pins received as an input. The input is the IO pin number, a flag to specify whether it should be input/output, and a flag to specify whether it is called to request/free the gpio pins

hcsr_sensor.h 	-> Common data structure used to store HCSR-04 device data shared between the platform device and the platform driver. This data structure is allocated per device, once a platform driver matches with a platform device on the virtual platform bus.

libioctl.h 		-> Specifies the ioctl definitions used in the sensor driver.

sensorchip.h 	-> Includes a data structure to contain platform device information.

kernelmain.c	-> user program to excerise on 2 device files performing R/W operations with separate samples set and delta for periodic delay. User input is the pin numbers for two devices

sensor_driver.c -> Implements sensor function using periodic measurement with the help of hrtimer.
	It has 2 ioctl functions:
	CONFIG_PINS: to configure trigger and echo pins using the IO pin numbers.
	SET_PARAMETERS: to set the number of samples per measurement and the sampling period.

	Read 	:- Reads the device buffer containing most recent 5 measures.
	WRITE 	:- Triggers a new measurement if no ongoing measurement, clears existing buffer if non-zero integer is passed as int argument else appends new data to buffer if int argument is 0. Returns EINVAL if ongoing measurement.

	probe	:- This initiliazes per device memory once a platform device is matched by this sensor driver with the help of the P_id_table[] of the driver.
	This register a device under misc class and maps this same device under the HCSR class of devices. Creates sysfs interface exposing the device attributes.

	remove	:- This removes the device from HCSR class and deregisters the device from misc class and free all the allocated device memory used during allocation.

	myfunc	:- This is the hrtimer callback function which generates triggers every delta ms for upto m+2 times.

platform_device.c -> This initializes the platform device structure embedded in struct sensor_chip per device and calls platform_device_register to register the devie on the platform bus to be detected by the platform driver sitting on the same virtual bus.

sysfs_test.sh 	-> This script is meant to test the sysfs interface for 2 HCSR devices.
						It also tests the /dev interface by resetting device attributes through ioctl.

==================================================================================================
NOTE:
1) Reboot

2) Export your SDK's toolchain path to your current directory to avoid unrecognized command.
export PATH="/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux/:$PATH"

3) Connect IO pins before starting the program.
=================================================================================================

Method:-

1) Change the following KDIR path to your source kernel path and the SROOT path to your sysroots path in the Makefile
KDIR:=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/usr/src/kernel
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/

2) Run make command to compile both user app, the platform sensor_driver and the platform device kernel objects.
make

3) copy 4 files from current dir to board dir using the below command:
	i) 	sensor_driver.ko
	ii)	kernelmain
	iii)platform_device.ko
	iv)	sysfs_test.sh

sudo scp <filename> root@<inet_address>:/home/root

4) chmod 777 ./sysfs_test.sh

5) Run the script
./sysfs_test.sh

6) User is prompted to enter device attributes for each device
Enter accordingly