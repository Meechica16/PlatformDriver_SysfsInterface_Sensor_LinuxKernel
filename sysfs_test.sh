#!/bin/bash

echo "Loading platform Driver and 2 platform devices"
insmod sensor_driver.ko
insmod platform_device.ko argNum=2

echo "Enter HCSR_0 attributes"
echo "********Enter first set of values*********"
echo "Enter IO trigger pin"
read trig
echo -n $trig > /sys/class/HCSR/HCSR_0/trigger

echo "Enter IO echo pin"
read echop
echo -n $echop > /sys/class/HCSR/HCSR_0/echo

echo "Enter number of samples to measure (samples >= 3)"
read sample
echo -n $sample > /sys/class/HCSR/HCSR_0/number_samples

echo "Enter sampling period (period < 90 ms for accurate measurements)"
read period
echo -n $period > /sys/class/HCSR/HCSR_0/sampling_period

echo "Enter HCSR_1 attributes"
echo "********Enter first set of values*********"
echo "Enter IO trigger pin"
read trig
echo -n $trig > /sys/class/HCSR/HCSR_1/trigger

echo "Enter IO echo pin"
read echop
echo -n $echop > /sys/class/HCSR/HCSR_1/echo

echo "Enter number of samples to measure (samples >= 3)"
read sample
echo -n $sample > /sys/class/HCSR/HCSR_1/number_samples

echo "Enter sampling period (period < 90 ms for accurate measurements)"
read period
echo -n $period > /sys/class/HCSR/HCSR_1/sampling_period
#///////////////////////////////////////////////////////////////////

echo "Enter non-zero integer to set enable on HCSR_0"
read int
echo "Enter non-zero integer to set enable on HCSR_1"
read int2
echo "Starting measurement on both devices"
echo -n $int > /sys/class/HCSR/HCSR_0/enable
sleep 1
echo -n $int2 > /sys/class/HCSR/HCSR_1/enable
sleep 1
echo "Distance measured on HCSR_0 = "
cat /sys/class/HCSR/HCSR_0/distance

echo "Distance measured on HCSR_1 = "
cat /sys/class/HCSR/HCSR_1/distance
#///////////////////////////////////////////////////////////////

#Second measurement for HCSR_0
echo "********Enter second set of values for HCSR_0*********"
echo "Enter number of samples to measure (samples >= 3)"
read sample
echo -n $sample > /sys/class/HCSR/HCSR_0/number_samples

echo "Enter sampling period (period < 90 ms for accurate measurements)"
read period
echo -n $period > /sys/class/HCSR/HCSR_0/sampling_period

echo "********Enter second set of values for HCSR_1*********"
echo "Enter number of samples to measure (samples >= 3)"
read sample
echo -n $sample > /sys/class/HCSR/HCSR_1/number_samples

echo "Enter sampling period (period < 90 ms for accurate measurements)"
read period
echo -n $period > /sys/class/HCSR/HCSR_1/sampling_period
#set enable
echo "Enter non-zero integer to set enable on HCSR_0"
read int
echo "Enter non-zero integer to set enable on HCSR_1"
read int2
echo "Starting measurement on both devices"
echo -n $int > /sys/class/HCSR/HCSR_0/enable
sleep 1
echo -n $int2 > /sys/class/HCSR/HCSR_1/enable
sleep 1
echo "Distance measured on HCSR_0 = "
cat /sys/class/HCSR/HCSR_0/distance

echo "Distance measured on HCSR_1 = "
cat /sys/class/HCSR/HCSR_1/distance

#testing /dev interface to reset the sampling attributes
./kernelmain
echo "Parameters changed by /dev interface"
echo "HCSR_0 samples and period"
cat /sys/class/HCSR/HCSR_0/number_samples
cat /sys/class/HCSR/HCSR_0/sampling_period
echo "HCSR_1 samples and period"
cat /sys/class/HCSR/HCSR_1/number_samples
cat /sys/class/HCSR/HCSR_1/sampling_period

#Third measurement to test disabling of ongoing measurement and retrieves most recent distance
echo "Enter non-zero integer to set enable on HCSR_0"
read int
echo "Enter non-zero integer to set enable on HCSR_1"
read int2

echo "Starting measurement on both devices"
echo -n $int > /sys/class/HCSR/HCSR_0/enable
echo "Distance measured on HCSR_0 = "
cat /sys/class/HCSR/HCSR_0/distance
echo "Disabling measurement"
echo -n 0 > /sys/class/HCSR/HCSR_0/enable
echo -n $int2 > /sys/class/HCSR/HCSR_1/enable
sleep 1
echo "Distance measured on HCSR_1 = "
cat /sys/class/HCSR/HCSR_1/distance

echo "Unloading Driver and the Devices"
rmmod platform_device.ko

