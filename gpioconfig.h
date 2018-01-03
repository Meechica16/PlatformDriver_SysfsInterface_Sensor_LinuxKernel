#include <linux/gpio.h>


void gpio_set_config(int gpioFunc, int gpioShift, int pinmux1, 
					int level1, int pinmux2, int level2, int dir, int free)
{
	if(!free)
	{
		if(pinmux1 != -1)
		{
			gpio_request(pinmux1, "muxSelect1");
			gpio_set_value_cansleep(pinmux1, level1);
		}
		if(pinmux2 != -1)
		{
			gpio_request(pinmux2, "muxSelect2");
			gpio_set_value_cansleep(pinmux2, level2);
		}

		if(gpioShift != -1)
		{
			gpio_request(gpioShift, "shiftReg");
			if(dir)
				gpio_direction_input(gpioShift);
			else
				gpio_direction_output(gpioShift, dir);

		}

		gpio_request(gpioFunc, "IOpin");
		if(dir)
			gpio_direction_input(gpioFunc);
		else{
			gpio_direction_output(gpioFunc, dir);
			gpio_set_value_cansleep(gpioFunc, 0);
		}
	}
	else
	{
		if(gpioShift != -1)
			gpio_free(gpioShift);
		if(pinmux1 != -1)
			gpio_free(pinmux1);
		if(pinmux2 != -1)
			gpio_free(pinmux2);

		gpio_free(gpioFunc);

	}
	
}

int gpio_config_pin(int IOpin, int inout, int free)
{
	int pin;
	switch(IOpin) {

		case 0:
			gpio_set_config(11, 32, -1, -1, -1, -1, inout, free);
			pin = 11;
			break;

		case 1:
			gpio_set_config(12, 28, 45, 0, -1, -1, inout, free);
			pin = 12;
			break;

		case 2:
			gpio_set_config(13, 34, 77, 0, -1, -1, inout, free);
			pin = 13;
			break;

		case 3:
			gpio_set_config(14, 16, 76, 0, 64, 0, inout, free);
			pin = 14;
			break;

		case 4:
			gpio_set_config(6, 36, -1, -1, -1, -1, inout, free);
			pin = 6;
			break;

		case 5:
			gpio_set_config(0, 18, 66, 0, -1, -1, inout, free);
			pin = 0;
			break;

		case 6:
			gpio_set_config(1, 20, 68, 0, -1, -1, inout, free);
			pin = 1;
			break;

		case 7:
			gpio_set_config(38, -1, -1, -1, -1, -1, inout, free);
			pin = 38;
			break;

		case 8:
			gpio_set_config(40, -1, -1, -1, -1, -1, inout, free);
			pin = 40;
			break;

		case 9:
			gpio_set_config(4, 22, 70, 0, -1, -1, inout, free);
			pin = 4;
			break;

		case 10:
			gpio_set_config(10, 26, 74, 0, -1, -1, inout, free);
			pin = 10;
			break;

		case 11:
			gpio_set_config(5, 24, 44, 0, 72, 0, inout, free);
			pin = 5;
			break;

		case 12:
			gpio_set_config(15, 42, -1, -1, -1, -1, inout, free);
			pin = 15;
			break;			

		case 13:
			gpio_set_config(7, 30, 46, 0, -1, -1, inout, free);
			pin = 7;
			break;

		case 14:
			gpio_set_config(48, -1, -1, -1, -1, -1, inout, free);
			pin = 48;
			break;

		case 15:
			gpio_set_config(50, -1, -1, -1, -1, -1, inout, free);
			pin = 50;
			break;

		case 16:
			gpio_set_config(52, -1, -1, -1, -1, -1, inout, free);
			pin = 52;
			break;

		case 17:
			gpio_set_config(54, -1, -1, -1, -1, -1, inout, free);
			pin = 54;
			break;

		case 18:
			gpio_set_config(56, -1, 60, 1, 78, 1, inout, free);
			pin = 56;
			break;

		case 19:
			gpio_set_config(58, -1, 60, 1, 79, 1, inout, free);
			pin = 58;
			break;

		default:
			printk(KERN_INFO "Invalid pin number... Select another IO pin\n");
			pin = -1;
			break;
	}
	return pin;
}