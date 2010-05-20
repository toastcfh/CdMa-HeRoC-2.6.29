#ifndef _PROXIMITY_SENSOR_H
#define _PROXIMITY_SENSOR_H
#include<linux/earlysuspend.h>

#define PASSION_PROXIMITY_INT_N              (90)
#define PASSION_PROXIMITY_POWER_N            (120)

#define PROXIMITY_SENSOR_NAME "proximity_sensor"

struct proximity_platform_data {
	struct input_dev *input_dev;
	struct early_suspend early_suspend_proximity;
	int enable;
	int intr;
};


#endif
