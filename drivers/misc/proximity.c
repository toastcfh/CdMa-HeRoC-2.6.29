/*
 * drivers/misc/proximity.c - proximity sensor driver
 *
 *  Copyright (C) 2009 zion huang <zion_huang@htc.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include<linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/proximity.h>
#include <mach/vreg.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>

static struct proximity_platform_data *this_proximity;

static ssize_t proximity_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	ssize_t val;
	int err = 0;
	val = -1;
	sscanf(buf, "%u", &val);
	if (val < 0 || val > 1)
		return -EINVAL;

	if (val==1) {
		err = gpio_direction_output(this_proximity->enable,0);
		if (err < 0)
			printk(KERN_ERR"%s: gpio_direction_input failed\n", __func__);
		printk("this_proximity->enable = %d\n", gpio_get_value(this_proximity->enable));
	}
	if (val==0) {
		err = gpio_direction_output(this_proximity->enable,1);
		if (err < 0)
			printk(KERN_ERR"%s: gpio_direction_input failed\n", __func__);
		printk("this_proximity->enable = %d\n", gpio_get_value(this_proximity->enable));
	}
	return count;
}

static DEVICE_ATTR(proximity, 0644, NULL,	proximity_store);

static irqreturn_t proximity_interrupt(int irq, void *dev_id)
{
	struct proximity_platform_data *dev = dev_id;
//	printk(KERN_ERR"%s: enter, irq=%d, gpio=%d\n", __func__,dev->intr,gpio_get_value(dev->intr));
	printk(KERN_ERR"proxmity sensor : distance getting %s\n", gpio_get_value(dev->intr) ? "away":"close");

	input_report_abs(dev->input_dev, ABS_DISTANCE, gpio_get_value(dev->intr));
	set_irq_type(gpio_to_irq(dev->intr), gpio_get_value(dev->intr) ? IRQF_TRIGGER_LOW: IRQF_TRIGGER_HIGH);
	input_sync(dev->input_dev);

	return IRQ_HANDLED;
}

static int proximity_open(struct inode *inode, struct file *file)
{
	gpio_direction_output(this_proximity->enable,0);
	printk("this_proximity->enable = %d\n", gpio_get_value(this_proximity->enable));

	return 0;
}

static int proximity_release(struct inode *inode, struct file *file)
{
	gpio_direction_output(this_proximity->enable,1);
	printk("this_proximity->enable = %d\n", gpio_get_value(this_proximity->enable));

	return 0;
}

static void proximity_early_suspend(struct early_suspend *handler)
{
	printk(KERN_ERR"%s: enter\n", __func__);
	disable_irq(gpio_to_irq(this_proximity->intr));

	gpio_direction_output(this_proximity->enable,1);
}

static void proximity_early_resume(struct early_suspend *handler)
{
	printk(KERN_ERR"%s: enter\n", __func__);

	gpio_direction_output(this_proximity->enable,0);

	enable_irq(gpio_to_irq(this_proximity->intr));
}

static struct file_operations proximity_fops = {
	.owner = THIS_MODULE,
	.open = proximity_open,
	.release = proximity_release,
};

static struct miscdevice proximity_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity_sensor",
	.fops = &proximity_fops,
};

static int proximity_sensor_probe(struct platform_device *pdev)
{
	struct proximity_platform_data *proximity =pdev->dev.platform_data;
	int err = 0;

	printk(KERN_ERR"%s: enter, irq=%d\n", __func__,gpio_to_irq(proximity->intr));
	err = gpio_direction_output(48,0);
	if (err < 0)
		printk(KERN_ERR"%s: gpio_direction_input failed\n", __func__);

	err = gpio_request(proximity->enable, "proximity_sensor");
	if (err < 0)
		printk(KERN_ERR"%s: gpio_request failed\n", __func__);

	err = gpio_direction_output(proximity->enable,1);
	if (err < 0)
		printk(KERN_ERR"%s: gpio_direction_input failed\n", __func__);

	err = gpio_request(proximity->intr, "proximity_sensor");
	if (err < 0)
		printk(KERN_ERR"%s: gpio_request failed\n", __func__);

	err = gpio_direction_input(proximity->intr);
	if (err < 0)
		printk(KERN_ERR"%s: gpio_direction_input failed\n", __func__);

	proximity->input_dev = input_allocate_device();

	if (!proximity->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR"%s: Failed to allocate input device\n", __func__);
	}
	set_bit(EV_ABS, proximity->input_dev->evbit);
	input_set_abs_params(proximity->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	proximity->input_dev->name = "proximity";

	err = input_register_device(proximity->input_dev);

	#if 0
	err = request_irq(gpio_to_irq(proximity->intr), proximity_interrupt, IRQF_TRIGGER_LOW|IRQF_TRIGGER_HIGH,
	#else
	err = request_irq(MSM_GPIO_TO_INT(proximity->intr), proximity_interrupt, IRQF_TRIGGER_LOW|IRQF_TRIGGER_HIGH,
	#endif
				  "proximity", proximity);
	printk(KERN_ERR"proximity_sensor_probe(): proximity->intr=%d\n", proximity->intr);

	if (err < 0) {
		printk(KERN_ERR"%s: request irq failed, proximity: %p\n", __func__, proximity);
	}
	else
		printk(KERN_ERR"proximity_sensor_probe(): request_irq()=%d\n", err);

	err = misc_register(&proximity_device);
	if (err) {
		printk(KERN_ERR "%s: misc register failed\n",__func__);
	}
	else
		printk(KERN_ERR"proximity_sensor_probe(): misc_register()=%d\n", err);

	proximity->early_suspend_proximity.suspend = proximity_early_suspend;
	proximity->early_suspend_proximity.resume = proximity_early_resume;
	register_early_suspend(&proximity->early_suspend_proximity);

	err = device_create_file(&pdev->dev, &dev_attr_proximity);
	if (err)
		printk(KERN_ERR"%s: device_create_file failed\n", __func__);
	this_proximity = proximity;
//	disable_irq(gpio_to_irq(this_proximity->intr));

	return 0;
}

static int proximity_sensor_remove(struct platform_device *pdev)
{

	return 0;
}

static struct platform_driver proximity_sensor_driver = {
	.probe		= proximity_sensor_probe,
	.remove		= proximity_sensor_remove,
	.driver		= {
		.name		= PROXIMITY_SENSOR_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init proximity_sensor_init(void)
{
	printk(KERN_ERR"%s: enter\n", __func__);
	return platform_driver_register(&proximity_sensor_driver);
}

static void __exit proximity_sensor_exit(void)
{
	platform_driver_unregister(&proximity_sensor_driver);
}

module_init(proximity_sensor_init);
module_exit(proximity_sensor_exit);

MODULE_AUTHOR("zion huang <zion_huang@htc.com>");
MODULE_DESCRIPTION("proximity sensor driver");
MODULE_LICENSE("GPL");
