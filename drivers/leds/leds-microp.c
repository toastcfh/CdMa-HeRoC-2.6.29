/* include/asm/mach-msm/leds-microp.c
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef CONFIG_MICROP_COMMON
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <mach/atmega_microp.h>

static int microp_write_led_mode(struct led_classdev *led_cdev,
				uint8_t mode, uint16_t off_timer)
{
	struct microp_led_data *ldata;
	uint8_t data[7];
	int ret;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	if (!strcmp(ldata->ldev.name, "green")) {
		data[0] = 0x01;
		data[1] = mode;
		data[2] = off_timer >> 8;
		data[3] = off_timer & 0xFF;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
	} else if (!strcmp(ldata->ldev.name, "amber")) {
		data[0] = 0x02;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = mode;
		data[5] = off_timer >> 8;
		data[6] = off_timer & 0xFF;
	} else if (!strcmp(ldata->ldev.name, "blue")) {
		data[0] = 0x04;
		data[1] = mode;
		data[2] = off_timer >> 8;
		data[3] = off_timer & 0xFF;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
	}

	ret = microp_i2c_write(MICROP_I2C_WCMD_LED_MODE, data, 7);
	if (ret == 0) {
		mutex_lock(&ldata->led_data_mutex);
		if (mode > 1)
			ldata->blink = mode;
		ldata->mode = mode;
		mutex_unlock(&ldata->led_data_mutex);
	}
	return ret;
}

static void microp_led_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct microp_led_data *ldata;
	unsigned long flags;
	int ret;
	uint8_t mode;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	if (brightness > 255)
		brightness = 255;
	led_cdev->brightness = brightness;

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	ldata->brightness = brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	if (brightness)
		mode = 1;
	else
		mode = 0;

	ret = microp_write_led_mode(led_cdev, mode, 0xffff);
	if (ret)
		pr_err("%s: led_brightness_set failed to set mode\n", __func__);
}

static void microp_led_jogball_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct microp_led_data *ldata;
	unsigned long flags;
	uint8_t data[3] = {0, 0, 0};
	int ret = 0;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	ldata->brightness = brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	switch (brightness) {
	case 0:
		data[0] = 0;
		break;
	case 1:
		data[0] = 3;
		data[1] = data[2] = 0xFF;
		break;
	case 3:
		data[0] = 1;
		data[1] = data[2] = 0xFF;
		break;
	case 7:
		data[0] = 2;
		data[1] = 0;
		data[2] = 60;
		break;
	default:
		pr_warning("%s: unknown value: %d\n", __func__, brightness);
		break;
	}
	ret = microp_i2c_write(MICROP_I2C_WCMD_JOGBALL_LED_MODE, data, 3);
	if (ret < 0)
		pr_err("%s failed on set jogball mode:0x%2.2X\n",
				__func__, data[0]);
}

static void microp_led_gpo_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct microp_led_data *ldata;
	unsigned long flags;
	uint8_t enable, addr, data[3] = {0, 0, 0};
	int ret = 0;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	ldata->brightness = brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	enable = brightness ? 1 : 0;
	if (enable)
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_EN;
	else
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_DIS;
	data[0] = ldata->led_config->mask_w[0];
	data[1] = ldata->led_config->mask_w[1];
	data[2] = ldata->led_config->mask_w[2];
	ret = microp_i2c_write(addr, data, 3);;
	if (ret < 0) {
		pr_err("%s failed on set gpo led mode:%d\n",
				__func__, brightness);
	}
}

static void microp_led_pwm_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct microp_led_data *ldata;
	unsigned long flags;
	uint8_t data[4] = {0, 0, 0, 0};
	int ret = 0;
	uint8_t value;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	spin_lock_irqsave(&ldata->brightness_lock, flags);
	ldata->brightness = brightness;
	spin_unlock_irqrestore(&ldata->brightness_lock, flags);

	value = brightness >= 255 ? 0x20 : 0;

	data[0] = ldata->led_config->fade_time;
	if (brightness)
		data[1] = ldata->led_config->init_value ?
			ldata->led_config->init_value :
			brightness;
	else
		data[1] = 0x00;
	data[2] = ldata->led_config->led_pin >> 8;
	data[3] = ldata->led_config->led_pin;

	ret = microp_i2c_write(MICROP_I2C_WCMD_LED_PWM, data, 4);
	if (ret < 0)
		pr_err("%s failed on set pwm led mode:0x%2.2X\n",
				__func__, data[1]);
}

static ssize_t microp_led_blink_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	int ret;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	mutex_lock(&ldata->led_data_mutex);
	ret = sprintf(buf, "%d\n", ldata->blink ? ldata->blink - 1 : 0);
	mutex_unlock(&ldata->led_data_mutex);

	return ret;
}

static ssize_t microp_led_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	int val, ret;
	uint8_t mode;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < 0 || val > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	mutex_lock(&ldata->led_data_mutex);
	switch (val) {
	case 0: /* stop flashing */
		ldata->blink = 0;
		if (led_cdev->brightness)
			mode = 1;
		else
			mode = 0;
		break;
	case 1:
	case 2:
	case 3:
		mode = val + 1;
		break;
	case 4:
		if (!strcmp(ldata->ldev.name, "amber")) {
			mode = val + 1;
			break;
		}
	default:
		mutex_unlock(&ldata->led_data_mutex);
		return -EINVAL;
	}
	mutex_unlock(&ldata->led_data_mutex);

	ret = microp_write_led_mode(led_cdev, mode, 0xffff);
	if (ret)
		pr_err("%s:%s set blink failed\n", __func__, led_cdev->name);

	return count;
}

static DEVICE_ATTR(blink, 0644, microp_led_blink_show,
					microp_led_blink_store);

static ssize_t microp_led_off_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	uint8_t data[2];
	int ret, offtime;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	dev_dbg(dev, "Getting %s remaining time\n", led_cdev->name);

	if (!strcmp(ldata->ldev.name, "green"))
		ret = microp_i2c_read(MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME,
				data, 2);
	else if (!strcmp(ldata->ldev.name, "amber"))
		ret = microp_i2c_read(MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME,
				data, 2);
	else if (!strcmp(ldata->ldev.name, "blue"))
		ret = microp_i2c_read(MICROP_I2C_RCMD_BLUE_LED_REMAIN_TIME,
				data, 2);
	else {
		pr_err("%s: Unknown led %s\n", __func__, ldata->ldev.name);
		return -EINVAL;
	}

	if (ret)
		pr_err("%s: %s get off_timer failed\n",
			__func__, led_cdev->name);

	offtime = (int)((data[1] | data[0] << 8) * 2);

	ret = sprintf(buf, "Time remains %d:%d\n", offtime / 60, offtime % 60);
	return ret;
}

static ssize_t microp_led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	int min, sec, ret;
	uint16_t off_timer;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	dev_dbg(dev, "Setting %s off_timer to %d min %d sec\n",
			led_cdev->name, min, sec);

	if (!min && !sec)
		off_timer = 0xFFFF;
	else
		off_timer = (min * 60 + sec) / 2;

	ret = microp_write_led_mode(led_cdev, ldata->mode, off_timer);
	if (ret)
		pr_err("%s: %s set off_timer %d min %d sec failed\n",
			__func__, led_cdev->name, min, sec);

	return count;
}

static DEVICE_ATTR(off_timer, 0644, microp_led_off_timer_show,
					microp_led_off_timer_store);

static ssize_t microp_jogball_color_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	int rpwm, gpwm, bpwm, ret;
	uint8_t data[4];

	rpwm = -1;
	gpwm = -1;
	bpwm = -1;
	sscanf(buf, "%d %d %d", &rpwm, &gpwm, &bpwm);

	if (rpwm < 0 || rpwm > 255)
		return -EINVAL;
	if (gpwm < 0 || gpwm > 255)
		return -EINVAL;
	if (bpwm < 0 || bpwm > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	dev_dbg(&client->dev, "Setting %s color to R=%d, G=%d, B=%d\n",
			led_cdev->name, rpwm, gpwm, bpwm);

	data[0] = rpwm;
	data[1] = gpwm;
	data[2] = bpwm;
	data[3] = 0x00;

	ret = microp_i2c_write(MICROP_I2C_WCMD_JOGBALL_LED_PWM_SET, data, 4);
	if (ret) {
		dev_err(&client->dev,
			"%s set color R=%d G=%d B=%d failed\n",
			led_cdev->name, rpwm, gpwm, bpwm);
	}
	return count;
}

static DEVICE_ATTR(color, 0644, NULL, microp_jogball_color_store);

static int microp_led_probe(struct platform_device *pdev)
{
	struct microp_led_platform_data *pdata;
	struct microp_led_data *ldata;
	int i, ret;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		pr_err("%s: platform data is NULL\n", __func__);
		return -ENODEV;
	}

	ldata = kzalloc(sizeof(struct microp_led_data)
			* pdata->num_leds, GFP_KERNEL);
	if (!ldata && pdata->num_leds) {
		ret = -ENOMEM;
		pr_err("%s: failed on allocate ldata\n", __func__);
		goto err_exit;
	}

	dev_set_drvdata(&pdev->dev, ldata);
	for (i = 0; i < pdata->num_leds; i++) {
		ldata[i].led_config = pdata->led_config + i;
		ldata[i].ldev.name = pdata->led_config[i].name;
		if (pdata->led_config[i].type == LED_JOGBALL)
			ldata[i].ldev.brightness_set
				= microp_led_jogball_brightness_set;
		else if (pdata->led_config[i].type == LED_GPO)
			ldata[i].ldev.brightness_set
				= microp_led_gpo_brightness_set;
		else if (pdata->led_config[i].type == LED_PWM)
			ldata[i].ldev.brightness_set
				= microp_led_pwm_brightness_set;
		else if (pdata->led_config[i].type == LED_RGB)
			ldata[i].ldev.brightness_set
				= microp_led_brightness_set;

		mutex_init(&ldata[i].led_data_mutex);
		spin_lock_init(&ldata[i].brightness_lock);
		ret = led_classdev_register(&pdev->dev, &ldata[i].ldev);
		if (ret < 0) {
			pr_err("%s: failed on led_classdev_register [%s]\n",
				__func__, ldata[i].ldev.name);
			goto err_register_led_cdev;
		}
	}

	for (i = 0; i < pdata->num_leds; i++) {
		if (pdata->led_config[i].type != LED_RGB)
			continue;
		ret = device_create_file(ldata[i].ldev.dev, &dev_attr_blink);
		if (ret < 0) {
			pr_err("%s: failed on create attr blink [%d]\n",
				__func__, i);
			goto err_register_attr_blink;
		}
	}

	for (i = 0; i < pdata->num_leds; i++) {
		if (pdata->led_config[i].type != LED_RGB)
			continue;
		ret = device_create_file(ldata[i].ldev.dev,
				&dev_attr_off_timer);
		if (ret < 0) {
			pr_err("%s: failed on create attr off_timer [%d]\n",
				__func__, i);
			goto err_register_attr_off_timer;
		}
	}

	for (i = 0; i < pdata->num_leds; i++) {
		if (pdata->led_config[i].type != LED_JOGBALL)
			continue;
		ret = device_create_file(ldata[i].ldev.dev, &dev_attr_color);
		if (ret < 0) {
			pr_err("%s: failed on create attr jogball color\n",
				__func__);
			goto err_register_attr_color;
		} else
			break;
	}
	return 0;

err_register_attr_color:
	i = pdata->num_leds;

err_register_attr_off_timer:
	for (i--; i >= 0; i--) {
		if (pdata->led_config[i].type != LED_RGB)
			continue;
		device_remove_file(ldata[i].ldev.dev,
				&dev_attr_off_timer);
	}
	i = pdata->num_leds;
err_register_attr_blink:
	for (i--; i >= 0; i--) {
		if (pdata->led_config[i].type != LED_RGB)
			continue;
		device_remove_file(ldata[i].ldev.dev,
				&dev_attr_blink);
	}
	i = pdata->num_leds;

err_register_led_cdev:
	for (i--; i >= 0; i--)
		led_classdev_unregister(&ldata[i].ldev);
	kfree(ldata);

err_exit:
	return ret;
}

static int __devexit microp_led_remove(struct platform_device *pdev)
{
	struct microp_led_platform_data *pdata;
	struct microp_led_data *ldata;
	int i;

	pdata = pdev->dev.platform_data;
	ldata = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&ldata[i].ldev);
		if (pdata->led_config[i].type == LED_RGB) {
			device_remove_file(ldata[i].ldev.dev,
				&dev_attr_off_timer);
			device_remove_file(ldata[i].ldev.dev,
				&dev_attr_blink);
		} else if (pdata->led_config[i].type == LED_JOGBALL)
			device_remove_file(ldata[i].ldev.dev, &dev_attr_color);
	}
	kfree(ldata);

	return 0;
}

static struct platform_driver microp_led_driver = {
	.probe = microp_led_probe,
	.remove = __devexit_p(microp_led_remove),
	.driver = {
		   .name = "leds-microp",
		   .owner = THIS_MODULE,
		   },
};

int __init microp_led_init(void)
{
	return platform_driver_register(&microp_led_driver);
}

void microp_led_exit(void)
{
	platform_driver_unregister(&microp_led_driver);
}
#endif /* end of #ifdef CONFIG_MICROP_COMMON*/

