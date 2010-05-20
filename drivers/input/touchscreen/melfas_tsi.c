/* drivers/input/touchscreen/melfas_tsi.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2009 HTC Inc.
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/melfas_tsi.h>
#include <linux/io.h>
#include <linux/earlysuspend.h>
#include <mach/system.h>

#define ENABLE_MELFAS_TOUCH_DEBUGFS 1
#ifdef ENABLE_MELFAS_TOUCH_DEBUGFS
#include <linux/debugfs.h>
#include <linux/gpio.h>
#endif
#define MELFAS_I2C_RETRY_TIMES 10
#define MELFAS_I2C_WRITE_BLOCK_SIZE 2
#define PRESSURE_THR 10
#define WIDTH_THR 2

static struct workqueue_struct *melfas_wq;

struct melfas_ts_data {
	uint8_t sensor_revision; /* Sensor Revision */
	uint8_t hw_revision; /* Hardware Revision */
	uint8_t comp_group; /* Compatibility Group */
	uint8_t fw_version; /* Firmware Version */
	struct i2c_client *client;
	struct input_dev *input_dev, *input_key_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct work;
	struct melfas_virtual_key *virtual_key_value;
	uint8_t virtual_key_num;
	uint16_t virt_key_type;
	int (*power)(int on);
	int (*reset)(void);
	struct early_suspend early_suspend;
	struct mutex melfas_ts_mutex;
	struct device dev;
	int wake_up;
	uint8_t first_pressed;
	uint8_t key_pressed;
	uint16_t max_x;
	uint16_t max_y;
	uint8_t reported_finger_count;
	int tp_en;
#ifdef ENABLE_MELFAS_TOUCH_DEBUGFS
	uint8_t debug_log_level;
	int intr;
#endif
};

static int melfas_i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	for (retry = 0; retry < MELFAS_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		mdelay(10);
	}
	if (retry == MELFAS_I2C_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			MELFAS_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}

static int melfas_i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[10];
	int i;

	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = buf,
		}
	};

	if (length + 1 > MELFAS_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++) {
		buf[i+1] = data[i];
	}

	for (retry = 0; retry < MELFAS_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == MELFAS_I2C_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			MELFAS_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}


#define ENABLE_IME_IMPROVEMENT

#ifdef ENABLE_IME_IMPROVEMENT
#define MULTI_FINGER_NUMBER 2

static int ime_threshold;
static int report_x[MULTI_FINGER_NUMBER];
static int report_y[MULTI_FINGER_NUMBER];
static int touch_hit;
static int ime_work_area[4];
static int ime_work_area_x1;
static int ime_work_area_x2;
static int ime_work_area_y1;
static int ime_work_area_y2;
static int display_width = 320;
static int display_height = 480;

static ssize_t ime_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ime_threshold);
}

static ssize_t ime_threshold_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	char *ptr_data = (char *)buf;
	unsigned long val;

	val = simple_strtoul(ptr_data, NULL, 10);

	if (val >= 0 && val <= max(display_width, display_height))
		ime_threshold = val;
	else
		ime_threshold = 0;
	return count;
}

static ssize_t ime_work_area_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d,%d,%d,%d\n", ime_work_area[0],
			ime_work_area[1], ime_work_area[2], ime_work_area[3]);
}

static ssize_t ime_work_area_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	char *ptr_data = (char *)buf;
	char *p;
	int pt_count = 0;
	unsigned long val[4];

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 4)
			break;

		val[pt_count] = simple_strtoul(p, NULL, 10);

		pt_count++;
	}

	if (pt_count >= 4 && display_width && display_height) {
		ime_work_area[0] = val[0];
		ime_work_area[1] = val[1];
		ime_work_area[2] = val[2];
		ime_work_area[3] = val[3];

		if (val[0] <= 1)
			ime_work_area_x1 = 0;
		else
			ime_work_area_x1 = val[0];

		if (val[1] >= display_width - 1)
			ime_work_area_x2 = 319;
		else
			ime_work_area_x2 = val[1];

		if (val[2] <= 1)
			ime_work_area_y1 = 0;
		else
			ime_work_area_y1 = val[2];

		if (val[3] >= display_height - 1)
			ime_work_area_y2 = 479;
		else
			ime_work_area_y2 = val[3];
	}

	return count;
}

static void clear_queue(void)
{
	/* Clear report point coordinate */
	int i;

	for (i = 0; i < MULTI_FINGER_NUMBER; i++) {
		report_x[i] = 0;
		report_y[i] = 0;
	}

	touch_hit = 0;
}

static DEVICE_ATTR(ime_threshold, 0666, ime_threshold_show,
		ime_threshold_store);
static DEVICE_ATTR(ime_work_area, 0666, ime_work_area_show,
		ime_work_area_store);
#endif

static int melfas_i2c_op_mode_switch(struct i2c_client *client, uint16_t mode)
{
	int ret = 0;
	uint8_t dummy_data[1] = { 0 };
	uint32_t loop_i = 0;
	const uint8_t retry_times = 10;
	switch (mode) {
	case MELFAS_I2C_OP_ACTIVE_MODE: /* active mode */
		for (loop_i = 1; loop_i <= retry_times; loop_i++) {
			if (1 == (loop_i % 5)) {
				if (melfas_i2c_write_block(client, 0x55, NULL, 0) < 0)
					printk(KERN_ERR "%s: melfas_i2c_write_block failed (0x%X)\n",
						__func__, 0x55);
				mdelay(15);
			}
			mdelay(10);
			dummy_data[0] = 0;
			ret = melfas_i2c_read_block(client, MELFAS_I2C_CMD_OP_MODE, dummy_data, 1);
			if (ret < 0)
				printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
					__func__, MELFAS_I2C_CMD_OP_MODE);
			if (dummy_data[0] & MELFAS_I2C_OP_ACTIVE_MODE)
				break;
		}
		if (loop_i > retry_times) {
			printk(KERN_ERR "\n%s: Can't change operation mode: 0x%2.2X!!\n\n", __func__, mode);
			ret = -EIO;
		}
		break;
	case MELFAS_I2C_OP_IDLE_MODE: /* idle mode */
		dummy_data[0] = MELFAS_I2C_OP_IDLE_MODE;
		for (loop_i = 1; loop_i <= retry_times; loop_i++) {
			ret = melfas_i2c_write_block(client,
				MELFAS_I2C_CMD_OP_MODE, dummy_data, 1);
			if (ret < 0) {
				printk(KERN_ERR "%s: melfas_i2c_write_block failed (0x%X), retry now...\n",
					__func__, MELFAS_I2C_CMD_OP_MODE);
				msleep(10);
			} else
				break;
		}
		if (loop_i > retry_times) {
			printk(KERN_ERR "\n%s: Can't change operation mode: 0x%2.2X!!\n\n", __func__, mode);
			ret = -EIO;
		}
		break;
	case MELFAS_I2C_OP_SLEEP_MODE: /* sleep mode */
		dummy_data[0] = MELFAS_I2C_OP_SLEEP_MODE;
		for (loop_i = 1; loop_i <= retry_times; loop_i++) {
			ret = melfas_i2c_write_block(client,
				MELFAS_I2C_CMD_OP_MODE, dummy_data, 1);
			if (ret < 0) {
				printk(KERN_ERR "%s: melfas_i2c_write_block failed (0x%X), retry now...\n",
					__func__, MELFAS_I2C_CMD_OP_MODE);
				msleep(10);
			} else
				break;
		}
		if (loop_i > retry_times) {
			printk(KERN_ERR "\n%s: Can't change operation mode: 0x%2.2X!!\n\n", __func__, mode);
			ret = -EIO;
		}
		break;
	default:
		printk(KERN_WARNING "%s: switch to unknown mode??\n", __func__);
		break;
	};
	return ret;
}

static int melfas_ts_report(struct melfas_ts_data *ts)
{
	int ret;
	uint8_t buffer[11];
	uint16_t key_x_axis;
	uint8_t press_pressure;
	uint8_t press_width = 0;
	uint8_t finger_num;
	uint8_t key_value;
	int loop_i, loop_j;
	int finger2_pressed = 0;
	static uint16_t position_data[2][2];
	static uint16_t ref_z, ref_w, ref_pos[2][2];
	int point_moved = 1; /* for ENABLE_IME_IMPROVEMENT */

	int buf_len = (ts->sensor_revision == MELFAS_DIAMOND_PATTERN)? 11 : 10;

	mutex_lock(&ts->melfas_ts_mutex);
	memset(buffer, 0x0, sizeof(buffer));
	memset(position_data, 0x0, sizeof(position_data));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_INPUT_INFORMATION, buffer, buf_len);

	if (ret < 0) {
		printk(KERN_ERR "%s: get data fail: %X !!\n", __func__,
			MELFAS_I2C_CMD_INPUT_INFORMATION);
		mutex_unlock(&ts->melfas_ts_mutex);
		return ret;
	}

	key_x_axis = (buffer[0] & 0x80) << 1|buffer[4];
	key_value = buffer[6] & 0x7;
	press_pressure = buffer[3];

	if (buffer[buf_len - 1])
		press_width = (buffer[buf_len - 1] / 2) + (buffer[buf_len - 1] % 2);
	else
		press_width = 0;

	position_data[0][0] =  buffer[1] | (buffer[0] & 0x18) << 5;
	position_data[0][1] =  buffer[2] | (buffer[0] & 0x60) << 3;
	finger_num = buffer[0] & 0x3;
	finger2_pressed = (finger_num > 1 && finger_num <= 3);
	if (finger2_pressed) {
		position_data[1][0] = buffer[7] | (buffer[6] & 0x18) << 5;
		position_data[1][1] = buffer[8] | (buffer[6] & 0x60) << 3;
	}

#ifdef ENABLE_MELFAS_TOUCH_DEBUGFS
	if (ts->debug_log_level & 0x1) {
		printk("%s: dump raw data\n\t", __func__);
		for (loop_i = 0; loop_i < 11; loop_i++)
			printk("0x%2.2X ", buffer[loop_i]);
		printk("\n");
	}
#endif

	if (ts->sensor_revision == MELFAS_TRIANGLE_PATTERN)
		key_x_axis = key_x_axis  * 320 / 255;

	/* handle key event */
	if (key_x_axis) {
		/* In the virtual-key region, there is no Y value.	Since the
		* virtual-key region reports an X value in the range 0-255 / 0-319,
		* we need to scale it to the width of the display.
		*/
		if (key_x_axis != ref_pos[0][1]) {
			ref_pos[0][1] = key_x_axis;
#ifdef CONFIG_TOUCHSCREEN_CONCATENATE_REPORT
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 1 << 16 | 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION,
				1 << 31 | key_x_axis << 16 | 490);
#else
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, key_x_axis);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 490);
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);
#endif
#ifdef ENABLE_IME_IMPROVEMENT
			clear_queue();
#endif
			if (!ts->key_pressed) {
				ts->key_pressed = 1;
				if (!ts->first_pressed)
				printk(KERN_INFO "K@%d\n", key_x_axis);
			}
		}
		goto done;
	}

	if (!(buffer[0] & 0x3)) {
		/* When key_x_axis == 0 and the finger is not on the LCM part,
		* then the finger left the virtual key area.
		*/
#ifdef CONFIG_TOUCHSCREEN_CONCATENATE_REPORT
		input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
			0 << 16 | press_width);
		input_report_abs(ts->input_dev, ABS_MT_POSITION,
			1 << 31 | position_data[0][0] << 16 | position_data[0][1]);
#else
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(ts->input_dev);
#endif
#ifdef ENABLE_IME_IMPROVEMENT
					clear_queue();
#endif
		if (ts->key_pressed) {
			ts->key_pressed = 0;
			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				printk(KERN_INFO "E@%d, %d\n",
					position_data[0][0], position_data[0][1]);
			}
		}
	} else if ((buffer[0] & 0x03) == 3 || press_width > 10) {
		mutex_unlock(&ts->melfas_ts_mutex);
		return 0;
	} else {
		/* handle draw screen */
		if (abs(press_pressure - ref_z) <  PRESSURE_THR
			&& abs(press_width - ref_w) < WIDTH_THR && ts->key_pressed
			&& ref_pos[0][0] == position_data[0][0] && ref_pos[0][1] == position_data[0][1]
			&& ref_pos[1][0] == position_data[1][0] && ref_pos[1][1] == position_data[1][1])
			goto done;
		else {

#ifdef ENABLE_IME_IMPROVEMENT
			if (ime_threshold > 0) {
				int dx = 0;
				int dy = 0;
				int moved = 0;
				int finger = buffer[0] & 0x3;

				point_moved = 0;

				if ((position_data[0][0] >= ime_work_area_x1 && position_data[0][0] <= ime_work_area_x2) &&
					(position_data[0][1] >= ime_work_area_y1 && position_data[0][1] <= ime_work_area_y2)) {

					dx = abs(position_data[0][0] - report_x[0]);
					dy = abs(position_data[0][1] - report_y[0]);

					if ((dx > ime_threshold) || (dy > ime_threshold)) {
						if ((finger == 1) && (report_x[0] != 0) && (report_y[0] != 0))
							touch_hit = 1;

						moved = 1;
					}

					if (finger != 1) {
						moved = 0;
						report_x[0] = 0;
						report_y[0] = 0;
					}

					if (moved) {
						report_x[0] = position_data[0][0];
						report_y[0] = position_data[0][1];
					}

					if ((touch_hit == 1) || (moved == 1))
						point_moved = 1;
				} else { /* Not in IME work area */
					point_moved = 1;
				}
			}
			if (!point_moved)
				goto done;
#endif

#ifdef CONFIG_TOUCHSCREEN_CONCATENATE_REPORT
		input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
			press_pressure << 16 | press_width);
		input_report_abs(ts->input_dev, ABS_MT_POSITION,
			(!finger2_pressed) << 31 | position_data[0][0] << 16 | position_data[0][1]);
		if (finger2_pressed) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
				press_pressure << 16 | press_width);
			input_report_abs(ts->input_dev, ABS_MT_POSITION,
				1 << 31 | position_data[1][0] << 16 | position_data[1][1]);
		}
#else
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, press_pressure);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, press_width);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, position_data[0][0]);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, position_data[0][1]);
		input_mt_sync(ts->input_dev);
		if (finger2_pressed) {
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, press_pressure);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, press_width);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, position_data[1][0]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, position_data[1][1]);
			input_mt_sync(ts->input_dev);
		} else if (ts->reported_finger_count > 1) {
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(ts->input_dev);
		}
		ts->reported_finger_count = finger_num;
		input_sync(ts->input_dev);
#endif
		if (!ts->key_pressed) {
			ts->key_pressed = 1;
			if (!ts->first_pressed)
				printk(KERN_INFO "S@%d, %d\n",
					position_data[0][0], position_data[0][1]);
		}
		ref_z = press_pressure;
		ref_w = press_width;
		for (loop_i = 0; loop_i < 2; loop_i++)
			for (loop_j = 0; loop_j < 2; loop_j++)
			ref_pos[loop_i][loop_j] = position_data[loop_i][loop_j];
		}
	}

done:

#ifdef ENABLE_MELFAS_TOUCH_DEBUGFS
	if (ts->debug_log_level & 0x2)
		printk(KERN_INFO "%s: X1: %3d, Y1: %3d, X2: %3d, Y2: %3d\n"
			"\t\tZ: %3d (%d), key: %3d(%d), width: %2d\n",
			__func__, position_data[0][0], position_data[0][1],
			position_data[1][0], position_data[1][1],
			press_pressure, finger2_pressed, key_x_axis, key_value,
			press_width);
#endif
	mutex_unlock(&ts->melfas_ts_mutex);

	return ret;
}

static void melfas_ts_work_func(struct work_struct *work)
{
	int ret;
	struct melfas_ts_data *ts =
		container_of(work, struct melfas_ts_data, work);

	ret = melfas_ts_report(ts);
	if (ret < 0) {
		printk(KERN_ERR "%s: fail... %d\n", __func__, ret);
		ts->reset();
		melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_IDLE_MODE);
	}
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer,
					struct melfas_ts_data, timer);
	queue_work(melfas_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;
	disable_irq(ts->client->irq);
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	int ret;
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);

	if (ts->use_irq)
		disable_irq(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	/* if work was pending disable-count is now 2 */
	if (ret && ts->use_irq)
		enable_irq(ts->client->irq);

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0) {
			printk(KERN_ERR "%s: power off failed\n", __func__);
		}
	}
	if (ts->wake_up && (ts->sensor_revision == MELFAS_DIAMOND_PATTERN)) {
		gpio_set_value(ts->wake_up, 1);
		msleep(40);
		gpio_set_value(ts->wake_up, 0);
	}
	melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_SLEEP_MODE);
	gpio_direction_output(ts->tp_en, 0);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	int ret;
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "%s: power on failed\n", __func__);
		}
	}
	ts->first_pressed = 0;
	ts->key_pressed = 0;

	if (ts->wake_up && (ts->sensor_revision == MELFAS_DIAMOND_PATTERN)) {
		gpio_set_value(ts->wake_up, 1);
		msleep(40);
		gpio_set_value(ts->wake_up, 0);
	}

	if (ts->use_irq)
		enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	if (melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_IDLE_MODE) < 0) {
		ts->reset();
		melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_IDLE_MODE);
	}
	msleep(150);
	return;
}
#endif

static ssize_t melfas_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct melfas_ts_data *ts_data;
	ts_data = container_of(dev, struct melfas_ts_data, dev);
	sprintf(buf, "%s\n", MELFAS_I2C_NAME);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, 0444, melfas_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int malfas_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
		kobject_del(android_touch_kobj);
		return ret;
	}

	return 0;
}
#ifdef ENABLE_MELFAS_TOUCH_DEBUGFS
static int16_t debugfs_addr;
static int16_t debugfs_data_length;
static char debug_buffer[PAGE_SIZE];

static int16_t str_to_uint8_t(char *str)
{
	uint8_t loop_i;
	uint8_t str_len = strlen(str);
	uint8_t ret = 0;
	for (loop_i = 0; loop_i < str_len; loop_i++) {
		if (str[loop_i] >= 0x30 && str[loop_i] <= 0x39)
			ret |= (str[loop_i] - 0x30) << ((str_len - loop_i - 1) * 4);
		else if (str[loop_i] >= 0x41 && str[loop_i] <= 0x46)
			ret |= (str[loop_i] - 0x37) << ((str_len - loop_i - 1) * 4);
		else if (str[loop_i] >= 0x61 && str[loop_i] <= 0x66)
			ret |= (str[loop_i] - 0x57) << ((str_len - loop_i - 1) * 4);
		else {
			ret = -1;
			break;
		}
	}
	return ret;
}

static int melfas_ts_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t melfas_ts_debugfs_register_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	uint8_t data_tmp[10];
	int ret = 0;
	int loop_i;
	char *buf = debug_buffer;
	struct melfas_ts_data *ts = file->private_data;
	memset(data_tmp, 0x0, sizeof(data_tmp));
	memset(debug_buffer, 0x0, sizeof(debug_buffer));
	if (debugfs_data_length > 10) {
		printk(KERN_ERR "%s: size too big %d > 10\n",
			__func__, debugfs_data_length);
		return 0;
	}
	if (debugfs_addr && debugfs_data_length) {
		mutex_lock(&ts->melfas_ts_mutex);
		melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_ACTIVE_MODE);
		melfas_i2c_read_block(ts->client,
			(uint8_t)debugfs_addr, data_tmp,
			(uint8_t)debugfs_data_length);
		melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_IDLE_MODE);
		mutex_unlock(&ts->melfas_ts_mutex);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"addr: 0x%2.2X, data_length: 0x%2.2X, data:\n",
			debugfs_addr, debugfs_data_length);
		for (loop_i = 0; loop_i < debugfs_data_length; loop_i++)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"0x%2.2X ", data_tmp[loop_i]);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	} else
		printk(KERN_ERR "%s: unknown addr(0x%2.2X) or data length(0x%2.2X)\n",
			__func__, debugfs_addr, debugfs_data_length);
	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t melfas_ts_debugfs_register_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	char data_tmp[3];
	int16_t data_t;
	struct melfas_ts_data *ts = file->private_data;
	memset(data_tmp, 0x0, sizeof(data_tmp));
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
	     buf[4] == ':' && buf[7] == '\n') {
		data_tmp[0] = buf[2];
		data_tmp[1] = buf[3];
		debugfs_addr = str_to_uint8_t(data_tmp);
		if (debugfs_addr < 0) {
			printk(KERN_ERR "%s: wrong string: addr fail...", __func__);
			debugfs_addr = 0;
			return -1;
		}
		data_tmp[0] = buf[5];
		data_tmp[1] = buf[6];
		data_t  = str_to_uint8_t(data_tmp);
		if (data_t < 0) {
			printk(KERN_ERR "%s: wrong string: tmp fail...", __func__);
			return -1;
		}
		if (buf[0] == 'r') {
			debugfs_data_length = data_t;
		} else if (buf[0] == 'w') {
			mutex_lock(&ts->melfas_ts_mutex);
			melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_ACTIVE_MODE);
			melfas_i2c_write_block(ts->client,
				(uint8_t)debugfs_addr, (uint8_t *)&data_t, 1);
			melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_IDLE_MODE);
			mutex_unlock(&ts->melfas_ts_mutex);
		}
	}
	return count;
}

const struct file_operations debug_register_ops = {
	.open = melfas_ts_debugfs_open,
	.read = melfas_ts_debugfs_register_read,
	.write = melfas_ts_debugfs_register_write,
};

static ssize_t melfas_ts_debugfs_debug_level_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char *buf = debug_buffer;
	struct melfas_ts_data *ts = file->private_data;
	memset(debug_buffer, 0x0, sizeof(debug_buffer));
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d\n", ts->debug_log_level);
	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t melfas_ts_debugfs_debug_level_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	struct melfas_ts_data *ts = file->private_data;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->debug_log_level = buf[0] - 0x30;

	return count;
}

const struct file_operations debug_debug_level_ops = {
	.open = melfas_ts_debugfs_open,
	.read = melfas_ts_debugfs_debug_level_read,
	.write = melfas_ts_debugfs_debug_level_write,
};

static ssize_t melfas_ts_debugfs_interrupt_pin_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char *buf = debug_buffer;
	struct melfas_ts_data *ts = file->private_data;

	memset(debug_buffer, 0x0, sizeof(debug_buffer));
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d\n", gpio_get_value(ts->intr));
	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

const struct file_operations debug_interrupt_pin_ops = {
	.open = melfas_ts_debugfs_open,
	.read = melfas_ts_debugfs_interrupt_pin_read,
};

static void malfas_touch_debugfs_init(struct melfas_ts_data *ts)
{
	struct dentry *dent;
	dent = debugfs_create_dir("melfas_ts_debug", 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("register", 0644, dent, ts, &debug_register_ops);
	debugfs_create_file("debug_level", 0644, dent, ts, &debug_debug_level_ops);
	debugfs_create_file("interrupt_pin", 0644, dent, ts, &debug_interrupt_pin_ops);
}
#endif

static int melfas_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0;
	struct melfas_i2c_rmi_platform_data *pdata;
	uint8_t buffer[2];
	uint16_t loop_i;
#ifdef ENABLE_IME_IMPROVEMENT
	int i;
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: kzalloc fail...\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&ts->work, melfas_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	mutex_init(&ts->melfas_ts_mutex);

	if (pdata) {
		ts->power = pdata->power;
		ts->intr = pdata->intr;
		ts->wake_up = pdata->wake_up;
		ts->reset = pdata->reset;
		ts->tp_en = pdata->tp_en;
	}
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
	if (ts->wake_up) {
		gpio_set_value(ts->wake_up, 1);
		msleep(40);
		gpio_set_value(ts->wake_up, 0);
	}

	melfas_i2c_op_mode_switch(client, MELFAS_I2C_OP_ACTIVE_MODE);
	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_OP_MODE, buffer, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_OP_MODE);
		goto err_detect_failed;
	}
	printk(KERN_INFO "%s: detected done, Operation Mode: 0x%x\n",
		__func__, buffer[0]);

	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_SENSOR_REV, buffer, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_FIRMWARE_VER);
		goto err_detect_failed;
	}
	ts->sensor_revision = buffer[0];

	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_HARDWARE_REV, buffer, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_FIRMWARE_VER);
		goto err_detect_failed;
	}
	ts->hw_revision = buffer[0];

	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_COMPATILITY_GROUP_REV, buffer, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_FIRMWARE_VER);
		goto err_detect_failed;
	}
	ts->comp_group = buffer[0];

	while (pdata->version > ts->sensor_revision)
			pdata++;
	ts->virtual_key_value = pdata->virtual_key;
	ts->virtual_key_num = pdata->virtual_key_num;
	ts->virt_key_type = pdata->key_type;

	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_FIRMWARE_VER, buffer, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_FIRMWARE_VER);
		goto err_detect_failed;
	}
	ts->fw_version = buffer[0];

	/* Get the max_x and max_y info */
	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_X_SIZE, buffer, 2);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_X_SIZE);
		goto err_detect_failed;
	}
	ts->max_x = (buffer[0] & 0x3) << 8 | buffer[1];

	memset(buffer, 0x0, sizeof(buffer));
	ret = melfas_i2c_read_block(ts->client,
		MELFAS_I2C_CMD_Y_SIZE, buffer, 2);
	if (ret < 0) {
		printk(KERN_ERR "%s: melfas_i2c_read_block failed (0x%X)\n",
			__func__, MELFAS_I2C_CMD_Y_SIZE);
		goto err_detect_failed;
	}
	ts->max_y = (buffer[0] & 0x3) << 8 | buffer[1];

	printk(KERN_INFO "%s: sensor_rev: 0x%2.2X, hw_rev: 0x%2.2X\n"
		"    comp_group: 0x%2.2X, fw_ver: 0x%X\n"
		"    max X size: %d, max Y size: %d\n",
		__func__, ts->sensor_revision, ts->hw_revision,
		ts->comp_group, ts->fw_version,
		ts->max_x, ts->max_y);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
#ifdef CONFIG_TOUCHSCREEN_CONCATENATE_REPORT
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((255 << 16) | 255), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, ((1 << 31) | (ts->max_x << 16) | ts->max_y), 0, 0);
#endif
	ts->input_dev->name = "melfas-tsi-touchscreen";
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if (ts->virtual_key_num) {
		ts->input_key_dev = input_allocate_device();
		set_bit(ts->virt_key_type, ts->input_key_dev->evbit);
		for (loop_i = 0; loop_i < ts->virtual_key_num; loop_i++)
			set_bit(ts->virtual_key_value[loop_i].keycode,
				ts->input_key_dev->keybit);
		ts->input_key_dev->name = pdata->input_name;
		ret = input_register_device(ts->input_key_dev);
		if (ret) {
			printk(KERN_ERR "%s: Unable to register %s input device\n",
				__func__, ts->input_key_dev->name);
			goto err_input_key_register_device_failed;
		}
	}

#ifdef ENABLE_IME_IMPROVEMENT
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_ime_threshold);
	if (ret) {
		printk(KERN_ERR "ENABLE_IME_IMPROVEMENT: "
				"Error to create ime_threshold\n");
		goto err_input_register_device_failed;
	}
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_ime_work_area);
	if (ret) {
		printk(KERN_ERR "ENABLE_IME_IMPROVEMENT: "
				"Error to create ime_work_area\n");
		device_remove_file(&ts->input_dev->dev,
				   &dev_attr_ime_threshold);
		goto err_input_register_device_failed;
	}

	ime_threshold = 0;
	for (i = 0; i < MULTI_FINGER_NUMBER; i++) {
		report_x[i] = 0;
		report_y[i] = 0;
	}
	touch_hit = 0;
#endif

	if (client->irq) {
		ret = request_irq(client->irq, melfas_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
		if (ret == 0) {
			/* default value of 0x01 register should be 0x12 =>
			  * it should have been changed to interrupt mode
			  */
			memset(buffer, 0x0, sizeof(buffer));
			ret = melfas_i2c_read_block(ts->client,
				MELFAS_I2C_CMD_OP_MODE, buffer, 1);
			if (ret < 0 || (buffer[0] & (0x1 << 6))) {
				free_irq(client->irq, ts);
				printk(KERN_WARNING "%s: free client irq %d\n",
					__func__, client->irq);
			} else {
				ts->use_irq = 1;
			}
		}
		melfas_ts_report(ts);
	}
	melfas_i2c_op_mode_switch(ts->client, MELFAS_I2C_OP_IDLE_MODE);
	msleep(150); /*make sure no other i2c command to bother melfas*/
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "%s: Start touchscreen %s in %s mode\n",
		__func__, ts->input_dev->name,
		ts->use_irq ? "interrupt" : "polling");

	malfas_touch_sysfs_init();

	 set_melfas_reset_pin(ts->tp_en);
#ifdef ENABLE_MELFAS_TOUCH_DEBUGFS
	malfas_touch_debugfs_init(ts);
#endif
	return 0;

err_input_key_register_device_failed:
	input_free_device(ts->input_key_dev);

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
	kfree(ts);
	return ret;
}


static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver = {
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
	.id_table	= melfas_ts_id,
	.driver = {
		.name	= MELFAS_I2C_NAME,
	},
};

static int __devinit melfas_ts_init(void)
{
	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq)
		return -ENOMEM;
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
