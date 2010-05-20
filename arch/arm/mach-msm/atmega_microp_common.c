/* arch/arm/mach-msm/atmega_microp_common.c
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
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <mach/atmega_microp.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <mach/drv_callback.h>
#include <mach/htc_35mm_remote.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include "proc_comm.h"

#define I2C_READ_RETRY_TIMES			10
#define I2C_WRITE_RETRY_TIMES			10
#define MICROP_I2C_WRITE_BLOCK_SIZE	31

#ifdef CONFIG_HTC_HEADSET
#define notify_35mm_headset_insert(insert) \
	htc_35mm_remote_notify_insert_ext_headset(insert)
#define notify_35mm_button_status(key_level) \
	htc_35mm_remote_notify_button_status(key_level)
#else
#define notify_35mm_headset_insert(insert) do {} while (0)
#define notify_35mm_button_status(key_level) do {} while (0)
#endif

/*
 * TODO: pass from board
 * */
#ifdef CONFIG_MACH_SUPERSONIC
#define MICROP_INT_MASK (0x2)
#endif

int __init microp_led_init(void);
void microp_led_exit(void);

static struct i2c_client *private_microp_client;
static struct microp_oj_callback *oj_callback;
static struct microp_psensor_callback *ps_callback;
static struct microp_bl_callback *bl_callback;
static struct microp_gs_callback *gs_callback;
static struct microp_ops *board_ops;

static int is_35mm_hpin;

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_read_block(struct i2c_client *client, uint8_t addr,
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

	mdelay(1);
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		mdelay(5);
	}

	dev_dbg(&client->dev, "R [%02X] = %s\n",
			addr, hex2string(data, length));

	if (retry > I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int i;

	struct i2c_msg msg[] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = buf,
	}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mdelay(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(5);
	}
	if (retry > I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

int microp_i2c_read(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_read_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_read);

int microp_i2c_write(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_write_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_write);

int microp_register_oj_callback(struct microp_oj_callback *oj)
{
	oj_callback = oj;

	if (private_microp_client != NULL) {
		oj_callback->oj_init = NULL;
		return 0;
	}

	return 1;
}

int microp_register_ps_callback(struct microp_psensor_callback *ps)
{
	ps_callback = ps;

	if (private_microp_client != NULL) {
		ps_callback->ps_init = NULL;
		return 0;
	}

	return 1;
}

int microp_register_gs_callback(struct microp_gs_callback *gs)
{
	gs_callback = gs;

	if (private_microp_client != NULL) {
		gs_callback->gs_init = NULL;
		return 0;
	}

	return 1;
}

void microp_register_backlight_callback(struct microp_bl_callback *bl)
{
	bl_callback = bl;
}

void microp_register_ops(struct microp_ops *ops)
{
	board_ops = ops;
}

struct i2c_client *get_microp_i2c_client(void)
{
	return private_microp_client;
}

EXPORT_SYMBOL(get_microp_i2c_client);


/* add microp notify behavior ===== */
static LIST_HEAD(g_microp_notifier_list);
static DEFINE_MUTEX(notify_sem);

static void send_interrupt_notify(uint16_t intr_status)
{
	static struct t_microp_interrupt_notifier *notifier;

	mutex_lock(&notify_sem);
	list_for_each_entry(notifier,
		&g_microp_notifier_list,
		notifier_link) {
			if (intr_status & (notifier->intr_pin)) {
				if (notifier->func != NULL) {
					notifier->func();
				}
			}
		}
	mutex_unlock(&notify_sem);
}

struct microp_function_config *get_microp_function(const char *name)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_function_config	*microp_function = NULL;
	struct i2c_client *client = private_microp_client;
	int i;
	pdata = client->dev.platform_data;

	for (i = 0; i < pdata->num_functions; i++) {
		if (!strcmp(pdata->microp_function[i].name, name)) {
			microp_function = &pdata->microp_function[i];
			break;
		}
	}
	return microp_function;
}

struct microp_function_config	*microp_register_notifier \
	(struct t_microp_interrupt_notifier *notifier)
{
	struct microp_function_config	*microp_fun;

	if (!notifier || !notifier->name || !notifier->func)
		return NULL;

	microp_fun = get_microp_function(notifier->name);
	if (microp_fun) {
		notifier->intr_pin = microp_fun->int_pin;
		mutex_lock(&notify_sem);
		list_add(&notifier->notifier_link,
			&g_microp_notifier_list);
		mutex_unlock(&notify_sem);
	}

	return microp_fun;
}

/* END: add microp interrupt notify behavior ===== */

int microp_function_check(struct i2c_client *client, uint8_t category)
{
	struct microp_i2c_platform_data *pdata;
	int i, ret = -1;

	pdata = client->dev.platform_data;

	for (i = 0; i < pdata->num_functions; i++) {
		if (pdata->microp_function[i].category == category) {
			ret = i;
			break;
		}
	}
	if (ret < 0)
		pr_err("%s: No function %d !!\n", __func__, category);

	return ret;
}

static int microp_gpo_set(uint8_t node, uint8_t enable)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[3], addr;

	pdata = client->dev.platform_data;
	if (enable)
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_EN;
	else
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_DIS;
	data[0] = pdata->microp_function[node].mask_w[0];
	data[1] = pdata->microp_function[node].mask_w[1];
	data[2] = pdata->microp_function[node].mask_w[2];
	return i2c_write_block(client, addr, data, 3);
}

int microp_write_interrupt(struct i2c_client *client,
		uint16_t interrupt, uint8_t enable)
{
	uint8_t data[2], addr;
	int ret = -1;

	if (enable)
		addr = MICROP_I2C_WCMD_GPI_INT_CTL_EN;
	else
		addr = MICROP_I2C_WCMD_GPI_INT_CTL_DIS;

	data[0] = interrupt >> 8;
	data[1] = interrupt & 0xFF;
	ret = i2c_write_block(client, addr, data, 2);

	if (ret < 0)
		dev_err(&client->dev, "%s: %s 0x%x interrupt failed\n",
			__func__, (enable ? "enable" : "disable"), interrupt);
	return ret;
}

int microp_read_adc(uint8_t *data)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret = 0;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	mutex_lock(&cdata->microp_i2c_mutex);
	if (i2c_write_block(client, MICROP_I2C_WCMD_READ_ADC_VALUE_REQ,
			data, 2) < 0) {
		dev_err(&client->dev, "%s: request adc fail\n", __func__);
		ret = -EIO;
		goto exit;
	}
	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_ADC_VALUE, data, 2) < 0) {
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
		ret = -EIO;
		goto exit;
	}
exit:
	mutex_unlock(&cdata->microp_i2c_mutex);
	return ret;
}

EXPORT_SYMBOL(microp_read_adc);

int microp_read_gpio_status(uint8_t *data)
{
	struct i2c_client *client;
	struct microp_i2c_platform_data *pdata;
	int length;

	client = private_microp_client;
	pdata = client->dev.platform_data;

	if (pdata->cmd_diff & CMD_83_DIFF)
		length = 2;
	else
		length = 3;
	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_GPIO_STATUS,
			data, length) < 0) {
		dev_err(&client->dev, "%s: read gpio status fail\n", __func__);
		return -EIO;
	}
	return 0;
}

static int get_remote_keycode(uint8_t *data)
{
	struct i2c_client *client = private_microp_client;

	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_REMOTE_KEYCODE,
			data, 2) < 0) {
		dev_err(&client->dev, "%s: read remote keycode fail\n", __func__);
		return -EIO;
	}
	if (!data[1])
		return 1;			/*no keycode*/
	else if (data[1] & 0x80)
		data[1] = 0x00;	/*release keycode*/

	return 0;
}

static void microp_hpin_work_func(struct work_struct *work)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret = 0;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	ret = microp_write_interrupt(client,
				cdata->int_pin.int_remotekey, 1);
	if (ret < 0)
		printk(KERN_ERR "%s: set remote key interrupt error\n",
			__func__);
}

static void microp_pm_power_off(struct i2c_client *client)
{
	return;
}

static void microp_reset_system(void)
{
	return;
}

static int microp_oj_intr_enable(struct i2c_client *client, uint8_t enable)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);
	enable = enable ? 1 : 0;
	return microp_write_interrupt(client,
			cdata->int_pin.int_oj, enable);
}

static int microp_spi_enable(struct i2c_client *client, uint8_t enable)
{
	uint8_t data;
	int ret = 0;

	data = enable ? 1 : 0;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_SPI_EN, &data, 1);
	if (ret != 0)
		printk(KERN_ERR "%s: set SPI %s fail\n", __func__,
			(enable ? "enable" : "disable"));

	return ret;
}

int microp_spi_vote_enable(int spi_device, uint8_t enable)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[2] = {0, 0};
	int ret = 0;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	if (spi_device == SPI_OJ)
		microp_oj_intr_enable(client, enable);

	mutex_lock(&cdata->microp_i2c_mutex);
	if (enable)
		cdata->spi_devices_vote |= spi_device;
	else
		cdata->spi_devices_vote &= ~spi_device;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_SPI_BL_STATUS, data, 2);
	if (ret != 0) {
		printk(KERN_ERR "%s: read SPI/BL status fail\n", __func__);
		goto exit;
	}

	if ((data[1] & 0x01) ==
		((pdata->spi_devices & cdata->spi_devices_vote) ? 1 : 0))
		goto exit;

	if (pdata->spi_devices & cdata->spi_devices_vote)
		enable = 1;
	else
		enable = 0;
	mutex_unlock(&cdata->microp_i2c_mutex);

	ret = microp_spi_enable(client, enable);
	return ret;

exit:
	mutex_unlock(&cdata->microp_i2c_mutex);
	return ret;

}

EXPORT_SYMBOL(microp_spi_vote_enable);

static void microp_reset_microp(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;

	pdata = client->dev.platform_data;

	gpio_set_value(pdata->gpio_reset, 0);
	udelay(120);
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(5);
}

int microp_notify_mic_value(void)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[2], node = 0;
	int ret = 0;

	client = private_microp_client;
	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	node = pdata->function_node[MICROP_FUNCTION_REMOTEKEY];
	data[0] = 0x00;
	data[1] = pdata->microp_function[node].channel;
	microp_read_adc(data);
	if ((data[0] << 8 | data[1]) >= 200) {
		ret = 1;
	} else
		ret = 0;
	schedule_delayed_work(&cdata->hpin_enable_intr_work, msecs_to_jiffies(700));

	printk(KERN_DEBUG "%s: microp_mic_status =0x%d\n", __func__, ret);
	is_35mm_hpin = 1;

	return ret;
}

int microp_notify_unplug_mic(void)
{
	struct microp_i2c_platform_data *pdata;
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	int ret;

	is_35mm_hpin = 0;

	if (!client)	{
		dev_err(&client->dev, "%s: dataset: client is empty\n",
				__func__);
		return -1;
	}
	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	ret = microp_write_interrupt(client,
				cdata->int_pin.int_remotekey, 0);
	if (ret < 0)
		dev_err(&client->dev, "%s: set remote key interrupt error\n",
				__func__);

	printk(KERN_INFO"%s: headset plug out\n", __func__);

	return ret;
}

static void hpin_debounce_do_work(struct work_struct *work)
{
	uint8_t data[3];
	struct microp_i2c_client_data *cdata;
	int insert = 0;
	struct i2c_client *client;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	microp_read_gpio_status(data);
	insert = (((data[0] << 16 | data[1] << 8 | data[2])
			& cdata->gpio.hpin) == 0)
			? 1 : 0;
	if (insert != cdata->headset_is_in) {
		cdata->headset_is_in = insert;
		notify_35mm_headset_insert(cdata->headset_is_in);
	}
}

static ssize_t microp_version_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%04X\n", cdata->version);
}

static DEVICE_ATTR(version, 0644, microp_version_show, NULL);

static ssize_t microp_reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val != 1)
		return -EINVAL;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	microp_reset_microp(client);
	if (board_ops->init_microp_func)
		board_ops->init_microp_func(client);

	return count;
}

static DEVICE_ATTR(reset, 0644, NULL, microp_reset_store);

static ssize_t microp_remotekey_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[3], node = 0;
	int ret;

	client = to_i2c_client(dev);
	pdata = client->dev.platform_data;

	node = pdata->function_node[MICROP_FUNCTION_REMOTEKEY];
	data[0] = 0x00;
	data[1] = pdata->microp_function[node].channel;
	microp_read_adc(data);
	ret = sprintf(buf,
				"Remote Key[0x%03X] => button %d\n",
				(data[0] << 8 | data[1]), data[2]);

	return ret;
}

static DEVICE_ATTR(key_adc, 0644, microp_remotekey_adc_show, NULL);

static ssize_t microp_gpio_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t data[3] = {0, 0, 0};
	int ret;

	microp_read_gpio_status(data);
	ret = sprintf(buf, "PB = 0x%x, PC = 0x%x, PD = 0x%x\n",
				data[0], data[1], data[2]);

	return ret;
}

static ssize_t microp_gpio_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int enable = 0, tmp[3] = {0, 0, 0};
	uint8_t addr, data[3] = {0, 0, 0};

	sscanf(buf, "%d %d %d %d", &enable, &tmp[0], &tmp[1], &tmp[2]);

	if (enable != 0 && enable != 1)
		return -EINVAL;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	if (enable)
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_EN;
	else
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_DIS;
	data[0] = (uint8_t)tmp[0];
	data[1] = (uint8_t)tmp[1];
	data[2] = (uint8_t)tmp[2];
	i2c_write_block(client, addr, data, 3);

	return count;
}

static DEVICE_ATTR(gpio, 0644,  microp_gpio_show,
			microp_gpio_store);

static irqreturn_t microp_intr_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = to_i2c_client(dev_id);
	cdata = i2c_get_clientdata(client);

	disable_irq(client->irq);
	queue_work(cdata->microp_queue, &cdata->microp_intr_work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_MACH_SUPERSONIC
static void microp_int_dispatch(u32 status)
{
	unsigned int mask;
	int irq;

	printk(KERN_DEBUG "%s\n", __func__);
	while (status) {
		mask = status & -status;
		irq = fls(mask) - 1;
		status &= ~mask;
		printk(KERN_DEBUG "%s: %d\n", __func__, FIRST_MICROP_IRQ + irq);
		generic_handle_irq(FIRST_MICROP_IRQ + irq);
	}
}
#endif

static void microp_intr_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[3], node = 0;
	uint16_t intr_status = 0;
	int sd_insert = 0, keycode = 0, ps_data = 0;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}

	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_GPI_INT_STATUS,
			data, 2) < 0)
		dev_err(&client->dev, "%s: read interrupt status fail\n",
				__func__);
	intr_status = data[0]<<8 | data[1];
	if (i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_STATUS_CLR,
			data, 2) < 0)
		dev_err(&client->dev, "%s: clear interrupt status fail\n",
				__func__);

	dev_info(&client->dev, "intr_status= %x\n", intr_status);

	send_interrupt_notify(intr_status);

#ifdef CONFIG_MACH_SUPERSONIC
	/* these interrupt is not handled by microp */
	if (intr_status & MICROP_INT_MASK)
		microp_int_dispatch(intr_status);
#endif
	/*
	if (intr_status & cdata->int_pin.int_lsensor) {

	}
	*/
	if (intr_status & cdata->int_pin.int_remotekey) {
		node = pdata->function_node[MICROP_FUNCTION_REMOTEKEY];
		if (get_remote_keycode(data) == 0) {
			keycode = (int)data[1];

			if (cdata->int_pin.int_hpin) {
				if (cdata->headset_is_in)
					notify_35mm_button_status(keycode);
			} else
				if (is_35mm_hpin)
					notify_35mm_button_status(keycode);
		}
	}
	if (intr_status & cdata->int_pin.int_oj) {
		data[0] = 0x00;
		if (i2c_write_block(client, MICROP_I2C_WCMD_OJ_INT_STATUS,
				data, 1) < 0)
			dev_err(&client->dev, "%s: clear OJ interrupt status fail\n",
				__func__);
		if (oj_callback && oj_callback->oj_intr)
			oj_callback->oj_intr();
	}
	if (intr_status & cdata->int_pin.int_reset) {
		dev_info(&client->dev, "Reset button is pressed\n");
		microp_reset_system();
	}
	if (intr_status & cdata->int_pin.int_simcard) {
		dev_info(&client->dev, "SIM Card is plugged/unplugged\n");
		microp_pm_power_off(client);
	}
	if (intr_status & cdata->int_pin.int_psensor) {
		dev_info(&client->dev, "P Sensor detected\n");
		microp_read_gpio_status(data);
		ps_data = ((data[0] << 16 | data[1] << 8 | data[2])
				& cdata->gpio.psensor) ? 1 : 0;
		if (ps_callback && ps_callback->ps_intr)
			ps_callback->ps_intr(ps_data);
	}
	if (intr_status & cdata->int_pin.int_sdcard) {
		dev_info(&client->dev, "SD Card is plugged/unplugged\n");
		msleep(300);
		microp_read_gpio_status(data);
		sd_insert = ((data[0] << 16 | data[1] << 8 | data[2])
				& cdata->gpio.sdcard) ? 1 : 0;
		if (sd_insert != cdata->sdcard_is_in) {
			cdata->sdcard_is_in = sd_insert;
			cnf_driver_event("sdcard_detect", &cdata->sdcard_is_in);
		}
	}
	if (intr_status & cdata->int_pin.int_hpin) {
		wake_lock_timeout \
					(&cdata->hpin_wake_lock, 3*HZ);
		dev_info(&client->dev, "HPIN insert/remove\n");
		if (!cdata->headset_is_in)
			schedule_delayed_work(&cdata->hpin_debounce_work, msecs_to_jiffies(500));
		else
			schedule_delayed_work(&cdata->hpin_debounce_work, msecs_to_jiffies(300));
	}

	enable_irq(client->irq);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void microp_early_suspend(struct early_suspend *h)
{
	struct microp_i2c_client_data *cdata;
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_platform_data *pdata;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	atomic_set(&cdata->microp_is_suspend, 1);
/*
	if (atomic_read(&cdata->als_intr_enable_flag)) {
		ret = microp_auto_backlight_enable(client, 0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			atomic_set(&cdata->als_intr_enabled, 0);
	}
	if (board_ops->als_pwr_enable)
		board_ops->als_pwr_enable(LS_PWR_ON, 0);
*/
}

static void microp_late_resume(struct early_suspend *h)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;
/*
	if (board_ops->als_pwr_enable) {
		board_ops->als_pwr_enable(LS_PWR_ON, 1);
		delay_time = 800;
	}
	schedule_delayed_work(&cdata->ls_on_work,
				msecs_to_jiffies(delay_time));
*/
	atomic_set(&cdata->microp_is_suspend, 0);
}
#endif

static int __devexit microp_i2c_remove(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cdata->early_suspend);
#endif

	if (client->irq)
		free_irq(client->irq, &client->dev);

	gpio_free(pdata->gpio_reset);

	device_remove_file(&client->dev, &dev_attr_reset);
	device_remove_file(&client->dev, &dev_attr_version);
	//device_remove_file(&client->dev, &dev_attr_ls_adc);
	device_remove_file(&client->dev, &dev_attr_key_adc);
	//device_remove_file(&client->dev, &dev_attr_ls_auto);
	device_remove_file(&client->dev, &dev_attr_gpio);
	destroy_workqueue(cdata->microp_queue);
	microp_led_exit();
	kfree(cdata);

	return 0;
}

static int microp_i2c_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
	return 0;
}

static int microp_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static int microp_i2c_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	int ret;

	private_microp_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "failed on get pdata\n");
		goto err_exit;
	}
	pdata->dev_id = (void *)&client->dev;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_VERSION, data, 2);
	if (ret || !(data[0] && data[1])) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp version\n");
		goto err_exit;
	}
	dev_info(&client->dev, "microp version [%02X][%02X]\n",
			data[0], data[1]);

	ret = gpio_request(pdata->gpio_reset, "atmega_microp");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio reset\n");
		goto err_exit;
	}
	ret = gpio_direction_output(pdata->gpio_reset, 1);
	if (ret < 0) {
		dev_err(&client->dev,
				"failed on gpio_direction_output reset\n");
		goto err_gpio_reset;
	}

	cdata = kzalloc(sizeof(struct microp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);
	cdata->version = data[0] << 8 | data[1];
	atomic_set(&cdata->microp_is_suspend, 0);
/*
	atomic_set(&cdata->als_intr_enabled, 0);
	atomic_set(&cdata->als_intr_enable_flag, 0);
*/
	cdata->spi_devices_vote = pdata->spi_devices_init;
/*
	cdata->als_kadc = 1;
	cdata->als_gadc = 1;
*/
	ret = microp_led_init();
	if (ret < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on initialize led driver\n");
		goto err_led_init;
	}

	mutex_init(&cdata->microp_i2c_mutex);

	cdata->microp_queue = create_singlethread_workqueue("microp_work_q");
	if (cdata->microp_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}

	if (client->irq) {
		INIT_WORK(&cdata->microp_intr_work, microp_intr_work_func);

		ret = request_irq(client->irq, microp_intr_irq_handler,
			IRQF_TRIGGER_LOW, "microp_intrrupt",
			&client->dev);
		if (ret) {
			dev_err(&client->dev, "request_irq failed\n");
			goto err_intr;
		}
		ret = set_irq_wake(client->irq, 1);
		if (ret) {
			dev_err(&client->dev, "set_irq_wake failed\n");
			goto err_intr;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	cdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cdata->early_suspend.suspend = microp_early_suspend;
	cdata->early_suspend.resume = microp_late_resume;
	register_early_suspend(&cdata->early_suspend);
#endif
	ret = device_create_file(&client->dev, &dev_attr_reset);
	ret = device_create_file(&client->dev, &dev_attr_version);
	ret = device_create_file(&client->dev, &dev_attr_key_adc);
	ret = device_create_file(&client->dev, &dev_attr_gpio);

	if (board_ops->init_microp_func) {
		ret = board_ops->init_microp_func(client);
		if (ret) {
			dev_err(&client->dev,
				"failed on microp function initialize\n");
			goto err_fun_init;
		}
	}
/*
	INIT_DELAYED_WORK(&cdata->ls_on_work,
			microp_als_on_work_func);
*/
	INIT_DELAYED_WORK(&cdata->hpin_enable_intr_work,
			microp_hpin_work_func);
	INIT_DELAYED_WORK(&cdata->hpin_debounce_work,
			hpin_debounce_do_work);

	if (oj_callback && oj_callback->oj_init)
		oj_callback->oj_init();

	if (ps_callback && ps_callback->ps_init)
		ps_callback->ps_init();

	if (gs_callback && gs_callback->gs_init)
		(*gs_callback->gs_init)();

	return 0;

err_fun_init:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cdata->early_suspend);
#endif
	device_remove_file(&client->dev, &dev_attr_reset);
	device_remove_file(&client->dev, &dev_attr_version);
	//device_remove_file(&client->dev, &dev_attr_ls_adc);
	device_remove_file(&client->dev, &dev_attr_key_adc);
	//device_remove_file(&client->dev, &dev_attr_ls_auto);
	device_remove_file(&client->dev, &dev_attr_gpio);
	destroy_workqueue(cdata->microp_queue);
err_intr:
err_create_work_queue:
	microp_led_exit();
err_led_init:
	kfree(cdata);
err_cdata:
err_gpio_reset:
	gpio_free(pdata->gpio_reset);
err_exit:
	private_microp_client = NULL;
	return ret;
}

static const struct i2c_device_id microp_i2c_id[] = {
	{ MICROP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver microp_i2c_driver = {
	.driver = {
		   .name = MICROP_I2C_NAME,
		   },
	.id_table = microp_i2c_id,
	.probe = microp_i2c_probe,
	.suspend = microp_i2c_suspend,
	.resume = microp_i2c_resume,
	.remove = __devexit_p(microp_i2c_remove),
};

static void microp_irq_ack(unsigned int irq)
{
	;
}

static void microp_irq_mask(unsigned int irq)
{
	;
}

static void microp_irq_unmask(unsigned int irq)
{
	;
}

static struct irq_chip microp_irq_chip = {
	.name = "microp",
	.disable = microp_irq_mask,
	.ack = microp_irq_ack,
	.mask = microp_irq_mask,
	.unmask = microp_irq_unmask,
};

static int __init microp_common_init(void)
{
	int ret;
	int n, MICROP_IRQ_END = FIRST_MICROP_IRQ + NR_MICROP_IRQS;

	ret = i2c_add_driver(&microp_i2c_driver);
	if (ret)
		return ret;

	for (n = FIRST_MICROP_IRQ; n < MICROP_IRQ_END; n++) {
		set_irq_chip(n, &microp_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}
	return 0;
}

static void __exit microp_common_exit(void)
{
	i2c_del_driver(&microp_i2c_driver);
}

module_init(microp_common_init);
module_exit(microp_common_exit);

MODULE_AUTHOR("Eric Huang <Eric.SP_Huang@htc.com>");
MODULE_DESCRIPTION("Atmega MicroP driver");
MODULE_LICENSE("GPL");
