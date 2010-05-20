/* arch/arm/mach-msm/htc_headset_common.c
 *
 * Copyright (C) 2009 HTC, Inc.
 * Author: Arec Kao <Arec_Kao@htc.com>
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
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include <mach/htc_headset_common.h>

/* #define CONFIG_DEBUG_H2W */

#define H2WI(fmt, arg...) \
	printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#define H2WE(fmt, arg...) \
	printk(KERN_ERR "[H2W] %s " fmt "\r\n", __func__, ## arg)

#ifdef CONFIG_DEBUG_H2W
#define H2W_DBG(fmt, arg...) \
	printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#else
#define H2W_DBG(fmt, arg...) do {} while (0)
#endif

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(flag, _mode, _show, _store)

static struct h2w_info *hi;
static int h2w_path;

void button_pressed(int type)
{
	printk(KERN_INFO "[H2W] button_pressed %d\n", type);
	atomic_set(&hi->btn_state, type);
	input_report_key(hi->input, type, 1);
	input_sync(hi->input);
}

void button_released(int type)
{
	printk(KERN_INFO "[H2W] button_released %d\n", type);
	atomic_set(&hi->btn_state, 0);
	input_report_key(hi->input, type, 0);
	input_sync(hi->input);
}

static void enable_h2w_irq(void)
{
	unsigned long irq_flags;

	local_irq_save(irq_flags);
	enable_irq(hi->irq_btn);
	enable_irq(hi->irq);
	set_irq_wake(hi->irq, 1);
	set_irq_wake(hi->irq_btn, 1);
	local_irq_restore(irq_flags);
}

void disable_h2w_irq(void)
{
	unsigned long irq_flags;

	local_irq_save(irq_flags);
	disable_irq(hi->irq_btn);
	disable_irq(hi->irq);
	set_irq_wake(hi->irq, 0);
	set_irq_wake(hi->irq_btn, 0);
	local_irq_restore(irq_flags);
}

static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	if (!hi) {
		pr_err("%s: htc_headset_common not initialized yet\n", __func__);
		return ret;
	}

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("%s: failed to set parameter: %d\n", __func__, ret);
		return ret;
	}

	switch (h2w_path) {
	case H2W_GPIO:
		hi->configure(H2W_GPIO);
		enable_h2w_irq();
		break;
	case H2W_UART3:
		disable_h2w_irq();
		hi->configure(H2W_UART3);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&h2w_path, S_IWUSR | S_IRUGO);

/*
static int h2w_irq_enable(void *argu)
{
	int *enable = (int *) argu;
	if (*enable)
		enable_h2w_irq();
	else
		disable_h2w_irq();
	return 1;
}
*/

/*****************
 * H2W proctocol *
 *****************/
static inline void h2w_begin_command(void)
{
	mutex_lock(&hi->mutex_rc_lock);
	/* Disable H2W interrupt */
	set_irq_type(hi->irq_btn, IRQF_TRIGGER_HIGH);
	disable_irq(hi->irq);
	disable_irq(hi->irq_btn);

	/* Set H2W_CLK as output low */
	hi->set_clk(0);
	hi->set_clk_dir(1);
}

static inline void h2w_end_command(void)
{
	/* Set H2W_CLK as input */
	hi->set_clk_dir(0);

	/* Enable H2W interrupt */
	enable_irq(hi->irq);
	enable_irq(hi->irq_btn);
	set_irq_type(hi->irq_btn, IRQF_TRIGGER_RISING);
	mutex_unlock(&hi->mutex_rc_lock);
}

static inline void one_clock_write(unsigned short flag)
{
	if (flag)
		hi->set_dat(1);
	else
		hi->set_dat(0);

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(hi->speed);
	hi->set_clk(0);
}

static inline void one_clock_write_RWbit(unsigned short flag)
{
	if (flag)
		hi->set_dat(1);
	else
		hi->set_dat(0);

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(hi->speed);
	hi->set_clk(0);
	hi->set_dat_dir(0);
	udelay(hi->speed);
}

static inline void h2w_reset(void)
{
	/* Set H2W_DAT as output low */
	hi->set_dat(0);
	hi->set_dat_dir(1);

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(4 * hi->speed);
	hi->set_dat(1);
	udelay(hi->speed);
	hi->set_dat(0);
	udelay(hi->speed);
	hi->set_clk(0);
	udelay(hi->speed);
}

static inline void h2w_start(void)
{
	udelay(hi->speed);
	hi->set_clk(1);
	udelay(2 * hi->speed);
	hi->set_clk(0);
	udelay(hi->speed);
}

static inline int h2w_ack(void)
{
	int retry_times = 0;

ack_resend:
	if (retry_times == MAX_ACK_RESEND_TIMES)
		return -1;

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(2 * hi->speed);

	if (!hi->get_dat()) {
		retry_times++;
		hi->set_clk(0);
		udelay(hi->speed);
		goto ack_resend;
	}

	hi->set_clk(0);
	udelay(hi->speed);
	return 0;
}

static unsigned char h2w_readc(void)
{
	unsigned char h2w_read_data = 0x0;
	int index;

	for (index = 0; index < 8; index++) {
		hi->set_clk(0);
		udelay(hi->speed);
		hi->set_clk(1);
		udelay(hi->speed);
		if (hi->get_dat())
			h2w_read_data |= (1 << (7 - index));
	}
	hi->set_clk(0);
	udelay(hi->speed);

	return h2w_read_data;
}

static int h2w_readc_cmd(H2W_ADDR address)
{
	int ret = -1, retry_times = 0;
	unsigned char read_data;

read_resend:
	if (retry_times == MAX_HOST_RESEND_TIMES)
		goto err_read;

	h2w_reset();
	h2w_start();
	/* Write address */
	one_clock_write(address & 0x1000);
	one_clock_write(address & 0x0800);
	one_clock_write(address & 0x0400);
	one_clock_write(address & 0x0200);
	one_clock_write(address & 0x0100);
	one_clock_write(address & 0x0080);
	one_clock_write(address & 0x0040);
	one_clock_write(address & 0x0020);
	one_clock_write(address & 0x0010);
	one_clock_write(address & 0x0008);
	one_clock_write(address & 0x0004);
	one_clock_write(address & 0x0002);
	one_clock_write(address & 0x0001);
	one_clock_write_RWbit(1);
	if (h2w_ack() < 0) {
		H2W_DBG("Addr NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto read_resend;
	}

	read_data = h2w_readc();

	if (h2w_ack() < 0) {
		H2W_DBG("Data NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto read_resend;
	}
	ret = (int)read_data;

err_read:
	if (ret < 0)
		H2WE("NO ACK.\n");

	return ret;
}


static int h2w_writec_cmd(H2W_ADDR address, unsigned char data)
{
	int ret = -1;
	int retry_times = 0;

write_resend:
	if (retry_times == MAX_HOST_RESEND_TIMES)
		goto err_write;

	h2w_reset();
	h2w_start();

	/* Write address */
	one_clock_write(address & 0x1000);
	one_clock_write(address & 0x0800);
	one_clock_write(address & 0x0400);
	one_clock_write(address & 0x0200);
	one_clock_write(address & 0x0100);
	one_clock_write(address & 0x0080);
	one_clock_write(address & 0x0040);
	one_clock_write(address & 0x0020);
	one_clock_write(address & 0x0010);
	one_clock_write(address & 0x0008);
	one_clock_write(address & 0x0004);
	one_clock_write(address & 0x0002);
	one_clock_write(address & 0x0001);
	one_clock_write_RWbit(0);
	if (h2w_ack() < 0) {
		H2W_DBG("Addr NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto write_resend;
	}

	/* Write data */
	hi->set_dat_dir(1);
	one_clock_write(data & 0x0080);
	one_clock_write(data & 0x0040);
	one_clock_write(data & 0x0020);
	one_clock_write(data & 0x0010);
	one_clock_write(data & 0x0008);
	one_clock_write(data & 0x0004);
	one_clock_write(data & 0x0002);
	one_clock_write_RWbit(data & 0x0001);
	if (h2w_ack() < 0) {
		H2W_DBG("Data NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto write_resend;
	}
	ret = 0;

err_write:
	if (ret < 0)
		H2WE("NO ACK.\n");

	return ret;
}

int h2w_enable_phone_in(int enable)
{
	int ret = 0;

	h2w_begin_command();
	if (enable)
		ret = h2w_writec_cmd(H2W_ASCR0, H2W_ASCR_AUDIO_IN |
				     H2W_ASCR_ACT_EN | H2W_ASCR_PHONE_IN);
	else
		ret = h2w_writec_cmd(H2W_ASCR0, H2W_ASCR_AUDIO_IN |
				     H2W_ASCR_ACT_EN);

	udelay(10);
	h2w_end_command();

	return ret;
}

int h2w_enable_mute_light(int enable)
{
	int ret = 0;

	h2w_begin_command();
	if (enable)
		ret = h2w_writec_cmd(H2W_LEDCT0, H2W_LED_MTL);
	else
		ret = h2w_writec_cmd(H2W_LEDCT0, H2W_LED_OFF);

	udelay(10);
	h2w_end_command();

	return ret;
}

static int h2w_get_fnkey(void)
{
	int ret;
	h2w_begin_command();
	ret = h2w_readc_cmd(H2W_FNKEY_UPDOWN);
	h2w_end_command();
	return ret;
}

static int h2w_dev_init(H2W_INFO *ph2w_info)
{
	int ret = -1;
	unsigned char ascr0 = 0;
	int h2w_sys = 0, maxgpadd = 0, maxadd = 0, key = 0;

	hi->speed = H2W_50KHz;
	h2w_begin_command();

	/* read H2W_SYSTEM */
	h2w_sys = h2w_readc_cmd(H2W_SYSTEM);
	if (h2w_sys == -1) {
		H2WE("read H2W_SYSTEM(0x0000) failed.\n");
		goto err_plugin;
	}
	ph2w_info->ACC_CLASS = (h2w_sys & 0x03);
	ph2w_info->AUDIO_DEVICE  = (h2w_sys & 0x04) > 0 ? 1 : 0;
	ph2w_info->HW_REV = (h2w_sys & 0x18) >> 3;
	ph2w_info->SLEEP_PR  = (h2w_sys & 0x20) >> 5;
	ph2w_info->CLK_SP = (h2w_sys & 0xC0) >> 6;

	/* enter init mode */
	if (h2w_writec_cmd(H2W_ASCR0, H2W_ASCR_DEVICE_INI) < 0) {
		H2WE("write H2W_ASCR0(0x0002) failed.\n");
		goto err_plugin;
	}
	udelay(10);

	/* read H2W_MAX_GP_ADD */
	maxgpadd = h2w_readc_cmd(H2W_MAX_GP_ADD);
	if (maxgpadd == -1) {
		H2WE("write H2W_MAX_GP_ADD(0x0001) failed.\n");
		goto err_plugin;
	}
	ph2w_info->CLK_SP += (maxgpadd & 0x60) >> 3;
	ph2w_info->MAX_GP_ADD = (maxgpadd & 0x1F);

	/* read key group */
	if (ph2w_info->MAX_GP_ADD >= 1) {
		ph2w_info->KEY_MAXADD = h2w_readc_cmd(H2W_KEY_MAXADD);
		if (ph2w_info->KEY_MAXADD == -1)
			goto err_plugin;
		if (ph2w_info->KEY_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_ASCII_DOWN);
			if (key < 0)
				goto err_plugin;
			ph2w_info->ASCII_DOWN = (key == 0xFF) ? 1 : 0;
		}
		if (ph2w_info->KEY_MAXADD >= 2) {
			key = h2w_readc_cmd(H2W_ASCII_UP);
			if (key == -1)
				goto err_plugin;
			ph2w_info->ASCII_UP = (key == 0xFF) ? 1 : 0;
		}
		if (ph2w_info->KEY_MAXADD >= 3) {
			key = h2w_readc_cmd(H2W_FNKEY_UPDOWN);
			if (key == -1)
				goto err_plugin;
			ph2w_info->FNKEY_UPDOWN = (key == 0xFF) ? 1 : 0;
		}
		if (ph2w_info->KEY_MAXADD >= 4) {
			key = h2w_readc_cmd(H2W_KD_STATUS);
			if (key == -1)
				goto err_plugin;
			ph2w_info->KD_STATUS = (key == 0x01) ? 1 : 0;
		}
	}

	/* read led group */
	if (ph2w_info->MAX_GP_ADD >= 2) {
		ph2w_info->LED_MAXADD = h2w_readc_cmd(H2W_LED_MAXADD);
		if (ph2w_info->LED_MAXADD == -1)
			goto err_plugin;
		if (ph2w_info->LED_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_LEDCT0);
			if (key == -1)
				goto err_plugin;
			ph2w_info->LEDCT0 = (key == 0x02) ? 1 : 0;
		}
	}

	/* read group 3, 4, 5 */
	if (ph2w_info->MAX_GP_ADD >= 3) {
		maxadd = h2w_readc_cmd(H2W_CRDL_MAXADD);
		if (maxadd == -1)
			goto err_plugin;
	}
	if (ph2w_info->MAX_GP_ADD >= 4) {
		maxadd = h2w_readc_cmd(H2W_CARKIT_MAXADD);
		if (maxadd == -1)
			goto err_plugin;
	}
	if (ph2w_info->MAX_GP_ADD >= 5) {
		maxadd = h2w_readc_cmd(H2W_USBHOST_MAXADD);
		if (maxadd == -1)
			goto err_plugin;
	}

	/* read medical group */
	if (ph2w_info->MAX_GP_ADD >= 6) {
		ph2w_info->MED_MAXADD = h2w_readc_cmd(H2W_MED_MAXADD);
		if (ph2w_info->MED_MAXADD == -1)
			goto err_plugin;
		if (ph2w_info->MED_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_MED_CONTROL);
			if (key == -1)
				goto err_plugin;
		ph2w_info->DATA_EN = (key & 0x01);
		ph2w_info->AP_EN = (key & 0x02) >> 1;
		ph2w_info->AP_ID = (key & 0x1c) >> 2;
		}
		if (ph2w_info->MED_MAXADD >= 2) {
			key = h2w_readc_cmd(H2W_MED_IN_DATA);
			if (key == -1)
				goto err_plugin;
		}
	}

	if (ph2w_info->AUDIO_DEVICE)
		ascr0 = H2W_ASCR_AUDIO_IN | H2W_ASCR_ACT_EN;
	else
		ascr0 = H2W_ASCR_ACT_EN;

	if (h2w_writec_cmd(H2W_ASCR0, ascr0) < 0)
		goto err_plugin;
	udelay(10);

	ret = 0;

	/* adjust speed */
	if (ph2w_info->MAX_GP_ADD == 2) {
		/* Remote control */
		hi->speed = H2W_250KHz;
	} else if (ph2w_info->MAX_GP_ADD == 6) {
		if (ph2w_info->MED_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_MED_CONTROL);
			if (key == -1)
				goto err_plugin;
			ph2w_info->DATA_EN   = (key & 0x01);
			ph2w_info->AP_EN = (key & 0x02) >> 1;
			ph2w_info->AP_ID = (key & 0x1c) >> 2;
		}
	}

err_plugin:
	h2w_end_command();

	return ret;
}

int h2w_dev_detect(void)
{
	int ret = -1;
	int retry_times;

	for (retry_times = 5; retry_times; retry_times--) {
		/* Enable H2W Power */
		hi->h2w_power(1);
		msleep(100);
		memset(&hi->h2w_info, 0, sizeof(H2W_INFO));
		if (h2w_dev_init(&hi->h2w_info) < 0) {
			hi->h2w_power(0);
			msleep(100);
		} else if (hi->h2w_info.MAX_GP_ADD == 2) {
			ret = 0;
			break;
		} else {
			printk(KERN_INFO "h2w_detect: detect error(%d)\n"
				, hi->h2w_info.MAX_GP_ADD);
			hi->h2w_power(0);
			msleep(100);
		}
		printk(KERN_INFO "h2w_detect(%d)\n"
				, hi->h2w_info.MAX_GP_ADD);
	}
	H2W_DBG("h2w_detect:(%d)\n", retry_times);
	return ret;
}

int is_accessary_pluged_in(void)
{
	int type = 0;
	int clk1 = 0, dat1 = 0, clk2 = 0, dat2 = 0, clk3 = 0, dat3 = 0;

	/* Step1: save H2W_CLK and H2W_DAT */
	/* Delay 10ms for pin stable. */
	msleep(10);
	clk1 = gpio_get_value(hi->h2w_clk);
	dat1 = gpio_get_value(hi->h2w_data);

	/*
	 * Step2: set GPIO_CABLE_IN1 as output high and GPIO_CABLE_IN2 as
	 * input
	 */
	gpio_direction_output(hi->cable_in1, 1);
	gpio_direction_input(hi->cable_in2);
	/* Delay 10ms for pin stable. */
	msleep(10);
	/* Step 3: save H2W_CLK and H2W_DAT */
	clk2 = gpio_get_value(hi->h2w_clk);
	dat2 = gpio_get_value(hi->h2w_data);

	/*
	 * Step 4: set GPIO_CABLE_IN1 as input and GPIO_CABLE_IN2 as output
	 * high
	 */
	gpio_direction_input(hi->cable_in1);
	gpio_direction_output(hi->cable_in2, 1);
	/* Delay 10ms for pin stable. */
	msleep(10);
	/* Step 5: save H2W_CLK and H2W_DAT */
	clk3 = gpio_get_value(hi->h2w_clk);
	dat3 = gpio_get_value(hi->h2w_data);

	/* Step 6: set both GPIO_CABLE_IN1 and GPIO_CABLE_IN2 as input */
	gpio_direction_input(hi->cable_in1);
	gpio_direction_input(hi->cable_in2);

	H2WI("(%d,%d) (%d,%d) (%d,%d)",
		clk1, dat1, clk2, dat2, clk3, dat3);

	if ((clk1 == 0) && (dat1 == 1) &&
	    (clk2 == 0) && (dat2 == 1) &&
	    (clk3 == 0) && (dat3 == 1))
		type = HTC_HEADSET;
	else if ((clk1 == 0) && (dat1 == 0) &&
		 (clk2 == 0) && (dat2 == 0) &&
		 (clk3 == 0) &&  (dat3 == 0))
		type = NORMAL_HEARPHONE;
	else if ((clk1 == 0) && (dat1 == 0) &&
		 (clk2 == 1) && (dat2 == 0) &&
		 (clk3 == 0) && (dat3 == 1))
		type = H2W_DEVICE;
	else if ((clk1 == 0) && (dat1 == 0) &&
		 (clk2 == 1) && (dat2 == 1) &&
		 (clk3 == 1) && (dat3 == 1))
		type = USB_CRADLE;
	else if ((clk1 == 0) && (dat1 == 1) &&
		 (clk2 == 1) && (dat2 == 1) &&
		 (clk3 == 0) && (dat3 == 1))
		type = UART_DEBUG;
	else if ((clk1 == 1) && (dat1 == 0) &&
		 (clk2 == 1) && (dat2 == 0) &&
		 (clk3 == 1) && (dat3 == 0))
		type = H2W_TVOUT;
	else
		type = NO_DEVICE;

	return type;
}

void headset_button_event(int is_press, int type)
{
	if (!is_press) {
		if (hi->ignore_btn)
			hi->ignore_btn = 0;
		else
			button_released(type);
	} else {
		if (!hi->ignore_btn && !atomic_read(&hi->btn_state))
			button_pressed(type);
	}
}

void button_h2w_do_work(struct work_struct *w)
{
	int key, press, keyname, h2w_key = 1;

	H2W_DBG("");

	if (switch_get_state(&hi->sdev) & BIT_HEADSET) {
		switch (hi->h2w_dev_type) {
		case HTC_HEADSET:
			if (!gpio_get_value(hi->h2w_data))
				headset_button_event(1, KEY_MEDIA); /* press */
			else
				headset_button_event(0, KEY_MEDIA);
			break;
		case H2W_DEVICE:
			if ((hi->get_dat() == 1) && (hi->get_clk() == 1)) {
				/* Don't do anything because H2W pull out. */
				H2WE("Remote Control pull out.\n");
			} else {
				key = h2w_get_fnkey();
				press = (key > 0x7F) ? 0 : 1;
				keyname = key & 0x7F;
				 /* H2WI("key = %d, press = %d,
					 keyname = %d \n",
					 key, press, keyname); */
				switch (keyname) {
				case H2W_KEY_PLAY:
					H2WI("H2W_KEY_PLAY");
					key = KEY_PLAYPAUSE;
					break;
				case H2W_KEY_FORWARD:
					H2WI("H2W_KEY_FORWARD");
					key = KEY_NEXTSONG;
					break;
				case H2W_KEY_BACKWARD:
					H2WI("H2W_KEY_BACKWARD");
					key = KEY_PREVIOUSSONG;
					break;
				case H2W_KEY_VOLUP:
					H2WI("H2W_KEY_VOLUP");
					key = KEY_VOLUMEUP;
					break;
				case H2W_KEY_VOLDOWN:
					H2WI("H2W_KEY_VOLDOWN");
					key = KEY_VOLUMEDOWN;
					break;
				case H2W_KEY_PICKUP:
					H2WI("H2W_KEY_PICKUP");
					key = KEY_SEND;
					break;
				case H2W_KEY_HANGUP:
					H2WI("H2W_KEY_HANGUP");
					key = KEY_END;
					break;
				case H2W_KEY_MUTE:
					H2WI("H2W_KEY_MUTE");
					key = KEY_MUTE;
					break;
				case H2W_KEY_HOLD:
					H2WI("H2W_KEY_HOLD");
					break;
				default:
				H2WI("default");
					h2w_key = 0;
				}
				if (h2w_key) {
					if (press)
						H2WI("Press\n");
					else
						H2WI("Release\n");
					wake_lock_timeout
					(&hi->headset_wake_lock, 1.5*HZ);
					input_report_key(hi->input, key, press);
				}
			}
			break;
		} /* end switch */
	}
}

void remove_headset(void)
{
	int state;

	H2W_DBG("");

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev) &
			~(BIT_HEADSET | BIT_HEADSET_NO_MIC | BIT_TV_OUT);
	if (hi->ext_35mm_status == HTC_35MM_MIC)
		state |= BIT_HEADSET;
	else if (hi->ext_35mm_status == HTC_35MM_NO_MIC)
		state |= BIT_HEADSET_NO_MIC;
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);

	hi->configure(H2W_GPIO);

	/* Disable button */
	switch (hi->h2w_dev_type) {
	case HTC_HEADSET:
		if (atomic_read(&hi->btn_state))
			button_released(atomic_read(&hi->btn_state));
		printk(KERN_INFO "Remove HTC_HEADSET\n");
		break;
	case NORMAL_HEARPHONE:
		hi->remove_11pin_35mm();
		printk(KERN_INFO "Remove NORMAL_HEARPHONE\n");
		break;
	case H2W_DEVICE:
		hi->h2w_power(0);
		/*have H2W device*/
		if (!gpio_get_value(hi->cable_in1) ||
			!gpio_get_value(hi->cable_in2))
			set_irq_type(hi->irq_btn, IRQF_TRIGGER_HIGH);
		else /*no H2W device*/
			set_irq_type(hi->irq_btn, IRQF_TRIGGER_LOW);
		hi->set_clk_dir(0);
		hi->set_dat_dir(0);
		printk(KERN_INFO "Remove H2W_DEVICE\n");
		break;
	case USB_CRADLE:
		printk(KERN_INFO "Remove USB_CRADLE\n");
		break;
	case H2W_TVOUT:
		printk(KERN_INFO "Remove H2W_TVOUT\n");
		break;
	default:
		printk(KERN_INFO "Unknown Accessory\n");
	}

	hi->h2w_dev_type = 0;
}

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
extern void msm_serial_debug_enable(int);
#endif
void insert_headset(int type)
{
	unsigned long irq_flags;
	int state;

	H2W_DBG("");

	if (hi->h2w_dev_type)
		remove_headset();

	hi->h2w_dev_type = type;
	hi->ignore_btn = 0;
	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	mutex_unlock(&hi->mutex_lock);
	state &= ~(BIT_HEADSET_NO_MIC | BIT_HEADSET | BIT_TV_OUT);

	switch (type) {
	case HTC_HEADSET:
		printk(KERN_INFO "Insert HTC_HEADSET\n");
		state |= BIT_HEADSET;
		hi->ignore_btn = !gpio_get_value(hi->h2w_data);
		break;
	case NORMAL_HEARPHONE:
		hi->insert_11pin_35mm(&state);
		printk(KERN_INFO "Insert NORMAL_HEARPHONE\n");
		break;
	case H2W_DEVICE:
		printk(KERN_INFO "Insert H2W_DEVICE\n");
		if (!hi->set_dat) {
			printk(KERN_INFO "Don't support H2W_DEVICE\n");
			hi->h2w_dev_type = 0;
			return;
		}
		if (h2w_dev_detect() < 0) {
			printk(KERN_INFO "H2W_DEVICE -- Non detected\n");
			hi->set_clk_dir(0);
			hi->set_dat_dir(0);
			hi->h2w_dev_type = 0;
		} else {
			printk(KERN_INFO "H2W_DEVICE -- detected\n");
			local_irq_save(irq_flags);
			set_irq_type(hi->irq_btn, IRQF_TRIGGER_RISING);
			local_irq_restore(irq_flags);
			state |= BIT_HEADSET;
			if (hi->rc_flag & H2W_MuteLed)
				h2w_enable_mute_light(1);
			if (hi->rc_flag & H2W_PhoneIn)
				h2w_enable_phone_in(1);
		}
		break;
	case USB_CRADLE:
		printk(KERN_INFO "Insert USB_CRADLE\n");
		state |= BIT_HEADSET_NO_MIC;
		break;
	case UART_DEBUG:
		printk(KERN_INFO "Insert UART_DEBUG\n");
		disable_h2w_irq();
		hi->configure(hi->debug_uart);
		return;
	case H2W_TVOUT:
		printk(KERN_INFO "Insert H2W_TVOUT\n");
		state |= BIT_TV_OUT;
		if (hi->ext_35mm_status == HTC_35MM_MIC)
			state |= BIT_HEADSET;
		else
			state |= BIT_HEADSET_NO_MIC;
		break;
	default:
		printk(KERN_INFO "Unknown Accessory\n");
		return;
	}
	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	hi->configure(H2W_GPIO);

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
	msm_serial_debug_enable(false);
#endif

	/* On some non-standard headset adapters (usually those without a
	 * button) the btn line is pulled down at the same time as the detect
	 * line. We can check here by sampling the button line, if it is
	 * low then it is probably a bad adapter so ignore the button.
	 * If the button is released then we stop ignoring the button, so that
	 * the user can recover from the situation where a headset is plugged
	 * in with button held down.
	 */

	 /* hi->debounce_time = ktime_set(0, 200000000);   20 ms */
}

void detect_h2w_do_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int type;

	H2W_DBG("");

	if (hi->h2w_dev_type) {
		/* Remove H2W device */
		remove_headset();
		return;
	}

	/* Switch CPLD to GPIO to do detection */
	hi->configure(H2W_GPIO);

	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hi->irq);
	disable_irq(hi->irq_btn);
	local_irq_restore(irq_flags);

	/* Something plugged in, lets make sure its a headset */
	type = is_accessary_pluged_in();

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hi->irq);
	enable_irq(hi->irq_btn);
	local_irq_restore(irq_flags);

	if (hi->is_ext_insert && type != H2W_TVOUT)
		return;

	insert_headset(type);
}

int switch_send_event(unsigned int bit, int on)
{
	unsigned long state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(bit);

	if (on)
		state |= bit;

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int h2w_debug_set(void *data, u64 val)
{
	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, (int)val);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static int h2w_debug_get(void *data, u64 *val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(h2w_debug_fops, h2w_debug_get, h2w_debug_set, "%llu\n");
static int __init h2w_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("h2w", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("state", 0644, dent, NULL, &h2w_debug_fops);

	return 0;
}

device_initcall(h2w_debug_init);
#endif

static ssize_t tty_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	mutex_lock(&hi->mutex_lock);
	s += sprintf(s, "%d\n", hi->tty_enable_flag);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t tty_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_TTY_FULL | BIT_TTY_VCO | BIT_TTY_HCO);

	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 1;

		switch_set_state(&hi->sdev, state | BIT_TTY_FULL);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY FULL\n");
		return count;
	}
	if (count == (strlen("vco_enable") + 1) &&
	   strncmp(buf, "vco_enable", strlen("vco_enable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 2;

		switch_set_state(&hi->sdev, state | BIT_TTY_VCO);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY VCO\n");
		return count;
	}
	if (count == (strlen("hco_enable") + 1) &&
	   strncmp(buf, "hco_enable", strlen("hco_enable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 3;

		switch_set_state(&hi->sdev, state | BIT_TTY_HCO);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY HCO\n");
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 0;

		switch_set_state(&hi->sdev, state);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Disable TTY\n");
		return count;
	}
	printk(KERN_ERR "tty_enable_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(tty, 0666, tty_flag_show, tty_flag_store);

static ssize_t fm_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_FM_HEADSET | BIT_FM_SPEAKER);

	if (count == (strlen("fm_headset") + 1) &&
	   strncmp(buf, "fm_headset", strlen("fm_headset")) == 0) {
		hi->fm_flag = 1;
		state |= BIT_FM_HEADSET;
		printk(KERN_INFO "Enable FM HEADSET\n");
	} else if (count == (strlen("fm_speaker") + 1) &&
	   strncmp(buf, "fm_speaker", strlen("fm_speaker")) == 0) {
		hi->fm_flag = 2;
		state |= BIT_FM_SPEAKER;
		printk(KERN_INFO "Enable FM SPEAKER\n");
	} else if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->fm_flag = 0 ;
		printk(KERN_INFO "Disable FM\n");
	} else {
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_ERR "fm_enable_flag_store: invalid argument\n");
		return -EINVAL;
	}
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return count;
}

static ssize_t fm_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	mutex_lock(&hi->mutex_lock);
	if (hi->fm_flag == 0)
		show_str = "disable";
	if (hi->fm_flag == 1)
		show_str = "fm_headset";
	if (hi->fm_flag == 2)
		show_str = "fm_speaker";

	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}
static DEVICE_ACCESSORY_ATTR(fm, 0666, fm_flag_show, fm_flag_store);

static ssize_t mic_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	char *s = buf;
	char *show_str;
	mutex_lock(&hi->mutex_lock);
	if (hi->mic_switch_flag == 0)
		show_str = "11pin";
	if (hi->mic_switch_flag == 1)
		show_str = "35mm";
	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t mic_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == (strlen("35mm") + 1) &&
	   strncmp(buf, "35mm", strlen("35mm")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->mic_switch_flag = 1;

		if (hi->ext_mic_sel) {
			gpio_direction_output(hi->ext_mic_sel, 1);
			printk(KERN_INFO "ext_mic_sel to 1\n");
		}

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable 3.5mm MIC\n");
		return count;
	}
	if (count == (strlen("11pin") + 1) &&
	   strncmp(buf, "11pin", strlen("11pin")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->mic_switch_flag = 0;

		if (hi->ext_mic_sel) {
			gpio_direction_output(hi->ext_mic_sel, 0);
			printk(KERN_INFO "ext_mic_sel to 0\n");
		}
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable 11pin MIC\n");
		return count;
	}
	printk(KERN_ERR "mic_switch_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(mic, 0666, mic_flag_show, mic_flag_store);

static ssize_t mute_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	if ((hi->rc_flag & H2W_MuteLed) == 0)
		show_str = "unmute";
	if ((hi->rc_flag & H2W_MuteLed) == H2W_MuteLed)
		show_str = "mute";
	s += sprintf(s, "%s\n", show_str);
	return (s - buf);
}
static ssize_t mute_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == (strlen("unmute") + 1) &&
	   strncmp(buf, "unmute", strlen("unmute")) == 0) {
		hi->rc_flag = hi->rc_flag & (!H2W_MuteLed);
		h2w_enable_mute_light(0);
		printk(KERN_INFO "Turn off mute LED\n");
		return count;
	}
	if (count == (strlen("mute") + 1) &&
	   strncmp(buf, "mute", strlen("mute")) == 0) {
		hi->rc_flag = hi->rc_flag | H2W_MuteLed;
		h2w_enable_mute_light(1);
		printk(KERN_INFO "Turn on mute LED\n");
		return count;
	}
	printk(KERN_ERR "mute_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(mute, 0666, mute_flag_show, mute_flag_store);

static ssize_t phonein_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	if ((hi->rc_flag & H2W_PhoneIn) == 0)
		show_str = "phoneoff";
	if ((hi->rc_flag & H2W_PhoneIn) == H2W_PhoneIn)
		show_str = "phonein";
	s += sprintf(s, "%s\n", show_str);
	return (s - buf);
}

static ssize_t phonein_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == (strlen("phoneoff") + 1) &&
	   strncmp(buf, "phoneoff", strlen("phoneoff")) == 0) {
		if (hi->h2w_dev_type == H2W_DEVICE) {
			hi->rc_flag = hi->rc_flag & (!H2W_PhoneIn);
			h2w_enable_phone_in(0);
			printk(KERN_INFO "The phone call is OFF\n");
		}
		return count;
	}
	if (count == (strlen("phonein") + 1) &&
	   strncmp(buf, "phonein", strlen("phonein")) == 0) {
		if (hi->h2w_dev_type == H2W_DEVICE) {
			hi->rc_flag = hi->rc_flag | H2W_PhoneIn;
			h2w_enable_phone_in(1);
			printk(KERN_INFO "The phone call is ON\n");
		}
		return count;
	}
	printk(KERN_ERR "phonein_flag_store: invalid argument\n");
	return -EINVAL;
}

static DEVICE_ACCESSORY_ATTR(phonein, 0666, \
			     phonein_flag_show, phonein_flag_store);

int register_common_headset(struct h2w_info *h2w, int create_attr)
{

	int ret;
	hi = h2w;

	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		goto err_create_class;
	}

	hi->tty_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		goto err_create_tty_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret)
		goto err_create_tty_device_file;

	hi->fm_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		goto err_create_fm_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret)
		goto err_create_fm_device_file;

	if (create_attr) {
		hi->mic_dev = device_create(hi->htc_accessory_class,
					NULL, 0, "%s", "mic");
		if (unlikely(IS_ERR(hi->mic_dev))) {
			ret = PTR_ERR(hi->mic_dev);
			hi->mic_dev = NULL;
			goto err_create_mic_device;
		}
		/* register the attributes */
		ret = device_create_file(hi->mic_dev, &dev_attr_mic);
		if (ret)
			goto err_create_mic_device_file;
		hi->mute_dev = device_create(hi->htc_accessory_class,
					NULL, 0, "%s", "mute");
		if (unlikely(IS_ERR(hi->mute_dev))) {
			ret = PTR_ERR(hi->mute_dev);
			hi->mute_dev = NULL;
			goto err_create_mute_device;
		}
		/* register the attributes */
		ret = device_create_file(hi->mute_dev, &dev_attr_mute);
		if (ret)
			goto err_create_mute_device_file;
		hi->phonein_dev = device_create(hi->htc_accessory_class,
						NULL, 0, "%s", "phonein");
		if (unlikely(IS_ERR(hi->phonein_dev))) {
			ret = PTR_ERR(hi->phonein_dev);
			hi->phonein_dev = NULL;
			goto err_create_phonein_device;
		}
		/* register the attributes */
		ret = device_create_file(hi->phonein_dev, &dev_attr_phonein);
		if (ret)
			goto err_create_phonein_device_file;
	}
	return 0;

err_create_phonein_device_file:
	device_unregister(hi->phonein_dev);
err_create_phonein_device:
	device_remove_file(hi->mute_dev, &dev_attr_mute);
err_create_mute_device_file:
	device_unregister(hi->mute_dev);
err_create_mute_device:
	device_remove_file(hi->mic_dev, &dev_attr_mic);
err_create_mic_device_file:
	device_unregister(hi->mic_dev);
err_create_mic_device:
	device_remove_file(hi->fm_dev, &dev_attr_fm);
err_create_fm_device_file:
	device_unregister(hi->fm_dev);
err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);
err_create_tty_device_file:
	device_unregister(hi->tty_dev);
err_create_tty_device:
	class_destroy(hi->htc_accessory_class);
err_create_class:

	return ret;
}

void unregister_common_headset(struct h2w_info *h2w)
{
	hi = h2w;
	device_remove_file(hi->phonein_dev, &dev_attr_phonein);
	device_unregister(hi->phonein_dev);
	device_remove_file(hi->mute_dev, &dev_attr_mute);
	device_unregister(hi->mute_dev);
	device_remove_file(hi->mic_dev, &dev_attr_mic);
	device_unregister(hi->mic_dev);
	device_remove_file(hi->tty_dev, &dev_attr_tty);
	device_unregister(hi->tty_dev);
	device_remove_file(hi->fm_dev, &dev_attr_fm);
	device_unregister(hi->fm_dev);
	class_destroy(hi->htc_accessory_class);
}

MODULE_AUTHOR("Arec Kao <Arec_Kao@htc.com>");
MODULE_DESCRIPTION("HTC 2 Wire driver");
MODULE_LICENSE("GPL");
