/* arch/arm/mach-msm/htc_35mm_remote.c
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
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include <mach/atmega_microp.h>
#include <mach/htc_headset_common.h>

#ifdef CONFIG_HTC_AUDIOJACK
#include <mach/audio_jack.h>
#endif

/* #define CONFIG_DEBUG_H2W */

/*Delay 200ms when 11pin device plug in*/
#define H2W_DETECT_DELAY	msecs_to_jiffies(200)
#define BUTTON_H2W_DELAY	msecs_to_jiffies(10)
#define H2W_NO_DELAY		msecs_to_jiffies(0)

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

static struct workqueue_struct *detect_wq;

static DECLARE_DELAYED_WORK(detect_h2w_work, detect_h2w_do_work);

static void insert_35mm_do_work(struct work_struct *work);
static DECLARE_WORK(insert_35mm_work, insert_35mm_do_work);
static void remove_35mm_do_work(struct work_struct *work);
static DECLARE_WORK(remove_35mm_work, remove_35mm_do_work);

static struct workqueue_struct *button_wq;

static void button_35mm_do_work(struct work_struct *w);
static DECLARE_WORK(button_35mm_work, button_35mm_do_work);
void button_h2w_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(button_h2w_work, button_h2w_do_work);

static struct h2w_info *hi;

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

static void insert_11pin_35mm(int *state)
{
	/* support 3.5mm earphone with mic */
	printk(KERN_INFO "11pin_3.5mm_headset plug in\n");

	/* Turn On Mic Bias */
	if (!hi->mic_bias_state) {
		turn_mic_bias_on(1);
		hi->mic_bias_state = 1;
		/* Wait pin be stable */
		msleep(300);
	}

	/* Detect headset with or without microphone */
	if (!microp_notify_mic_value()) {
		/* without microphone */
		*state |= BIT_HEADSET_NO_MIC;
		hi->h2w_35mm_status = HTC_35MM_NO_MIC;
		printk(KERN_INFO "11pin_3.5mm without microphone\n");
	} else { /* with microphone */
		*state |= BIT_HEADSET;
		hi->h2w_35mm_status = HTC_35MM_MIC;
		printk(KERN_INFO "11pin_3.5mm with microphone\n");
	}

	if (hi->key_int_shutdown_gpio)
		gpio_set_value(hi->key_int_shutdown_gpio, 1);
}

static void remove_11pin_35mm(void)
{
	if (hi->mic_bias_state) {
		turn_mic_bias_on(0);
		hi->mic_bias_state = 0;
		msleep(300);
	}

	microp_notify_unplug_mic();
	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->h2w_35mm_status = HTC_35MM_UNPLUG;
	if (hi->key_int_shutdown_gpio)
		gpio_set_value(hi->key_int_shutdown_gpio, 0);
	printk(KERN_INFO "remove 11pin 3.5mm headset\n");
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;
	int state = BIT_HEADSET | BIT_HEADSET_NO_MIC;
	H2W_DBG("");
	set_irq_type(hi->irq_btn, IRQF_TRIGGER_LOW);
	do {
		value1 = gpio_get_value(hi->cable_in1);
		set_irq_type(hi->irq, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->cable_in1);
	} while (value1 != value2 && retry_limit-- > 0);

	H2WI("value2 = %d (%d retries), device=%d", value2,
		(10-retry_limit), switch_get_state(&hi->sdev));

	if ((hi->h2w_dev_type == 0) ^ value2) {
		if ((switch_get_state(&hi->sdev) & state) &&
		    !hi->is_ext_insert)
			hi->ignore_btn = 1;
		if (hi->is_wake_lock_ready)
			wake_lock_timeout(&hi->headset_wake_lock, 2.5*HZ);
		queue_delayed_work(detect_wq,
			&detect_h2w_work, H2W_DETECT_DELAY);
	}
	return IRQ_HANDLED;
}

static void button_35mm_do_work(struct work_struct *w)
{
	int key;

	if (!hi->is_ext_insert && !hi->h2w_35mm_status) {
		H2WI("3.5mm headset is plugged out, skip report key event");
		return;
	}

#ifdef CONFIG_HTC_AUDIOJACK
	if (!is_audio_jack_pin_stable()) {
		H2WI(" The HPIN tremble, skip the button event");
		return;
	}
#endif

	if (hi->key_level_flag) {
		switch (hi->key_level_flag) {
		case 1:
			H2WI("3.5mm RC: Play Pressed");
			key = KEY_MEDIA;
			break;
		case 2:
			H2WI("3.5mm RC: BACKWARD Pressed");
			key = KEY_PREVIOUSSONG;
			break;
		case 3:
			H2WI("3.5mm RC: FORWARD Pressed");
			key = KEY_NEXTSONG;
			break;
		default:
			H2WI("3.5mm RC: WRONG Button Pressed");
			return;
		}
		headset_button_event(1, key);
	} else { /* key release */
		if (atomic_read(&hi->btn_state))
			headset_button_event(0, atomic_read(&hi->btn_state));
		else
			H2WI("3.5mm RC: WRONG Button Release");
	}
	wake_lock_timeout(&hi->headset_wake_lock, 1.5*HZ);
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	H2W_DBG("");
	do {
		value1 = gpio_get_value(hi->h2w_data);
		if (hi->h2w_dev_type != H2W_DEVICE)
			set_irq_type(hi->irq_btn, value1 ?
				     IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->h2w_data);
	} while (value1 != value2 && retry_limit-- > 0);

	H2W_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	if (!hi->h2w_dev_type || hi->h2w_dev_type == H2W_TVOUT)
		queue_delayed_work(detect_wq, &detect_h2w_work,
							H2W_DETECT_DELAY);
	else
		queue_delayed_work(button_wq, &button_h2w_work,
							BUTTON_H2W_DELAY);
	return IRQ_HANDLED;
}

static void remove_35mm_do_work(struct work_struct *work)
{
	int state;

	wake_lock_timeout(&hi->headset_wake_lock, 2.5*HZ);

	H2W_DBG("");
	/*To solve the insert, remove, insert headset problem*/
	if (time_before_eq(jiffies, hi->insert_jiffies))
		msleep(800);
	if (hi->is_ext_insert) {
		H2WI("Skip 3.5mm headset plug out!!!");
		return;
	}

	printk(KERN_INFO "3.5mm_headset plug out\n");
	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	mutex_unlock(&hi->mutex_lock);

	if (hi->mic_bias_state) {
		turn_mic_bias_on(0);
		hi->mic_bias_state = 0;
	}

	microp_notify_unplug_mic();

	mutex_lock(&hi->mutex_lock);
	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->ext_35mm_status = HTC_35MM_UNPLUG;

	if (hi->key_int_shutdown_gpio)
		gpio_set_value(hi->key_int_shutdown_gpio, 0);

	if (hi->ext_mic_sel)
		gpio_direction_output(hi->ext_mic_sel, 0);

	if (hi->h2w_dev_type == H2W_TVOUT) {
		state &= ~(BIT_HEADSET | BIT_35MM_HEADSET);
		state |= BIT_HEADSET_NO_MIC;
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);

	} else if (hi->cable_in1 && !gpio_get_value(hi->cable_in1)) {
		state &= ~BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
		queue_delayed_work(detect_wq, &detect_h2w_work, H2W_NO_DELAY);
	} else {
		state &= ~(BIT_HEADSET | BIT_HEADSET_NO_MIC |
			BIT_35MM_HEADSET);
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
	}
}

static void insert_35mm_do_work(struct work_struct *work)
{
	int state;

	H2W_DBG("");
	hi->insert_jiffies = jiffies + 1*HZ;

	wake_lock_timeout(&hi->headset_wake_lock, 1.5*HZ);
	if (hi->h2w_dev_type && hi->is_ext_insert &&
		hi->h2w_dev_type != H2W_TVOUT)
		remove_headset();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	mutex_unlock(&hi->mutex_lock);

	if (hi->is_ext_insert) {
		printk(KERN_INFO "3.5mm_headset plug in\n");
		hi->ignore_btn = 0;
		state &= ~(BIT_HEADSET | BIT_HEADSET_NO_MIC);
		if (hi->ext_mic_sel)
			gpio_direction_output(hi->ext_mic_sel, 1);

		/* Turn On Mic Bias */
		if (!hi->mic_bias_state) {
			turn_mic_bias_on(1);
			hi->mic_bias_state = 1;
			/* Wait for pin stable */
			msleep(300);
		}

		/* Detect headset with or without microphone */

		if (!microp_notify_mic_value()) {
			/* without microphone */
			state |= BIT_HEADSET_NO_MIC;
			printk(KERN_INFO "3.5mm_headset without microphone\n");
		} else {
			/* with microphone */
			state |= BIT_HEADSET;
			printk(KERN_INFO "3.5mm_headset with microphone\n");
		}

		if (hi->key_int_shutdown_gpio)
			gpio_set_value(hi->key_int_shutdown_gpio, 1);

		state |= BIT_35MM_HEADSET;

		mutex_lock(&hi->mutex_lock);
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
		if (state & BIT_HEADSET_NO_MIC)
			hi->ext_35mm_status = HTC_35MM_NO_MIC;
		else
			hi->ext_35mm_status = HTC_35MM_MIC;

	}
}

int htc_35mm_remote_notify_insert_ext_headset(int insert)
{
	if (hi) {
		mutex_lock(&hi->mutex_lock);
		hi->is_ext_insert = insert;
		mutex_unlock(&hi->mutex_lock);

		H2WI(" %d", hi->is_ext_insert);
		if (!hi->is_ext_insert)
			queue_work(detect_wq, &remove_35mm_work);
		else
			queue_work(detect_wq, &insert_35mm_work);
	}
	return 1;
}

int htc_35mm_remote_notify_microp_ready(void)
{
	if (hi) {
		if (hi->is_ext_insert)
			queue_work(detect_wq, &insert_35mm_work);
		if (hi->h2w_35mm_status)
			insert_headset(NORMAL_HEARPHONE);
	}
	return 1;
}

int htc_35mm_remote_notify_button_status(int key_level)
{
	hi->key_level_flag = key_level;

	if (hi->ext_35mm_status == HTC_35MM_NO_MIC ||
		hi->h2w_35mm_status == HTC_35MM_NO_MIC) {

		printk(KERN_INFO "To re-detect mic!!!\n");
		queue_work(detect_wq, &insert_35mm_work);
	} else
		queue_work(button_wq, &button_35mm_work);

	return 1;
}

static int htc_35mm_probe(struct platform_device *pdev)
{
	int ret;

	struct h2w_platform_data *pdata = pdev->dev.platform_data;
	printk(KERN_INFO "H2W: htc_35mm_remote Registering H2W (headset) Driver\n");

	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	hi->ext_35mm_status = 0;
	hi->h2w_35mm_status = 0;
	hi->is_ext_insert = 0;
	hi->mic_bias_state = 0;
	hi->key_level_flag = -1;

	atomic_set(&hi->btn_state, 0);
	hi->ignore_btn = 0;
	hi->h2w_dev_type = 0;
	hi->is_wake_lock_ready = 0;
	hi->tty_enable_flag = 0;
	hi->fm_flag = 0;
	hi->mic_switch_flag = 1;
	hi->rc_flag = 0;
	hi->cable_in1 = pdata->cable_in1;
	hi->cable_in2 = pdata->cable_in2;
	hi->h2w_clk = pdata->h2w_clk;
	hi->h2w_data = pdata->h2w_data;
	hi->debug_uart = pdata->debug_uart;
	hi->configure = pdata->config;
	hi->set_dat = pdata->set_dat;
	hi->set_clk = pdata->set_clk;
	hi->set_dat_dir	= pdata->set_dat_dir;
	hi->set_clk_dir	= pdata->set_clk_dir;
	hi->get_dat = pdata->get_dat;
	hi->get_clk = pdata->get_clk;
	hi->speed = H2W_50KHz;
	hi->h2w_power = pdata->h2w_power;
	hi->ext_mic_sel = pdata->ext_mic_sel;
	hi->key_int_shutdown_gpio = pdata->key_int_shutdown_gpio;

	hi->insert_11pin_35mm = insert_11pin_35mm;
	hi->remove_11pin_35mm = remove_11pin_35mm;

	if (hi->h2w_power)
		hi->h2w_power(0);

	mutex_init(&hi->mutex_lock);
	mutex_init(&hi->mutex_rc_lock);

	wake_lock_init(&hi->headset_wake_lock, WAKE_LOCK_SUSPEND, "headset");

	hi->sdev.name = "h2w";
	hi->sdev.print_name = h2w_print_name;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	detect_wq = create_workqueue("detection");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		goto err_create_detect_work_queue;
	}
	button_wq = create_workqueue("button");
	if (button_wq  == NULL) {
			ret = -ENOMEM;
			goto err_create_button_work_queue;
	}

	if (hi->cable_in1 && hi->cable_in2) {
		ret = gpio_request(hi->cable_in1, "h2w_detect");
		if (ret < 0)
			goto err_request_detect_gpio;

		ret = gpio_request(hi->h2w_data, "h2w_button");
		if (ret < 0)
			goto err_request_button_gpio;

		ret = gpio_direction_input(hi->h2w_data);
		if (ret < 0)
			goto err_set_button_gpio;

		ret = gpio_direction_input(hi->cable_in1);
		if (ret < 0)
			goto err_set_detect_gpio;

		ret = gpio_direction_input(hi->cable_in2);
		if (ret < 0)
			goto err_set_cablein2_gpio;

		hi->irq = gpio_to_irq(hi->cable_in1);
		if (hi->irq < 0) {
			ret = hi->irq;
			goto err_get_h2w_detect_irq_num_failed;
		}

		hi->irq_btn = gpio_to_irq(hi->h2w_data);
		if (hi->irq_btn < 0) {
			ret = hi->irq_btn;
			goto err_get_button_irq_num_failed;
		}
	/* Set CPLD MUX to H2W <-> CPLD GPIO */
		hi->configure(H2W_GPIO);
	}
	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "h2w headset";
	set_bit(EV_SYN, hi->input->evbit);
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_MEDIA, hi->input->keybit);
	set_bit(KEY_NEXTSONG, hi->input->keybit);
	set_bit(KEY_PLAYPAUSE, hi->input->keybit);
	set_bit(KEY_PREVIOUSSONG, hi->input->keybit);
	set_bit(KEY_MUTE, hi->input->keybit);
	set_bit(KEY_VOLUMEUP, hi->input->keybit);
	set_bit(KEY_VOLUMEDOWN, hi->input->keybit);
	set_bit(KEY_END, hi->input->keybit);
	set_bit(KEY_SEND, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
	goto err_register_input_dev;

	ret = register_common_headset(hi,
		(hi->cable_in1 && hi->cable_in2) ? 1 : 0);
	if (ret)
		goto err_register_common_headset;

	if (hi->cable_in1 && hi->cable_in2) {
		ret = request_irq(hi->irq, detect_irq_handler,
				IRQF_TRIGGER_LOW, "h2w_detect", NULL);
		if (ret < 0)
			goto err_request_detect_irq;

		ret = request_irq(hi->irq_btn, button_irq_handler,
				IRQF_TRIGGER_LOW, "h2w_button", NULL);
		if (ret < 0)
			goto err_request_h2w_headset_button_irq;

		ret = set_irq_wake(hi->irq, 1);
		if (ret < 0)
			goto err_set_irq_wake;
		ret = set_irq_wake(hi->irq_btn, 1);
		if (ret < 0)
			goto err_set_irq_wake;
	}
	if (hi->ext_mic_sel)
		gpio_direction_output(hi->ext_mic_sel, 0);

	/*Fix me, work around: device will reboot */
	/* when call wake_lock_timeout() in detect_irq_handler at boot time*/
	msleep(500);
	hi->is_wake_lock_ready = 1;

	printk(KERN_INFO "H2W: Registering H2W (headset) driver finish\n");
	return 0;

err_set_irq_wake:
	if (hi->h2w_data)
		free_irq(hi->irq_btn, 0);
err_request_h2w_headset_button_irq:
	if (hi->cable_in1)
		free_irq(hi->irq, 0);
err_request_detect_irq:
err_register_common_headset:
	input_unregister_device(hi->input);
err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
err_get_button_irq_num_failed:
err_get_h2w_detect_irq_num_failed:
err_set_cablein2_gpio:
err_set_detect_gpio:
err_set_button_gpio:
	if (hi->h2w_data)
		gpio_free(hi->h2w_data);
err_request_button_gpio:
	if (hi->cable_in1)
		gpio_free(hi->cable_in1);
err_request_detect_gpio:
	destroy_workqueue(button_wq);
err_create_button_work_queue:
	destroy_workqueue(detect_wq);
err_create_detect_work_queue:
	switch_dev_unregister(&hi->sdev);
err_switch_dev_register:

	printk(KERN_ERR "H2W: Failed to register driver\n");

	return ret;
}

static int htc_35mm_remove(struct platform_device *pdev)
{
	H2W_DBG("");
	if ((switch_get_state(&hi->sdev) &
		(BIT_HEADSET | BIT_HEADSET_NO_MIC)) != 0)
		remove_headset();

	unregister_common_headset(hi);
	input_unregister_device(hi->input);
	if (hi->h2w_data) {
		gpio_free(hi->h2w_data);
		free_irq(hi->irq_btn, 0);
	}
	if (hi->cable_in1) {
		gpio_free(hi->cable_in1);
		free_irq(hi->irq, 0);
	}
	destroy_workqueue(detect_wq);
	destroy_workqueue(button_wq);
	switch_dev_unregister(&hi->sdev);

	return 0;
}


static struct platform_driver htc_35mm_driver = {
	.probe		= htc_35mm_probe,
	.remove		= htc_35mm_remove,
	.driver		= {
		.name		= "htc_headset",
		.owner		= THIS_MODULE,
	},
};


static int __init htc_35mm_init(void)
{
	H2W_DBG("");
	return platform_driver_register(&htc_35mm_driver);
}

static void __exit htc_35mm_exit(void)
{
	platform_driver_unregister(&htc_35mm_driver);
}

module_init(htc_35mm_init);
module_exit(htc_35mm_exit);

MODULE_AUTHOR("Arec Kao <Arec_Kao@htc.com>");
MODULE_DESCRIPTION("HTC 2 Wire detection driver");
MODULE_LICENSE("GPL");
