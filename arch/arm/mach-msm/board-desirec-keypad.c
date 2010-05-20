/* linux/arch/arm/mach-msm/board-desirec-keypad.c
 *
 * Copyright (C) 2008 HTC Corporation.
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


#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>

#include "board-desirec.h"

struct desirec_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
	uint16_t temp_state;
	uint16_t threshold;
};

static unsigned int desirec_col_gpios[] = { 35, 34, 33 };
static unsigned int desirec_row_gpios[] = { 42, 41, 40 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(desirec_row_gpios) + (row))

static const unsigned short desirec_keymap_0[ARRAY_SIZE(desirec_col_gpios) * ARRAY_SIZE(desirec_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_BACK,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_COMPOSE,

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = MATRIX_KEY(1, BTN_MOUSE),
};

static const unsigned short desirec_keymap_1[ARRAY_SIZE(desirec_col_gpios) * ARRAY_SIZE(desirec_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,

	[KEYMAP_INDEX(2, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 1)] = KEY_F13, /* add in XB, for device suspend/resume. in XA is KEY_RESERVED */
	[KEYMAP_INDEX(2, 2)] = MATRIX_KEY(1, BTN_MOUSE),
};

static const unsigned short desirec_keymap_2[ARRAY_SIZE(desirec_col_gpios) * ARRAY_SIZE(desirec_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,

	[KEYMAP_INDEX(2, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = MATRIX_KEY(1, BTN_MOUSE),
};

static struct gpio_event_matrix_info desirec_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = desirec_keymap_2,
	.output_gpios = desirec_col_gpios,
	.input_gpios = desirec_row_gpios,
	.noutputs = ARRAY_SIZE(desirec_col_gpios),
	.ninputs = ARRAY_SIZE(desirec_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.notintr_gpios = 40,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry desirec_keypad_nav_map[] = {
	{
		.gpio = DESIREC_POWER_KEY,
		.code = KEY_END
	},
};

static struct gpio_event_input_info desirec_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = 0,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = desirec_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(desirec_keypad_nav_map)
};

static bool nav_just_on;
static int nav_on_jiffies;

uint16_t desirec_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct desirec_axis_info *ai =
			container_of(info, struct desirec_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == ai->threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == ai->threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > ai->threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}

static uint32_t desirec_x_axis_gpios[] = {
	DESIREC_GPIO_BALL_LEFT, DESIREC_GPIO_BALL_RIGHT
};

static struct desirec_axis_info desirec_x_axis = {
	.threshold = 1,
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(desirec_x_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(desirec_x_axis_gpios),
		.map = desirec_axis_map,
		.gpio = desirec_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */
	}
};

static uint32_t desirec_y_axis_gpios[] = {
	DESIREC_GPIO_BALL_UP, DESIREC_GPIO_BALL_DOWN
};

static struct desirec_axis_info desirec_y_axis = {
	.threshold = 1,
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(desirec_y_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(desirec_y_axis_gpios),
		.map = desirec_axis_map,
		.gpio = desirec_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT  */
	}
};
/*
static struct gpio_event_direct_entry desirec_fake_key[] = {
	{ 0, BTN_MOUSE },
};

static struct gpio_event_input_info desirec_fake_nav_info = {
	.info.func = fake_gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = desirec_fake_key,
	.keymap_size = ARRAY_SIZE(desirec_fake_key)
};
*/
int desirec_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	if (on) {
		gpio_direction_input(desirec_x_axis_gpios[0]);
		gpio_direction_input(desirec_x_axis_gpios[1]);
		gpio_direction_input(desirec_y_axis_gpios[0]);
		gpio_direction_input(desirec_y_axis_gpios[1]);
		gpio_set_value(DESIREC_JOGBALL_EN, on);
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	} else {
		gpio_set_value(DESIREC_JOGBALL_EN, on);
		gpio_direction_output(desirec_x_axis_gpios[0], 0);
		gpio_direction_output(desirec_x_axis_gpios[1], 0);
		gpio_direction_output(desirec_y_axis_gpios[0], 0);
		gpio_direction_output(desirec_y_axis_gpios[1], 0);
	}
	return 0;
}

/*
static struct gpio_event_info *desirec_nav_info[] = {
	&desirec_x_axis.info.info,
	&desirec_y_axis.info.info,
	&desirec_fake_nav_info.info,
};

static struct gpio_event_platform_data desirec_nav_data = {
	.name = "desirec-nav",
	.info = desirec_nav_info,
	.info_count = ARRAY_SIZE(desirec_nav_info),
	.power = desirec_nav_power,
};

static struct platform_device desirec_nav_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 2,
	.dev		= {
		.platform_data	= &desirec_nav_data,
	},
};
*/

/*
static struct gpio_event_info *desirec_keypad_info[] = {
	&desirec_keypad_matrix_info.info,
	&desirec_keypad_nav_info.info,
};

static struct gpio_event_platform_data desirec_keypad_data = {
	.name = "desirec-keypad",
	.info = desirec_keypad_info,
	.info_count = ARRAY_SIZE(desirec_keypad_info)
};

static struct platform_device desirec_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &desirec_keypad_data,
	},
};

*/
static struct gpio_event_info *desirec_input_info[] = {
	&desirec_keypad_matrix_info.info,
	&desirec_keypad_nav_info.info,
	&desirec_x_axis.info.info,
	&desirec_y_axis.info.info,
};

static struct gpio_event_platform_data desirec_keypad_data = {
	.names = {
		"desirec-keypad",
		"desirec-nav",
		NULL,
	},
	.info = desirec_input_info,
	.info_count = ARRAY_SIZE(desirec_input_info),
	.power = desirec_nav_power,
};

static struct platform_device desirec_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &desirec_keypad_data,
	},
};

static int desirec_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data desirec_reset_keys_pdata = {
	.keys_up = desirec_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_VOLUMEUP,
		KEY_END,
		0
	},
};

static struct platform_device desirec_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &desirec_reset_keys_pdata,
};

static int __init desirec_init_keypad(void)
{
	if (!machine_is_desirec())
		return 0;
	if(system_rev == 0)
		desirec_keypad_matrix_info.keymap = desirec_keymap_0;
	else if(system_rev == 1)
		desirec_keypad_matrix_info.keymap = desirec_keymap_1;

	if (platform_device_register(&desirec_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&desirec_keypad_device);
}

device_initcall(desirec_init_keypad);
