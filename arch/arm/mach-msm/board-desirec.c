/* linux/arch/arm/mach-msm/board-desirec.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/melfas_tsi.h>
#include <mach/cy8c_i2c.h>
#include <linux/akm8973.h>
#include <mach/tpa6130.h>
#include <linux/bma150.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <mach/drv_callback.h>
#include <linux/proximity.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/capella_cm3602.h>

#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_fb.h>
#include "proc_comm.h"
#include "devices.h"
#include <mach/h2w_v1.h>
#include <mach/audio_jack.h>
#include <mach/microp_i2c.h>
#include <mach/htc_battery.h>
#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include "board-desirec.h"
#include "board-desirec-camsensor.h"
#include <mach/board_htc.h>
#include <mach/msm_serial_debugger.h>

static struct htc_battery_platform_data htc_battery_pdev_data = {
//	.gpio_mbat_in = HERO_GPIO_MBAT_IN,
//	.gpio_mchg_en_n = HERO_GPIO_MCHG_EN_N,
//	.gpio_iset = HERO_GPIO_ISET,
	.guage_driver = GUAGE_MODEM,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

static int touch_reset(void)
{
	printk(KERN_INFO"%s: enter\n", __func__);
	gpio_set_value(DESIREC_GPIO_TP_EN, 0);
	msleep(2);
	gpio_set_value(DESIREC_GPIO_TP_EN, 1);
	msleep(260);
	gpio_set_value(DESIREC_GPIO_WAKE_UP, 1);
	msleep(40);
	gpio_set_value(DESIREC_GPIO_WAKE_UP, 0);

	return 0;
}

/*
static struct proximity_platform_data desirec_proximity_data = {
	.intr = DESIREC_GPIO_PROXIMITY_INT_N,
	.enable = DESIREC_GPIO_PROXIMITY_POWER_N,
};

static struct platform_device desirec_proximity_device = {
	.name = "proximity_sensor",
	.dev		= {
		.platform_data	= &desirec_proximity_data,
	},
};
*/

static struct melfas_i2c_rmi_platform_data desirec_melfas_ts_data[] = {
	{
		.version = MELFAS_DIAMOND_PATTERN,
		.wake_up = DESIREC_GPIO_WAKE_UP,
		.intr = DESIREC_GPIO_TP_ATT_N,
		.reset = touch_reset,
		.tp_en = DESIREC_GPIO_TP_EN,
	},
	{
		.version = MELFAS_TRIANGLE_PATTERN,
		.wake_up = DESIREC_GPIO_WAKE_UP,
		.intr = DESIREC_GPIO_TP_ATT_N,
		.reset = touch_reset,
		.tp_en = DESIREC_GPIO_TP_EN,
	}
};

static struct synaptics_i2c_rmi_platform_data desirec_ts_data[] = {
	{
		.version = 0x0105,
		.inactive_left = -10 * 0x10000 / 320,
		.inactive_right = -10 * 0x10000 / 320,
		.inactive_top = -5 * 0x10000 / 480,
		.inactive_bottom = -35 * 0x10000 / 480,
		.dup_threshold = 10,
		.display_width = 320,
		.display_height = 480,

	},
	{
		.version = 0x0100,
		.sensitivity_adjust = 8,
		.inactive_left = -10 * 0x10000 / 320,
		.inactive_right = -10 * 0x10000 / 320,
		.inactive_top = -5 * 0x10000 / 480,
		.inactive_bottom = -35 * 0x10000 / 480,
		.dup_threshold = 10,
		.display_width = 320,
		.display_height = 480,
	},
};


static struct microp_pin_config microp_pins_0[] = {
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 6,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 8,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "low-power",
		.pin	= 9,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 1,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 0,
	},
	{
		.name	= "35mm_adc",
		.pin	= 16,
		.adc_pin = 1,
		.intr_pin = 1,
		.config = MICROP_PIN_CONFIG_UP_ADC,
		.levels = { 200, 0x3FF, 0, 33, 38, 82, 95, 167 },
	},
	{
		.name   = "adc",
		.pin    = 17,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 14, 16, 20, 20, 41, 158, 277, 451, 575 },
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 18,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask	 = { 0x00, 0x00, 0x00 },
	},
	{
		.pin	= 2,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 4,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 11,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 12,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 13,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 14,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.pin	= 15,
		.config = MICROP_PIN_CONFIG_GPO,
	},
};

static struct microp_pin_config microp_pins_2[] = {
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 6,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "vtkey-backlight",
		.pin	= 9,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.dutys = {0, 255, 255, 255, 255, 255, 255, 255, 255, 255},
	},
	{
		.name	= "jogball-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.auto_if_on = 1,
		.i_am_jogball_function = 1,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 1,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 0,
	},
	{
		.name	= "35mm_adc",
		.pin	= 16,
		.adc_pin = 1,
		.intr_pin = 1,
		.config = MICROP_PIN_CONFIG_UP_ADC,
		.levels = { 200, 0x3FF, 0, 33, 38, 82, 95, 167 },
	},
	{
		.name   = "adc",
		.pin    = 17,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 14, 16, 20, 20, 41, 158, 277, 451, 575 },
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 18,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask	 = { 0x00, 0x00, 0x00 },
	},
	{
		.pin	= 2,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 4,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 8,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 11,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 12,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 13,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.pin	= 14,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.pin	= 15,
		.config = MICROP_PIN_CONFIG_GPO,
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_pins   = ARRAY_SIZE(microp_pins_0),
	.pin_config = microp_pins_0,
	.gpio_reset = DESIREC_GPIO_UP_RESET_N,
	.cabc_backlight_enable = 1,
	.microp_enable_early_suspend = 1,
	.microp_enable_reset_button = 1,
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = DESIREC_LAYOUTS,
	.project_name = DESIREC_PROJECT_NAME,
	.reset = DESIREC_GPIO_COMPASS_RST_N,
	.intr = DESIREC_GPIO_COMPASS_INT_N,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = DESIREC_GPIO_GSENSOR_INT_N,
};

static struct tpa6130_platform_data headset_amp_platform_data = {
	.gpio_hp_sd = DESIREC_GPIO_HTC_HP_SD,
	.enable_rpc_server = 1,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = &desirec_ts_data,
		.irq = DESIREC_GPIO_TO_INT(DESIREC_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MELFAS_I2C_NAME, 0x22),
		.platform_data = &desirec_melfas_ts_data,
		.irq = DESIREC_GPIO_TO_INT(DESIREC_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = DESIREC_GPIO_TO_INT(DESIREC_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = DESIREC_GPIO_TO_INT(DESIREC_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO(BMA150_I2C_NAME, 0x38),
		.platform_data = &gsensor_platform_data,
		.irq = DESIREC_GPIO_TO_INT(DESIREC_GPIO_GSENSOR_INT_N),
	},

	{
		I2C_BOARD_INFO(TPA6130_I2C_NAME, 0xC0 >> 1),
		.platform_data = &headset_amp_platform_data,
	},

	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1)
	}
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_desirec_camera_on_gpios,
	.camera_gpio_off = config_desirec_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = DESIREC_GPIO_CAM_RST_N,
	.sensor_pwd     = DESIREC_CAM_PWDN,
	.vcm_pwd        = DESIREC_GPIO_VCM_PWDN,
	.pdata          = &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_NONE,
	.need_suspend   = 1,
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev        = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

static void desirec_phy_reset(void)
{
	printk("desirec_phy_reset\n");
	gpio_set_value(DESIREC_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(DESIREC_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

static void desirec_phy_shutdown(void)
{
	printk("desirec_phy_shutdown\n");
	gpio_set_value(DESIREC_GPIO_USB_PHY_RST_N, 0);
}

static struct pwr_sink desirec_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 100000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 125000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_BLUETOOTH,
		.ua_max	= 15000,
	},
	{
		.id	= PWRSINK_CAMERA,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_SDCARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_VIDEO,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id	= PWRSINK_SYSTEM_LOAD,
		.ua_max	= 100000,
		.percent_util = 38,
	},
};

static int desirec_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void desirec_pwrsink_resume_late(struct early_suspend *h)
{
	printk(KERN_INFO "desirec_pwrsink_resume_late\n");
	gpio_direction_output(DESIREC_GPIO_TP_EN, 1); /* for melfas workarround*/
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void desirec_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int desirec_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data desirec_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(desirec_pwrsink_table),
	.sinks		= desirec_pwrsink_table,
	.suspend_late	= desirec_pwrsink_suspend_late,
	.resume_early	= desirec_pwrsink_resume_early,
	.suspend_early	= desirec_pwrsink_suspend_early,
	.resume_late	= desirec_pwrsink_resume_late,
};

static struct platform_device desirec_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &desirec_pwrsink_data,
	},
};
/* Switch between UART3 and GPIO */
static uint32_t uart3_on_gpio_table[] = {
	/* RX */
	PCOM_GPIO_CFG(DESIREC_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_NO_PULL, 0),
	/* TX */
	PCOM_GPIO_CFG(DESIREC_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, 0),
};

/* default TX,RX to GPI */
static uint32_t uart3_off_gpi_table[] = {
	/* RX, H2W DATA */
	PCOM_GPIO_CFG(DESIREC_GPIO_H2W_DATA, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	/* TX, H2W CLK */
	PCOM_GPIO_CFG(DESIREC_GPIO_H2W_CLK, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
};

/* set TX,RX to GPO */
static uint32_t uart3_off_gpo_table[] = {
	/* RX, H2W DATA */
	PCOM_GPIO_CFG(DESIREC_GPIO_H2W_DATA, 0,
		      GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	/* TX, H2W CLK */
	PCOM_GPIO_CFG(DESIREC_GPIO_H2W_CLK, 0,
		      GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static int desirec_h2w_path = H2W_GPIO;

static void h2w_configure(int route)
{
	printk(KERN_INFO "H2W route = %d \n", route);
	switch (route) {
	case H2W_UART3:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+1, 0);
		desirec_h2w_path = H2W_UART3;
		printk(KERN_INFO "H2W -> UART3\n");
		break;
	case H2W_GPIO:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+1, 0);
		desirec_h2w_path = H2W_GPIO;
		printk(KERN_INFO "H2W -> GPIO\n");
		break;
	}
}

static void h2w_defconfig(void)
{
	h2w_configure(H2W_GPIO);
}

static void set_h2w_dat(int n)
{
	gpio_set_value(DESIREC_GPIO_H2W_DATA, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(DESIREC_GPIO_H2W_CLK, n);
}

static void set_h2w_dat_dir(int n)
{
#if 0
	if (n == 0) /* input */
		gpio_direction_input(DESIREC_GPIO_H2W_DATA);
	else
		gpio_configure(DESIREC_GPIO_H2W_DATA, GPIOF_DRIVE_OUTPUT);
#else
	if (n == 0) /* input */
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+0, 0);
	else
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpo_table+0, 0);
#endif
}

static void set_h2w_clk_dir(int n)
{
#if 0
	if (n == 0) /* input */
		gpio_direction_input(DESIREC_GPIO_H2W_CLK);
	else
		gpio_configure(DESIREC_GPIO_H2W_CLK, GPIOF_DRIVE_OUTPUT);
#else
	if (n == 0) /* input */
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+1, 0);
	else
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpo_table+1, 0);
#endif
}

static int get_h2w_dat(void)
{
	return gpio_get_value(DESIREC_GPIO_H2W_DATA);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(DESIREC_GPIO_H2W_CLK);
}

#ifdef CONFIG_HTC_HEADSET_V1
static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	int enable;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (desirec_h2w_path) {
	case H2W_GPIO:
		enable = 1;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	case H2W_UART3:
		enable = 0;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	default:
		desirec_h2w_path = -1;
		return -EINVAL;
	}

	h2w_configure(desirec_h2w_path);
	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&desirec_h2w_path, S_IWUSR | S_IRUGO);

#endif
static struct h2w_platform_data desirec_h2w_data = {
	.h2w_power		= DESIREC_GPIO_H2W_POWER,
	.cable_in1		= DESIREC_GPIO_CABLE_IN1,
	.cable_in2		= DESIREC_GPIO_CABLE_IN2,
	.h2w_clk		= DESIREC_GPIO_H2W_CLK,
	.h2w_data		= DESIREC_GPIO_H2W_DATA,
	.ext_mic_sel		= DESIREC_GPIO_AUD_EXTMIC_SEL,
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
};

static struct platform_device desirec_h2w = {
	.name		= "htc_headset",
	.id			= -1,
	.dev		= {
		.platform_data	= &desirec_h2w_data,
	},
};

static struct audio_jack_platform_data desirec_jack_data = {
	.gpio	= DESIREC_GPIO_35MM_HEADSET_DET,
};

static struct platform_device desirec_audio_jack = {
	.name		= "audio-jack",
	.id			= -1,
	.dev		= {
		.platform_data	= &desirec_jack_data,
	},
};

static struct platform_device desirec_rfkill = {
	.name = "desirec_rfkill",
	.id = -1,
};

/* Proximity Sensor (Capella_CM3502)*/
static int __capella_cm3602_power(int on)
{
	int rc;
	struct vreg *vreg = vreg_get(0, "ruim");
	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2800);

	if (on) {
		gpio_direction_output(DESIREC_GPIO_PROXIMITY_EN, 1);
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
		gpio_direction_output(DESIREC_GPIO_PROXIMITY_EN, 0);
	}

	return rc;
}

static DEFINE_SPINLOCK(capella_cm3602_lock);
static int capella_cm3602_open_cnt;

static int capella_cm3602_power(int on)
{
	int rc = 0;
	unsigned long flags;
	spin_lock_irqsave(&capella_cm3602_lock, flags);

	if (on) {
		if (!capella_cm3602_open_cnt++)
			rc = __capella_cm3602_power(1);
	} else if (capella_cm3602_open_cnt)
		if (!--capella_cm3602_open_cnt)
			rc = __capella_cm3602_power(0);

	spin_unlock_irqrestore(&capella_cm3602_lock, flags);
	return rc;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = DESIREC_GPIO_PROXIMITY_EN,
	.p_out = DESIREC_GPIO_PROXIMITY_INT_N
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
/* End Proximity Sensor (Capella_CM3502)*/

static struct msm_pmem_setting pmem_dual_die_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
#ifdef CONFIG_BUILD_CIQ
	.pmem_ciq_start = MSM_PMEM_CIQ_BASE,
	.pmem_ciq_size = MSM_PMEM_CIQ_SIZE,
	.pmem_ciq1_start = MSM_PMEM_CIQ1_BASE,
	.pmem_ciq1_size = MSM_PMEM_CIQ1_SIZE,
	.pmem_ciq2_start = MSM_PMEM_CIQ2_BASE,
	.pmem_ciq2_size = MSM_PMEM_CIQ2_SIZE,
	.pmem_ciq3_start = MSM_PMEM_CIQ3_BASE,
	.pmem_ciq3_size = MSM_PMEM_CIQ3_SIZE,
/*
	.pmem_ciq4_start = MSM_PMEM_CIQ4_BASE,
	.pmem_ciq4_size = MSM_PMEM_CIQ4_SIZE,
*/
#endif
};

#define MONO_DIE_PMEM_SHIFT  0x8000000

static struct msm_pmem_setting pmem_mono_die_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE - MONO_DIE_PMEM_SHIFT,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE - MONO_DIE_PMEM_SHIFT,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE - MONO_DIE_PMEM_SHIFT,
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE - MONO_DIE_PMEM_SHIFT,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
#ifdef CONFIG_BUILD_CIQ
	.pmem_ciq_start = MSM_PMEM_CIQ_BASE,
	.pmem_ciq_size = MSM_PMEM_CIQ_SIZE,
	.pmem_ciq1_start = MSM_PMEM_CIQ1_BASE,
	.pmem_ciq1_size = MSM_PMEM_CIQ1_SIZE,
	.pmem_ciq2_start = MSM_PMEM_CIQ2_BASE,
	.pmem_ciq2_size = MSM_PMEM_CIQ2_SIZE,
	.pmem_ciq3_start = MSM_PMEM_CIQ3_BASE,
	.pmem_ciq3_size = MSM_PMEM_CIQ3_SIZE,
/*
	.pmem_ciq4_start = MSM_PMEM_CIQ4_BASE,
	.pmem_ciq4_size = MSM_PMEM_CIQ4_SIZE,
*/
#endif
};

static struct msm_i2c_device_platform_data desirec_i2c_device_data = {
	.i2c_clock = 100000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_i2c,
	&desirec_h2w,
	&htc_battery_pdev,
	&desirec_audio_jack,
	&desirec_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&desirec_pwr_sink,
#endif
	&capella_cm3602,
	&msm_camera_sensor_s5k3e2fx
};

static struct platform_device *devices0[] __initdata = {
	&msm_device_i2c,
	&desirec_h2w,
	&htc_battery_pdev,
	&desirec_audio_jack,
	&desirec_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&desirec_pwr_sink,
#endif
	&msm_camera_sensor_s5k3e2fx
};

extern struct sys_timer msm_timer;

static void __init desirec_init_irq(void)
{
	printk("desirec_init_irq()\n");
	msm_init_irq();
}

static uint opt_disable_uart3;

module_param_named(disable_uart3, opt_disable_uart3, uint, 0);

static void clear_bluetooth_rx_irq_status(void)
{
	#define GPIO_INT_CLEAR_2 (MSM_GPIO1_BASE + 0x800 + 0x94)
	writel((1U << (DESIREC_GPIO_UART1_RX-43)), GPIO_INT_CLEAR_2);
}

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void desirec_reset(void)
{
	gpio_set_value(DESIREC_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	/* I2C */
	PCOM_GPIO_CFG(DESIREC_GPIO_I2C_CLK, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_I2C_DAT , 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
};


static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
/*	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),*/ /* MCLK */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
	/*steven yeh: modify MCLK driving strength to avoid overshot issue*/
};

void config_desirec_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_desirec_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
}

static struct msm_acpu_clock_platform_data desirec_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
#if defined(CONFIG_TURBO_MODE)
	.wait_for_irq_khz = 176000000,
#else
	.wait_for_irq_khz = 128000000,
#endif
};

static unsigned desirec_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};

static struct perflock_platform_data desirec_perflock_data = {
	.perf_acpu_table = desirec_perf_acpu_table,
	.table_size = ARRAY_SIZE(desirec_perf_acpu_table),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(DESIREC_GPIO_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif

static ssize_t desirec_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	/* center: x: home: 30, menu: 112, back: 215, search 295, y: 490*/
	return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":30:490:58:2"
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":112:490:94:2"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":215:490:78:2"
			":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":295:490:48:2"
			"\n");
}

static struct kobj_attribute desirec_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.melfas-tsi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &desirec_virtual_keys_show,
};

static ssize_t desirec_synaptics_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	/* center: x: home: 30, menu: 110, back: 205, search 285, y: 510 */
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":30:510:60:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":110:510:100:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":205:510:90:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":285:510:70:55"
	   "\n");
}

static struct kobj_attribute desirec_synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &desirec_synaptics_virtual_keys_show,
};

static struct attribute *desirec_properties_attrs[] = {
	&desirec_virtual_keys_attr.attr,
	&desirec_synaptics_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group desirec_properties_attr_group = {
	.attrs = desirec_properties_attrs,
};

static void __init desirec_init(void)
{
	int rc;
	struct kobject *properties_kobj;
	printk(KERN_INFO "%s() revision = %d\n", __func__, system_rev);

	config_gpios();

	if (system_rev > 0) {
		/* We need to set this pin to 0 only once on power-up; we will
		 * not actually enable the chip until we apply power to it via
		 * vreg.
		 */
		gpio_direction_output(DESIREC_GPIO_CM3602_EN, 0);
		/* disable power for cm3602 chip */
		__capella_cm3602_power(0);
	}

	msm_hw_reset_hook = desirec_reset;

	msm_acpu_clock_init(&desirec_clock_data);
	perflock_init(&desirec_perflock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1, INT_UART3_RX);
#endif

	msm_add_devices();

	clear_bluetooth_rx_irq_status();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_add_serial_devices(MSM_SERIAL_UART1DM);
#else
	msm_add_serial_devices(MSM_SERIAL_UART1);
#endif

	msm_add_serial_devices(MSM_SERIAL_UART3);

	msm_add_usb_devices(desirec_phy_reset, desirec_phy_shutdown);

	if (board_mcp_monodie())
		msm_add_mem_devices(&pmem_mono_die_setting);
	else
		msm_add_mem_devices(&pmem_dual_die_setting);

	msm_init_pmic_vibrator();

	rc = desirec_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
						&desirec_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	msm_device_i2c.dev.platform_data = &desirec_i2c_device_data;

	if (system_rev > 0) /*Aobve XB*/
		platform_add_devices(devices, ARRAY_SIZE(devices));
	else /* XA */
		platform_add_devices(devices0, ARRAY_SIZE(devices0));

	if (system_rev > 0) {
		microp_data.ls_power = capella_cm3602_power;
		if (system_rev >= 2) {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_2);
			microp_data.pin_config = microp_pins_2;
		}
	}

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
/* r porting*///	i2c_register_board_info(0, &i2c_s5k3e2fx, 1);
}

static void __init desirec_fixup(struct machine_desc *desc, struct tag *tags,
			      char **cmdline, struct meminfo *mi)
{
	parse_tag_monodie((const struct tag *)tags);

	mi->nr_banks = 1;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
	if (board_mcp_monodie()) {
		mi->bank[0].size = MSM_LINUX_SIZE1 + MSM_LINUX_SIZE2;
	} else {
		mi->nr_banks = 2;
		mi->bank[0].size = MSM_LINUX_SIZE1;
		mi->bank[1].start = MSM_LINUX_BASE2;
		mi->bank[1].node = PHYS_TO_NID(MSM_LINUX_BASE2);
		mi->bank[1].size = MSM_LINUX_SIZE2;
	}
}

static void __init desirec_map_io(void)
{
	msm_map_common_io();
	msm_clock_init();
}

MACHINE_START(DESIREC, "desirec")
/* Maintainer: Kant Kang <kant_kang@htc.com> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x11200100,
	.fixup          = desirec_fixup,
	.map_io         = desirec_map_io,
	.init_irq       = desirec_init_irq,
	.init_machine   = desirec_init,
	.timer          = &msm_timer,
MACHINE_END
