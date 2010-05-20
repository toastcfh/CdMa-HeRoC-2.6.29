/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "ov9665.h"

/* OV9665 Registers and their values */
/* Sensor Core Registers */
#define  REG_OV9665_MODEL_ID_H 0x0A
#define  REG_OV9665_MODEL_ID_L 0x0B
#define  OV9665_MODEL_ID     0x9663

/*  SOC Registers Page 1  */
#define  REG_OV9665_SENSOR_RESET     0x301A
#define  REG_OV9665_STANDBY_CONTROL  0x3202
#define  REG_OV9665_MCU_BOOT         0x3386

struct ov9665_work {
	struct work_struct work;
};

static struct ov9665_work *ov9665_sensorw;
static struct i2c_client *ov9665_client;

struct ov9665_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

static struct ov9665_ctrl *ov9665_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(ov9665_wait_queue);
DECLARE_MUTEX(ov9665_sem);


static int ov9665_i2c_txdata(unsigned short saddr,
				  unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};

	if (i2c_transfer(ov9665_client->adapter, msg, 1) < 0) {
		pr_info("ov9665_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int ov9665_i2c_write(unsigned short saddr,
				 unsigned char waddr, unsigned char wdata,
				 enum ov9665_width width)
{
	int rc = -EIO;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN:{
			buf[0] = (waddr & 0xFF00) >> 8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = (wdata & 0xFF00) >> 8;
			buf[3] = (wdata & 0x00FF);

			rc = ov9665_i2c_txdata(saddr, buf, 4);
		}
		break;

	case BYTE_LEN:{
			buf[0] = waddr;
			buf[1] = wdata;
			rc = ov9665_i2c_txdata(saddr, buf, 2);
		}
		break;

	default:
		break;
	}

	if (rc < 0)
		pr_info("i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		     waddr, wdata);

	return rc;
}

static int ov9665_i2c_write_table(struct ov9665_i2c_reg_conf const
				       *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = ov9665_i2c_write(ov9665_client->addr,
				       reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				       reg_conf_tbl->width);
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

static int ov9665_i2c_rxdata(unsigned short saddr,
			      unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxdata,
		 },
		{
		 .addr = saddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxdata,
		 },
	};

	if (i2c_transfer(ov9665_client->adapter, msgs, 2) < 0) {
		pr_info("ov9665_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}


static int ov9665_i2c_read(unsigned short saddr,
				unsigned short raddr, unsigned char *rdata)
{
	int rc = 0;
	unsigned char buf[1];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = ov9665_i2c_rxdata(saddr, buf,1);
	if (rc < 0)
		return rc;
	*rdata = buf[0];
	if (rc < 0)
		pr_info("ov9665_i2c_read failed!\n");

	return rc;
}

static int ov9665_pwd(const struct msm_camera_sensor_info *info){
	int rc=0;
	rc = gpio_request(info->sensor_pwd, "ov9665");
	if (!rc)
		gpio_direction_output(info->sensor_pwd, 0);
	else
		pr_err("GPIO(%d) request faile",info->sensor_pwd);
	gpio_free(info->sensor_pwd);
	
	return rc;
}

static int ov9665_reset(const struct msm_camera_sensor_info *dev)
{
	int rc = 0;
	rc = gpio_request(dev->sensor_reset, "ov9665");

	if (!rc)
		rc = gpio_direction_output(dev->sensor_reset, 1);
	else
		pr_err("GPIO(%d) request faile",dev->sensor_reset);
	
	gpio_free(dev->sensor_reset);
	return rc;
}

static int ov9665_set_lens_roll_off(void)
{
	return 0;
#if 0
	int rc = 0;
	rc = ov9665_i2c_write_table(&ov9665_regs.rftbl[0],
			ov9665_regs.rftbl_size);
	return rc;
#endif
}

static int ov9665_reg_init(void)
{
	int rc = 0;
	rc = ov9665_i2c_write_table(&ov9665_regs.register_init[0],
			ov9665_regs.register_init_size);
	return rc;
}

static int ov9665_init_done(const struct msm_camera_sensor_info *info)
{
	int rc = 0;
	rc = gpio_request(info->sensor_pwd, "ov9665");
	if (!rc)
		gpio_direction_output(info->sensor_pwd, 1);
	else
		pr_err("GPIO(%d) request faile",info->sensor_pwd);
	gpio_free(info->sensor_pwd);
	return rc;
}


static int ov9665_set_sensor_mode(int mode)
{

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		pr_info("ov9665:sensor set mode: preview\n");
		ov9665_i2c_write(ov9665_client->addr, 0x63, 0x00, BYTE_LEN);
		/*Windowing*/
		ov9665_i2c_write(ov9665_client->addr, 0x12, 0x40, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x4d, 0x09, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x17, 0x0c, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x18, 0x5d, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x19, 0x02, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x1a, 0x3f, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x03, 0x03, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x32, 0xad, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x2b, 0x00, BYTE_LEN);
		/*scaling*/
		ov9665_i2c_write(ov9665_client->addr, 0x64, 0xa4, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xab, 0xe7, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xb9, 0x50, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xba, 0x3c, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xbb, 0x50, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xbc, 0x3c, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x85, 0xe7, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x0d, 0x92, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x0d, 0x90, BYTE_LEN);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_info("ov9665:sensor set mode: snapshot\n");
		ov9665_i2c_write(ov9665_client->addr, 0x63, 0x01, BYTE_LEN);
		/*Windowing*/
		ov9665_i2c_write(ov9665_client->addr, 0x12, 0x00, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x4d, 0x11, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x17, 0x0c, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x18, 0x5d, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x19, 0x01, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x1a, 0x82, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x03, 0x03, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x32, 0x24, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x2b, 0x00, BYTE_LEN);
		/*scaling*/
		ov9665_i2c_write(ov9665_client->addr, 0x64, 0x24, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xab, 0xe7, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xb9, 0xa0, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xba, 0x80, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xbb, 0xa0, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0xbc, 0x80, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x85, 0xe7, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x0d, 0x82, BYTE_LEN);
		ov9665_i2c_write(ov9665_client->addr, 0x0d, 0x80, BYTE_LEN);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ov9665_set_effect(int mode, int effect)
{
	uint16_t reg_addr;
	uint16_t reg_val;
	int rc = 0;

	switch (effect) {
		case CAMERA_EFFECT_OFF:
			break;

		case CAMERA_EFFECT_MONO:
			break;

		case CAMERA_EFFECT_NEGATIVE:
			break;

		case CAMERA_EFFECT_SOLARIZE:
			break;

		case CAMERA_EFFECT_SEPIA:
			break;

		case CAMERA_EFFECT_PASTEL:
		case CAMERA_EFFECT_MOSAIC:
		case CAMERA_EFFECT_RESIZE:
			return -EINVAL;

		default:
			return -EINVAL;
	}

}

static int ov9665_sensor_init(const struct msm_camera_sensor_info *data)
{
	uint8_t model_id_h = 0,model_id_l = 0;
	uint16_t model_id;
	int rc = 0;
	pr_info("ov9665_sensor_init_probe \n");
	/* Read the Model ID of the sensor */
	rc = ov9665_i2c_read(ov9665_client->addr,
			      REG_OV9665_MODEL_ID_H, &model_id_h);
	if (rc < 0)
		goto init_probe_fail;
	
	rc = ov9665_i2c_read(ov9665_client->addr,
			      REG_OV9665_MODEL_ID_L, &model_id_l);
	if (rc < 0)
		goto init_probe_fail;
	model_id = (((model_id_h << 8) & 0xFF00) +(model_id_l));
	pr_info("ov9665: model_id = 0x%x\n", model_id);
	/* Check if it matches it with the value in Datasheet */
	if (model_id != OV9665_MODEL_ID) {
		rc = -EINVAL;
		goto init_probe_fail;
	}

	
	return rc;
init_probe_fail:
	return rc;
}

int ov9665_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int rc = 0;
	ov9665_ctrl = kzalloc(sizeof(struct ov9665_ctrl), GFP_KERNEL);
	if (!ov9665_ctrl) {
		pr_info("ov9665_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data == NULL) {
		pr_err("%s sensor data is NULL\n", __func__);
		return -EINVAL;
	}
	ov9665_ctrl->sensordata = data;
	/*switch PCLK and MCLK to 2nd cam*/
	pr_info("ov9665: ov9665_sensor_probe switch clk\n");
	if(data->camera_clk_switch != NULL)
		data->camera_clk_switch();

	/* Config power down */
	rc = gpio_request(data->sensor_pwd, "ov9665");
	if (!rc)
		gpio_direction_output(data->sensor_pwd, 0);
	else
		pr_info("GPIO(%d) request faile",data->sensor_pwd);
	gpio_free(data->sensor_pwd);
	mdelay(3);

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	/*read ID*/
	rc = ov9665_sensor_init(data);
	if (rc < 0) {
		pr_info("ov9665_sensor_init failed!\n");
		goto init_fail;
	}
	/*set initial register*/
	rc = ov9665_reg_init();
	if (rc < 0) {
		pr_info("ov9665_sensor_reg_init failed!\n");
		goto init_fail;
	}
init_done:
	return rc;

init_fail:
	kfree(ov9665_ctrl);
	return rc;
}

static int ov9665_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov9665_wait_queue);
	return 0;
}

int ov9665_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;
	if (copy_from_user(&cfg_data,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	pr_info("ov9665_ioctl, cfgtype = %d, mode = %d\n",
	     cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = ov9665_set_sensor_mode(cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = ov9665_set_effect(cfg_data.mode, cfg_data.cfg.effect);
		break;

	case CFG_GET_AF_MAX_STEPS:
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int ov9665_sensor_release(void)
{
	int rc = 0;
	down(&ov9665_sem);

	rc = gpio_request(ov9665_ctrl->sensordata->sensor_pwd, "ov9665");
	if (!rc)
		gpio_direction_output(ov9665_ctrl->sensordata->sensor_pwd, 1);
	else
		pr_info("GPIO(%d) request faile",
			ov9665_ctrl->sensordata->sensor_pwd);
	gpio_free(ov9665_ctrl->sensordata->sensor_pwd);

	kfree(ov9665_ctrl);
	ov9665_ctrl = NULL;

	up(&ov9665_sem);

	return rc;
}

static const char *Ov9665Vendor = "OmniVision";
static const char *Ov9665NAME = "ov9665";
static const char *Ov9665Size = "1M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", Ov9665Vendor, Ov9665NAME, Ov9665Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov9665;

static int ov9665_sysfs_init(void)
{
	int ret ;
	pr_info("ov9665:kobject creat and add\n");
	android_ov9665 = kobject_create_and_add("android_camera2", NULL);
	if (android_ov9665 == NULL) {
		pr_info("ov9665_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov9665:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov9665, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov9665_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov9665);
	}
	return 0 ;
}


static int ov9665_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	ov9665_sensorw = kzalloc(sizeof(struct ov9665_work), GFP_KERNEL);

	if (!ov9665_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov9665_sensorw);
	ov9665_init_client(client);
	ov9665_client = client;

	pr_info("ov9665_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(ov9665_sensorw);
	ov9665_sensorw = NULL;
	pr_info("ov9665_probe failed!\n");
	return rc;
}

static const struct i2c_device_id ov9665_i2c_id[] = {
	{"ov9665", 0},
	{},
};

static struct i2c_driver ov9665_i2c_driver = {
	.id_table = ov9665_i2c_id,
	.probe = ov9665_i2c_probe,
	.remove = __exit_p(ov9665_i2c_remove),
	.driver = {
		   .name = "ov9665",
		   },
};

static int ov9665_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&ov9665_i2c_driver);
	if (rc < 0 || ov9665_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}
	/*switch clk source*/
	pr_info("ov9665: ov9665_sensor_probe switch clk\n");
	if(info->camera_clk_switch != NULL)
		info->camera_clk_switch();
	
	/* Config power down */

	if(ov9665_pwd(info)<0)
		goto probe_fail;
	mdelay(3);

	/*Config reset */
	if(ov9665_reset(info)<0)
		goto probe_fail;
	mdelay(5);

	/*MCLK enable*/
	pr_info("ov9665: MCLK enable clk\n");
	msm_camio_clk_rate_set(24000000);
	mdelay(100);

	/* PLL Setup Start */
	rc = ov9665_i2c_write_table(&ov9665_regs.plltbl[0],
				     ov9665_regs.plltbl_size);

	rc = ov9665_sensor_init(info);
	if (rc < 0)
		goto probe_fail;
	/*set initial register*/
	rc = ov9665_reg_init();
	if (rc < 0)
		goto probe_fail;

	s->s_init = ov9665_sensor_open_init;
	s->s_release = ov9665_sensor_release;
	s->s_config = ov9665_sensor_config;

	/*init done*/
	mdelay(800);
	ov9665_init_done(info);
	ov9665_sysfs_init();

probe_done:
	pr_info("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
probe_fail:
	pr_err("OV9665 probe faile\n");
	return rc;

}

static int __ov9665_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov9665_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov9665_probe,
	.driver = {
		   .name = "msm_camera_ov9665",
		   .owner = THIS_MODULE,
		   },
};

static int __init ov9665_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov9665_init);
