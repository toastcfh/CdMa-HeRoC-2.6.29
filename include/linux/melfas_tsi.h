/*
 * include/linux/melfas_tsi.h - platform data structure for f75375s sensor
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef _LINUX_melfas_TSI_H
#define _LINUX_melfas_TSI_H

#define MELFAS_I2C_NAME "melfas-tsi-ts"
#define melfas_I2C_ADDR	0x22

#define MELFAS_I2C_CMD_STATUS 0X00
#define MELFAS_I2C_CMD_OP_MODE 0x01
#define MELFAS_I2C_CMD_SENS_CONTROL 0x02
#define MELFAS_I2C_CMD_PARTIAL_SCAN_MODE 0x03
#define MELFAS_I2C_CMD_STANDBY_COUNT 0x04
#define MELFAS_I2C_CMD_SENSITIVITY 0x05
#define MELFAS_I2C_CMD_X_SIZE 0x08
#define MELFAS_I2C_CMD_Y_SIZE 0x0A
#define MELFAS_I2C_CMD_INPUT_INFORMATION 0x10
#define MELFAS_I2C_CMD_INPUT2_INFORMATION 0x16
#define MELFAS_I2C_CMD_SENSOR_REV 0x20
#define MELFAS_I2C_CMD_HARDWARE_REV 0x21
#define MELFAS_I2C_CMD_COMPATILITY_GROUP_REV 0x22
#define MELFAS_I2C_CMD_FIRMWARE_VER 0x23

#define MELFAS_I2C_OP_SLEEP_MODE 0x00
#define MELFAS_I2C_OP_ACTIVE_MODE (0x1 << 1)
#define MELFAS_I2C_OP_IDLE_MODE (0x01 << 2)

#define MELFAS_TRIANGLE_PATTERN 0x41
#define MELFAS_DIAMOND_PATTERN 0x45
enum {
	melfas_FLIP_X = 1UL << 0,
	melfas_FLIP_Y = 1UL << 1,
	melfas_SWAP_XY = 1UL << 2,
	melfas_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

struct melfas_virtual_key {
	int status;
	int keycode;
	int range_min;
	int range_max;
};

struct melfas_i2c_rmi_platform_data {
	const char *input_name;
	uint16_t key_type;
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	int (*reset)(void);
	struct melfas_virtual_key *virtual_key;
	int virtual_key_num;
	int intr;
	int wake_up;
	int tp_en;
};

#endif /* _LINUX_melfas_I2C_tsi_H */
