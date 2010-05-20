/* linux/arch/arm/mach-msm/board-desirec.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_DESIREC_H
#define __ARCH_ARM_MACH_MSM_BOARD_DESIREC_H

#include <mach/board.h>

#define MSM_LINUX_BASE         0x19200000
#define MSM_LINUX_SIZE         0xC600000

#define MSM_LINUX_BASE1         0x11200000
#define MSM_LINUX_SIZE1         0x6E00000
#define MSM_LINUX_BASE2         0x20000000
#define MSM_LINUX_SIZE2         0x5800000

#define MSM_PMEM_GPU0_BASE      0x00000000
#define MSM_PMEM_GPU0_SIZE      0x00700000

#define MSM_FB_BASE             0x00700000
#define MSM_FB_SIZE             0x9b000

#define MSM_RAM_CONSOLE_BASE    0x007A0000
#define MSM_RAM_CONSOLE_SIZE    128 * SZ_1K

#ifdef CONFIG_BUILD_CIQ
#define MSM_PMEM_CIQ_BASE		MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE
#define MSM_PMEM_CIQ_SIZE		64 * SZ_1K
#define MSM_PMEM_CIQ1_BASE		MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ1_SIZE		MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ2_BASE		MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ2_SIZE		MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ3_BASE		MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ3_SIZE		MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ4_BASE		MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ4_SIZE		MSM_PMEM_CIQ_SIZE
#endif

#define MSM_PMEM_GPU1_BASE      0x25800000
#define MSM_PMEM_GPU1_SIZE      0x00800000

#define MSM_PMEM_MDP_BASE       0x26000000
#define MSM_PMEM_MDP_SIZE       0x00800000

#define MSM_PMEM_ADSP_BASE      0x26800000
#define MSM_PMEM_ADSP_SIZE      0x01000000

#define MSM_PMEM_CAMERA_BASE	0x27800000
#define MSM_PMEM_CAMERA_SIZE    0x00800000

#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>

#define DESIREC_GPIO_BALL_UP     (94)
#define DESIREC_GPIO_BALL_LEFT   (39)
#define DESIREC_GPIO_BALL_DOWN   (90)
#define DESIREC_GPIO_BALL_RIGHT  (37)
#define DESIREC_JOGBALL_EN           (88)
#define DESIREC_JOGBALL_EN_XA      (110)

#define DESIREC_GPIO_PROXIMITY_INT_N    (17)
#define DESIREC_GPIO_GSENSOR_INT_N         (18)
#define DESIREC_POWER_KEY                  		(20)
#define DESIREC_GPIO_UP_INT_N              (21)
#define DESIREC_GPIO_PS_HOLD             		(25)
#define DESIREC_GPIO_MDDI_RST_N            (26)
#define DESIREC_GPIO_TP_ATT_N              (36)
#define DESIREC_GPIO_SDMC_CD_N             (38)

/* BT */
#define DESIREC_GPIO_UART1_RTS             (43)
#define DESIREC_GPIO_UART1_CTS             (44)
#define DESIREC_GPIO_UART1_RX              (45)
#define DESIREC_GPIO_UART1_TX              (46)
#define DESIREC_GPIO_WB_SHUT_DOWN_N        (101)

#define DESIREC_GPIO_PROXIMITY_EN          (47)
#define DESIREC_GPIO_CM3602_EN             (48)
#define DESIREC_GPIO_I2C_CLK               (60)
#define DESIREC_GPIO_I2C_DAT               (61)

#define DESIREC_GPIO_UP_RESET_N            (76)
#define DESIREC_GPIO_WAKE_UP              (80)
#define DESIREC_GPIO_COMPASS_INT_N         (83)
#define DESIREC_GPIO_COMPASS_RST_N         (84)
#define DESIREC_PROJECT_NAME          "desirec"
#define DESIREC_LAYOUTS             { \
		{ {  0,  1, 0}, { -1,  0, 0}, {0, 0, 1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, 1} }, \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0, 1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1, 0} }  \
					}

#define DESIREC_GPIO_HTC_HP_SD             (89)

#define DESIREC_GPIO_VCM_PWDN              (91)
#define DESIREC_GPIO_CAM_RST_N             (92)
#define DESIREC_GPIO_USB_PHY_RST_N         (100)
#define DESIREC_GPIO_WIFI_EN               (102)
#define DESIREC_CAM_PWDN                   (107)
#define DESIREC_TP_LS_EN                   (108)
#define DESIREC_GPIO_TP_EN                 (109)

#define DESIREC_GPIO_TO_INT(x)             (x+64)/*from gpio_to_irq*/


/* H2W */
#define DESIREC_GPIO_CABLE_IN1             (49)
#define DESIREC_GPIO_CABLE_IN2             (31)
#define DESIREC_GPIO_UART3_RX              (86)
#define DESIREC_GPIO_UART3_TX              (87)
#define DESIREC_GPIO_H2W_DATA              (86)
#define DESIREC_GPIO_H2W_CLK               (87)
#define DESIREC_GPIO_HEADSET_MIC           (17)
#define DESIREC_GPIO_35MM_HEADSET_DET      (27)
#define DESIREC_GPIO_AUD_EXTMIC_SEL        (82)
#define DESIREC_GPIO_H2W_POWER             (93)

#define DESIREC_GPIO_VSYNC	(97)

int desirec_init_mmc(unsigned int sys_rev);
void config_desirec_camera_on_gpios(void);
void config_desirec_camera_off_gpios(void);
unsigned int camera_is_micron_5M(void);
#endif /* GUARD */
