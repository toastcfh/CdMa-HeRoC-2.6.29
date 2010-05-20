/* linux/arch/arm/mach-msm/board-heroc-camsensor.h
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
 
#ifndef _DESIREC_CAMSENSOR_
#define _DESIREC_CAMSENSOR_

int desirec_s5k3e2fx_i2c_write(unsigned short waddr, unsigned short wdata);
int desirec_s5k3e2fx_i2c_read(unsigned short u_addr, unsigned short *pu_data);
int desirec_s5k3e2fx_probe_init(void *client);
void desirec_s5k3e2fx_sensor_deinit(void);
int desirec_s5k3e2fx_write_exposuregain(
	uint32_t mode, uint16_t line, uint16_t gain, 
	uint16_t linelengthpck, uint16_t framelengthlines);
int desirec_s5k3e2fx_set_pclk(int rt, int div_adj);
int desirec_s5k3e2fx_sensor_setting(unsigned long arg);
int desirec_s5k3e2fx_late_resume(struct early_suspend *handler);
int desirec_s5k3e2fx_resume(void *client);
int desirec_s5k3e2fx_suspend(void *client,pm_message_t mesg);
int desirec_s5k3e2fx_power_down(void);
int desirec_s5k3e2fx_power_up(void);
int desirec_msm_camio_clk_rate_set(int rate);
int desirec_msm_camio_clk_disable(int clk_type);
int desirec_msm_camio_clk_enable (int clk_type);
int desirec_s5k3e2fx_camif_pad_reg_reset(void);
int desirec_s5k3e2fx_camif_app_reset(void);
void desirec_s5k3e2fx_camif_reset2(void);
int desirec_camif_clk_select(int internal);

#endif
