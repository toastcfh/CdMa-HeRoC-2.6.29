/* arch/arm/mach-msm/include/mach/htc_35mm_remote.h
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

#ifndef HTC_35MM_REMOTE_H
#define HTC_35MM_REMOTE_H

/* notify the 3.5mm driver of events */
int htc_35mm_remote_notify_insert_ext_headset(int insert);
int htc_35mm_remote_notify_microp_ready(void);
int htc_35mm_remote_notify_button_status(int key_level);
int htc_35mm_remote_notify_irq_enable(int enable);

#endif

