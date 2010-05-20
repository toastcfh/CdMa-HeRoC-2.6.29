/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/i2c.h>
#include <linux/mfd/tps65023.h>
#include <linux/delay.h>
#if 1
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/highmem.h>
#include <linux/smp_lock.h>
#include <asm/mmu_context.h>
#include <linux/interrupt.h>
#include <linux/capability.h>
#include <linux/completion.h>
#include <linux/kernel_stat.h>
#include <linux/debug_locks.h>
#include <linux/security.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/freezer.h>
#include <linux/vmalloc.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/pid_namespace.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/timer.h>
#include <linux/rcupdate.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/percpu.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysctl.h>
#include <linux/syscalls.h>
#include <linux/times.h>
#include <linux/tsacct_kern.h>
#include <linux/kprobes.h>
#include <linux/delayacct.h>
#include <linux/reciprocal_div.h>
#include <linux/unistd.h>
#include <linux/pagemap.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/bootmem.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/ftrace.h>
#include <trace/sched.h>

#include <asm/tlb.h>
#include <asm/irq_regs.h>

#endif

/* TPS65023_registers */
#define TPS65023_VERSION	0
#define TPS65023_PGOODZ		1
#define TPS65023_MASK		2
#define TPS65023_REG_CTRL	3
#define TPS65023_CON_CTRL	4
#define TPS65023_CON_CTRL2	5
#define TPS65023_DEFCORE	6
#define TPS65023_DEFSLEW	7
#define TPS65023_LDO_CTRL	8
#define TPS65023_MAX		9

static struct i2c_client *tpsclient;

#define TPS65023_RETRY_COUNT 12
static int TPSI2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
		 .addr = tpsclient->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = tpsclient->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < TPS65023_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(tpsclient->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= TPS65023_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n", __func__, TPS65023_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}

static int TPSI2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = tpsclient->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < TPS65023_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(tpsclient->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= TPS65023_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n", __func__, TPS65023_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}
int tps65023_set_dcdc1_level(int mvolts)
{
	int val;
	int ret;
    char buffer[3];

	if (!tpsclient)
		return -ENODEV;

	if (mvolts < 800 || mvolts > 1600)
		return -EINVAL;

	if (mvolts == 1600)
		val = 0x1F;
	else
		val = ((mvolts - 800)/25) & 0x1F;

#if 1
    buffer[0] = TPS65023_DEFCORE;
    buffer[1] = val;
	ret = TPSI2C_TxData(buffer, 2);

	if (!ret) {
        buffer[0] = TPS65023_CON_CTRL2;
        buffer[1] = 0x80;
        ret = TPSI2C_TxData(buffer, 2);
    }
#else
	ret = i2c_smbus_write_byte_data(tpsclient, TPS65023_DEFCORE, val);

	if (!ret)
		ret = i2c_smbus_write_byte_data(tpsclient,
				TPS65023_CON_CTRL2, 0x80);
#endif

	return ret;
}
EXPORT_SYMBOL(tps65023_set_dcdc1_level);

int tps65023_get_dcdc1_level(int *mvolts)
{
	int val;
	int ret;
    char buffer[3];

	if (!tpsclient)
		return -ENODEV;

#if 1
	buffer[0] = TPS65023_DEFCORE;
	ret = TPSI2C_RxData(buffer, 3);
    if (ret) {
		printk(KERN_ERR "%s TPSI2C fail\n", __func__);
        return ret;
    }
    val = buffer[0] & 0x1F;
#else
	val = i2c_smbus_read_byte_data(tpsclient, TPS65023_DEFCORE) & 0x1F;
#endif

	if (val == 0x1F)
		*mvolts = 1600;
	else
		*mvolts = (val * 25) + 800;
	return 0;
}
EXPORT_SYMBOL(tps65023_get_dcdc1_level);

static int tps65023_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
#if 1
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C)) {
		printk(KERN_ERR "TPS65023 does not support SMBUS_BYTE_DATA.\n");
		return -EINVAL;
	}
#else
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "TPS65023 does not support SMBUS_BYTE_DATA.\n");
		return -EINVAL;
	}
#endif

	tpsclient = client;
	printk(KERN_INFO "TPS65023: PMIC probed.\n");
	return 0;
}

static int __devexit tps65023_remove(struct i2c_client *client)
{
	tpsclient = NULL;
	return 0;
}

static const struct i2c_device_id tps65023_id[] = {
	{ "tps65023", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps65023_id);

static struct i2c_driver tps65023_driver = {
	.driver = {
		.name   = "tps65023",
		.owner  = THIS_MODULE,
	},
	.probe  = tps65023_probe,
	.remove = __devexit_p(tps65023_remove),
	.id_table = tps65023_id,
};

static int __init tps65023_init(void)
{
	return i2c_add_driver(&tps65023_driver);
}


static void __exit tps65023_exit(void)
{
	i2c_del_driver(&tps65023_driver);
}

module_init(tps65023_init);
module_exit(tps65023_exit);
