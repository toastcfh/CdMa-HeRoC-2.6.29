/* drivers/usb/function/fsync.c
 *
 * Function Device for the Android ADB Protocol
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
 * base on adb, modify to fsync
 * Author: Anthony_Chang@htc.com
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>

#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/sched.h>
#include "usb_function.h"

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(x)
#endif

/* allocate buffer size to: 16384 byte */
//#define ALLOCATE_16K_BUFF

#ifdef ALLOCATE_16K_BUFF
#define TXN_MAX 16384
#else
#define TXN_MAX 4096

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4
#endif

#define FSYNC_FUNCTION_NAME "fsync"

/* fsync_status in /sys 
 * #define USB_FSYNC_ADD_ATTR_FILE
 */
#ifdef USB_FSYNC_ADD_ATTR_FILE
static char fsync_status_tmp[32] = { 0 };
#endif
struct device fsync_dev;
static void fsync_bind(struct usb_endpoint **ept, void *_ctxt);
static void fsync_unbind(void *_ctxt);
static void fsync_configure(int configured, void *_ctxt);

struct fsync_context
{
	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t enable_excl;
	spinlock_t lock;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;
	struct platform_device *pdev;
	int registered;
};

static struct fsync_context _context;

static struct usb_function usb_func_fsync = {
	.bind = fsync_bind,
	.unbind = fsync_unbind,
	.configure = fsync_configure,

	.name = FSYNC_FUNCTION_NAME,
	.context = &_context,

	.ifc_class = 0xff,
	.ifc_subclass = 0x43,
	.ifc_protocol = 0x01,

	.ifc_name = "fsync",

	.ifc_ept_count = 2,
	.position_bit = USB_FUNCTION_FSYNC_NUM,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },

	/* the fsync function is only enabled when its driver file is open */
	.disabled = 1,
	.cdc_desc = NULL,
	.ifc_num = 1,
	.ifc_index = STRING_FSYNC,
};

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
static void req_put(struct fsync_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct fsync_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

static void fsync_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct fsync_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->tx_idle, req);

	wake_up(&ctxt->write_wq);
}

static void fsync_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct fsync_context *ctxt = req->context;

	if (req->status != 0) {
		ctxt->error = 1;
		req_put(ctxt, &ctxt->rx_idle, req);
	} else {
		req_put(ctxt, &ctxt->rx_done, req);
	}

	wake_up(&ctxt->read_wq);
}

static ssize_t fsync_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct fsync_context *ctxt = &_context;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	DBG("%s(%d)\n", __func__, count);

	if (_lock(&ctxt->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("%s: waiting for online state\n", __func__);
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0) {
			_unlock(&ctxt->read_excl);
			return ret;
		}
	}

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((req = req_get(ctxt, &ctxt->rx_idle))) {
requeue_req:
			req->length = TXN_MAX;
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("%s: failed to queue req %p (%d)\n", __func__, req, ret);
				r = -EIO;
				ctxt->error = 1;
				req_put(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("%s: rx %p queue\n", __func__, req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (ctxt->read_count > 0) {
			xfer = (ctxt->read_count < count) ? ctxt->read_count : count;

			if (copy_to_user(buf, ctxt->read_buf, xfer)) {
				r = -EFAULT;
				break;
			}
			ctxt->read_buf += xfer;
			ctxt->read_count -= xfer;
			buf += xfer;
			count -= xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_count == 0) {
				req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
				ctxt->read_req = 0;
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) || ctxt->error));

		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0)
				goto requeue_req;

			ctxt->read_req = req;
			ctxt->read_count = req->actual;
			ctxt->read_buf = req->buf;
			DBG("rx %p %d\n", req, req->actual);
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

fail:
	_unlock(&ctxt->read_excl);
	return r;
}

static ssize_t fsync_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct fsync_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("%s(%d)\n", __func__, count);

	if (_lock(&ctxt->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(ctxt->write_wq,
					       ((req = req_get(ctxt, &ctxt->tx_idle)) || ctxt->error));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			xfer = count > TXN_MAX ? TXN_MAX : count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ept_queue_xfer(ctxt->in, req);
			if (ret < 0) {
				DBG("%s: xfer error %d\n", __func__, ret);
				ctxt->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}


	if (req)
		req_put(ctxt, &ctxt->tx_idle, req);

	_unlock(&ctxt->write_excl);
	return r;
}

static int fsync_open(struct inode *ip, struct file *fp)
{
	struct fsync_context *ctxt = &_context;

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int fsync_release(struct inode *ip, struct file *fp)
{
	struct fsync_context *ctxt = &_context;

	_unlock(&ctxt->open_excl);
	return 0;
}

static struct file_operations fsync_fops = {
	.owner =   THIS_MODULE,
	.read =    fsync_read,
	.write =   fsync_write,
	.open =    fsync_open,
	.release = fsync_release,
};

static struct miscdevice fsync_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_fsync",
	.fops = &fsync_fops,
};

static int fsync_enable_open(struct inode *ip, struct file *fp)
{
	struct fsync_context *ctxt = &_context;

	if (_lock(&ctxt->enable_excl))
		return -EBUSY;

	printk(KERN_INFO "%s: enabling fsync function\n", __func__);
	usb_function_enable(FSYNC_FUNCTION_NAME, 1);
	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int fsync_enable_release(struct inode *ip, struct file *fp)
{
	struct fsync_context *ctxt = &_context;

	printk(KERN_INFO "%s: disabling fsync function\n", __func__);
	usb_function_enable(FSYNC_FUNCTION_NAME, 0);
	_unlock(&ctxt->enable_excl);
	return 0;
}

static struct file_operations fsync_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    fsync_enable_open,
	.release = fsync_enable_release,
};

static struct miscdevice fsync_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_fsync_en",
	.fops = &fsync_enable_fops,
};

#ifdef USB_FSYNC_ADD_ATTR_FILE
static ssize_t show_fsync_status(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;
	//struct msm_hsusb_platform_data *pdata = dev->platform_data;

	if(strlen(fsync_status_tmp))
		length = sprintf(buf, "%s", fsync_status_tmp); /* dummy */
	else
		length = 0;
	return length;
}

static ssize_t store_fsync_status(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	//struct usb_info *ui = the_usb_info;
	int data_buff_size = (sizeof(fsync_status_tmp)>strlen(buf))?
		strlen(buf):sizeof(fsync_status_tmp);
	int loop_i;

	/* avoid overflow, fsync_status_tmp[32] always is 0x0 */
	if(data_buff_size == 32)
		data_buff_size--;

	for(loop_i = 0; loop_i < data_buff_size; loop_i++)	{
		if(buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) /* 0-9 */
			continue;
		else
		if(buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) /* A-Z */
			continue;
		else
		if(buf[loop_i] >= 0x61 && buf[loop_i] <= 0x7A) /* a-z */
			continue;
		else
		if(buf[loop_i] == 0x0A) /* Line Feed */
			continue;
		else
		{
			printk(KERN_WARNING "%s(): get invaild char (0x%2.2X)\n", __func__, buf[loop_i]);
			return -EINVAL;
		}
	}

	memset(fsync_status_tmp, 0x0, sizeof(fsync_status_tmp));
	strncpy(fsync_status_tmp, buf, data_buff_size);

	return count;
}

static DEVICE_ATTR(fsync_status, 0644,
	show_fsync_status, store_fsync_status);
#endif

static int fsync_probe_reg (struct platform_device *pdev)
{
	struct fsync_context *ctxt = &_context;
	ctxt->pdev = pdev;
	return 0;
}

static struct platform_driver fsync_driver_reg = {
	.probe = fsync_probe_reg,
	.driver = { .name = FSYNC_FUNCTION_NAME, },
};

static void fsync_release_reg (struct device *dev) {}

static struct platform_device fsync_device_reg = {
	.name		= FSYNC_FUNCTION_NAME,
	.id		= -1,
	.dev		= {
		.release	= fsync_release_reg,
	},
};

static void fsync_dev_release (struct device *dev) {}

static void fsync_unbind(void *_ctxt)
{
	struct fsync_context *ctxt = _ctxt;
	struct usb_request *req;

	printk(KERN_INFO "%s()\n", __func__);

	while ((req = req_get(ctxt, &ctxt->rx_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}
	while ((req = req_get(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}
	if (ctxt->registered)	{
#ifdef USB_FSYNC_ADD_ATTR_FILE
		device_remove_file(&fsync_dev, &dev_attr_fsync_status);
#endif
		device_unregister(&fsync_dev);
		ctxt->registered = 0;
	}

	ctxt->online = 0;
	ctxt->error = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static void fsync_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct fsync_context *ctxt = _ctxt;
	struct usb_request *req;
	int ret;
#ifndef ALLOCATE_16K_BUFF
	int n;
#endif

	ctxt->out = ept[0];
	ctxt->in = ept[1];
	ctxt->registered = 0;
	printk(KERN_INFO "%s() %p, %p\n", __func__, ctxt->out, ctxt->in);

#ifndef ALLOCATE_16K_BUFF
	for (n = 0; n < RX_REQ_MAX; n++) {
#else
	{
#endif

		req = usb_ept_alloc_req(ctxt->out, TXN_MAX);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = fsync_complete_out;
		req_put(ctxt, &ctxt->rx_idle, req);
	}


#ifndef ALLOCATE_16K_BUFF
	for (n = 0; n < TX_REQ_MAX; n++) {
#else
	{
#endif
		req = usb_ept_alloc_req(ctxt->in, TXN_MAX);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = fsync_complete_in;
		req_put(ctxt, &ctxt->tx_idle, req);
	}

#ifndef ALLOCATE_16K_BUFF
	printk(KERN_INFO
	       "%s() allocated %d rx and %d tx requests\n",
	       __func__, RX_REQ_MAX, TX_REQ_MAX);
#else
	printk(KERN_INFO
		"%s(): allocated buffer: %d\n", __func__, TXN_MAX);
#endif

	misc_register(&fsync_device);
	misc_register(&fsync_enable_device);
	
	fsync_dev.release = fsync_dev_release;
	fsync_dev.parent = &ctxt->pdev->dev;
	strcpy(fsync_dev.bus_id, "interface");

	ret = device_register(&fsync_dev);
	if (ret != 0) {
		printk(KERN_WARNING "fsync_dev failed to register device: %d\n", ret);
		goto fail;
	}
#ifdef USB_FSYNC_ADD_ATTR_FILE
	ret = device_create_file(&fsync_dev, &dev_attr_fsync_status);
	if (ret != 0) {
		printk(KERN_WARNING "fsync_dev device_create_file failed: %d\n", ret);
		device_unregister(&fsync_dev);
		goto fail;
	}
#endif
	ctxt->registered = 1;

	return;

fail:
	printk(KERN_ERR "%s() could not allocate requests\n", __func__);
	fsync_unbind(ctxt);
}

static void fsync_configure(int configured, void *_ctxt)
{
	struct fsync_context *ctxt = _ctxt;
	struct usb_request *req;

	DBG("%s(): %s\n", __func__, (configured)?"success":"failure");

	if (configured) {
		ctxt->online = 1;

		/* if we have a stale request being read, recycle it */
		ctxt->read_buf = 0;
		ctxt->read_count = 0;
		if (ctxt->read_req) {
			req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
			ctxt->read_req = 0;
		}

		/* retire any completed rx requests from previous session */
		while ((req = req_get(ctxt, &ctxt->rx_done)))
			req_put(ctxt, &ctxt->rx_idle, req);

	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static int __init fsync_init(void)
{
	struct fsync_context *ctxt = &_context;
	int retval = 0;
	DBG("%s()\n", __func__);

	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
	atomic_set(&ctxt->enable_excl, 0);

	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	
	retval = platform_driver_register (&fsync_driver_reg);
	if (retval < 0)
		return retval;
	retval = platform_device_register (&fsync_device_reg);
	if (retval < 0)
		goto err_register_device_1;

	retval = usb_function_register(&usb_func_fsync);
	if (retval < 0)
		goto err_register_device_2;

	return retval;

err_register_device_2:
		platform_device_unregister(&fsync_device_reg);
err_register_device_1:
		platform_driver_unregister(&fsync_driver_reg);
		return retval;

}

module_init(fsync_init);
