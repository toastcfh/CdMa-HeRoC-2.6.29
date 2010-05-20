/* drivers/usb/function/mtp_tunnel.c
 *
 * Function Driver for MTP
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This code is based in part on the Android ADB driver, which
 * is Copyright (C) 2007 Google, Inc.
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/timer.h>

#include <asm/atomic.h>
#include <linux/uaccess.h>
#include <mach/msm_hsusb.h>

#include "usb_function.h"
#include <linux/usb/cdc.h>

/* please refer: Documentation/ioctl-number.txt and Documentation/ioctl/
 * and choice magic-number */
#define USB_MTP_IOC_MAGIC 0xFF

#define MTP_IOC_GET_CANCEL_REQUEST_ID	_IOR(USB_MTP_IOC_MAGIC, 0x21, __u32)
#define MTP_IOC_SET_DEVICE_STATUS	_IOW(USB_MTP_IOC_MAGIC, 0x26, int)

/* base on Annex. D in PIMA15740-2000 spec */
#define PIMA15740_CANCEL_REQUEST 0x64
#define PIMA15740_GET_EXTENDED_EVENT_DATA 0x65
#define PIMA15740_DEVICE_RESET_REQUEST 0x66
#define PIMA15740_GET_DEVICE_STATUS 0x67

extern uint32_t enabled_functions;
static int mtp_enabled;

static u32 mtp_status_code = 0x2001;
static int cancel_req;
static int garbage_check;
static unsigned int TransactionID;

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_INFO x)
#endif

#define TXN_MAX 16384 /*8192*/

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4

#define MAX_INTR_TX 1

#define MTP_TUNNEL_FUNCTION_NAME "mtp_tunnel"
#define NUM_EVENT	5

enum {
	EVENT_ONLINE = 0,
	EVENT_OFFLINE,
	EVENT_GET_DEVICE_STATUS,
	EVENT_CANCEL_REQUEST,
	EVENT_DEVICE_RESET,
};
/*
struct mtp_event {
	__le16 wEventCode;
	__le32 dwTransactionID;
	__le32 dwParam1;
	__le32 dwParam2;
	__le32 dwParam3;
} __attribute__ ((packed));

struct mtp_event_container {
	__le32 dwLength;
	__le16 wType;
	struct mtp_event data;
} __attribute__ ((packed));
*/
static char *event_string[NUM_EVENT] = {
	"online",
	"offline",
	"Get_Device_Status",
	"Cancel_Request",
	"Device_Reset",
};

struct mtp_tunnel_context {
	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t ioctl_excl;
	atomic_t enable_open_excl;
	atomic_t ctl_open_excl;
	atomic_t ctl_read_excl;
	atomic_t ctl_ioctl_excl;
	atomic_t event_open_excl;
	atomic_t event_excl;
	spinlock_t lock;

	struct usb_endpoint *ep0in;
	struct usb_endpoint *ep0out;
	struct usb_endpoint *intr_in;
	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;
	struct list_head event_queue;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	wait_queue_head_t ctl_read_wq;
	wait_queue_head_t event_wq;
	struct timer_list timer_cancel_req;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	struct usb_request *setup_out_req;
	unsigned char *read_buf;
	unsigned read_count;
	struct platform_device *pdev;
	unsigned ctl_open;
	u8 ioctl_tmp[2];
	u32 event;
};

static struct mtp_tunnel_context _context;

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
void static req_put(struct mtp_tunnel_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct mtp_tunnel_context *ctxt, struct list_head *head)
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

static void mtp_tunnel_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_tunnel_context *ctxt = req->context;
	DBG("%s\n", __func__);

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->tx_idle, req);

	wake_up(&ctxt->write_wq);
}

static void mtp_tunnel_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_tunnel_context *ctxt = req->context;
	DBG("%s: %p status = %d\n", __func__, req, req->status);

	if (req->status != 0) {
		ctxt->error = 1;
		req_put(ctxt, &ctxt->rx_idle, req);
	} else {
		DBG("mtp_tunnel_read complete: %d bytes\n", req->actual);
		req_put(ctxt, &ctxt->rx_done, req);
	}

	wake_up(&ctxt->read_wq);
}

static void mtp_tunnel_complete_notify(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_tunnel_context *ctxt = req->context;
	DBG("%s: status = %x\n", __func__, req->status);

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->event_queue, req);

	wake_up(&ctxt->event_wq);
}

#if 0
static int send_event(struct mtp_event *edata, unsigned length)
{
	struct mtp_tunnel_context *ctxt = &_context;
	struct usb_request *req;
	struct mtp_event_container *event;
	unsigned int ret;

	if (length > sizeof(struct mtp_event))
		return -1;

	req = req_get(ctxt, &ctxt->event_queue);
	if (!req)
		return -1;

	event = req->buf;
	event->dwLength = length + 6;
	event->wType = 0x0004; /* Event */
	memcpy(&event->data, edata, sizeof(struct mtp_event));
	req->length = event->dwLength;

	ret = usb_ept_queue_xfer(ctxt->intr_in, req);
	if (ret) {
		printk(KERN_ERR
		"send_notify_data: cannot queue status request,ret = %d\n",
			ret);
	}
	return 0;
}
#endif
static ssize_t mtp_tunnel_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	struct usb_request *req;
	int r = 0, xfer;
	int ret;

	DBG("mtp_tunnel_read(%d)\n", count);

	if (_lock(&ctxt->read_excl)) {
		printk(KERN_INFO "%s:-EBUSY\n", __func__);
		return -EBUSY;
	}

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("mtp_tunnel_read: waiting for online state\n");
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0) {
			_unlock(&ctxt->read_excl);
			return -EIO;
		}
	}

	while (count > 0) {
		if (ctxt->error) {
			printk(KERN_INFO "%s:-EIO(%d)\n", __func__, -EIO);
			r = -EIO;
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((req = req_get(ctxt, &ctxt->rx_idle))) {
requeue_req:
			req->length = TXN_MAX;
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				printk(KERN_INFO "%s: failed to queue req %p (%d)\n",
					__func__, req, ret);
				r = -EIO;
				ctxt->error = 1;
				req_put(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("%s(): rx %p queue\n", __func__, req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (ctxt->read_count > 0) {
			if (cancel_req) {
				printk(KERN_DEBUG "mtp: discard data(%d) %p\n",
					ctxt->read_count, ctxt->read_req);
				ctxt->read_count = 0;
				req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
				ctxt->read_req = 0;
				continue;
			}

			if (garbage_check) {
				int garbage_bytes;
				if (ctxt->read_count > 512) {
					printk(KERN_DEBUG "mtp: ignore garbage data(%d)\n",
						ctxt->read_count-16);
					garbage_bytes = ctxt->read_count - 16;
					ctxt->read_buf += garbage_bytes;
					ctxt->read_count -= garbage_bytes;
					ctxt->read_count = 16;
				}
				/*
				if (ctxt->read_count == 16) {
					int i;
					printk(KERN_DEBUG "mtp: ");
					for (i = 0; i < 16; i++)
						printk("%02x ", ctxt->read_buf[i]);
					printk("\n");
				}
				*/
				garbage_check = 0;
			}
			xfer = (ctxt->read_count < count) ? ctxt->read_count : count;

			if (copy_to_user(buf, ctxt->read_buf, xfer)) {
				printk(KERN_ERR "%s:-EFAULT\n", __func__);
				r = -EFAULT;
				break;
			}
			ctxt->read_buf += xfer;
			ctxt->read_count -= xfer;
			r += xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_count == 0) {
				req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
				ctxt->read_req = 0;
			}
			DBG("copy %d bytes to user\n", r);
			break;
		}

		/* wait for a request to complete */
		req = 0;
		DBG("%s: wait request to complete\n", __func__);
		ret = wait_event_interruptible(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) ||
						   ctxt->error));

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
			DBG("%s(): rx %p %d\n", __func__, req, req->actual);
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

static ssize_t mtp_tunnel_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("mtp_tunnel_write(%d)\n", count);

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
			r = -EIO;
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
				DBG("mtp_tunnel_write: xfer error %d\n", ret);
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

static int mtp_tunnel_open(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;
	DBG("%s\n", __func__);

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int mtp_tunnel_release(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	_unlock(&ctxt->open_excl);
	return 0;
}

static struct file_operations mtp_tunnel_fops = {
	.owner =   THIS_MODULE,
	.read =    mtp_tunnel_read,
	.write =   mtp_tunnel_write,
	.open =    mtp_tunnel_open,
	.release = mtp_tunnel_release,
};

static struct miscdevice mtp_tunnel_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_tunnel",
	.fops = &mtp_tunnel_fops,
};

static DECLARE_WAIT_QUEUE_HEAD(mtp_tunnel_enable_read_wait);

static int mtp_tunnel_enable_open(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	if (_lock(&ctxt->enable_open_excl))
		return -EBUSY;

	printk(KERN_INFO "%s(): Enabling %s function ###\n",
		__func__, MTP_TUNNEL_FUNCTION_NAME);

	usb_function_switch(1 << USB_FUNCTION_MTP_TUNNEL_NUM);

	return 0;
}

static int mtp_tunnel_enable_release(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	printk(KERN_INFO "%s(): Disabling %s function ###\n",
		__func__, MTP_TUNNEL_FUNCTION_NAME);

	/*
	if (usb_function_switch(3) != 0)
		return -EFAULT;
	ctxt->event = (1 << EVENT_OFFLINE);
	*/
	_unlock(&ctxt->enable_open_excl);
	return 0;
}


static struct file_operations mtp_tunnel_fops_enable = {
	.owner =	THIS_MODULE,
	.open =		mtp_tunnel_enable_open,
	.release =	mtp_tunnel_enable_release,
};

static struct miscdevice mtp_tunnel_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_enable",
	.fops = &mtp_tunnel_fops_enable,
};

static int
mtp_tunnel_ctl_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	struct mtp_tunnel_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;

	if (_lock(&ctxt->ctl_ioctl_excl))
		return -EBUSY;

	if (_IOC_TYPE(cmd) != USB_MTP_IOC_MAGIC) {
		printk(KERN_NOTICE "_IOC_TYPE(cmd) != USB_MTP_IOC_MAGIC\n");
		_unlock(&ctxt->ctl_ioctl_excl);
		return -EINVAL;
	}
	switch (cmd) {
	case MTP_IOC_GET_CANCEL_REQUEST_ID:
		DBG("%s: MTP_IOC_GET_CANCEL_REQUEST_ID\n", __func__);
		if (copy_to_user(argp, &TransactionID, sizeof(__u32))) {
			_unlock(&ctxt->ctl_ioctl_excl);
			printk(KERN_ERR "%s: MTP_IOC_GET_CANCEL_REQUEST_ID error\n", __func__);
			return -EFAULT;
		}
	break;
	case MTP_IOC_SET_DEVICE_STATUS:
		DBG("%s: MTP_IOC_SET_DEVICE_STATUS\n", __func__);
		mtp_status_code = arg;
		DBG("status : 0x%x\n", (unsigned int)mtp_status_code);
		if (cancel_req && mtp_status_code == 0x2001) {
			struct usb_request *req;
			garbage_check = 1;
			cancel_req = 0;
			/* Put a dummy request in rx_done to continue
			 * queuing request */
			req = req_get(ctxt, &ctxt->rx_idle);
			if (req) {
				req->actual = 0;
				req_put(ctxt, &ctxt->rx_done, req);
				wake_up(&ctxt->read_wq);
			}
		}
	break;
	default:
		printk(KERN_NOTICE "%s: default\n", __func__);
		_unlock(&ctxt->ctl_ioctl_excl);
		return -EINVAL;
		break;
	}
	_unlock(&ctxt->ctl_ioctl_excl);
	return 0;
}

static ssize_t mtp_tunnel_ctl_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	int r = 0, n = 0, i;
	unsigned long flags;
	DBG("%s\n", __func__);

	if (_lock(&ctxt->ctl_read_excl))
		return -EBUSY;

	if (!ctxt->ctl_open)
		return -EPERM;

wait_event:
	/*
	ret = wait_event_interruptible(ctxt->ctl_read_wq, (ctxt->online || ctxt->error));
	if (ret < 0) {
		_unlock(&ctxt->ctl_read_excl);
		return ret;
	}
	*/
	if (wait_event_interruptible(ctxt->ctl_read_wq, (ctxt->event ||
		!ctxt->ctl_open)) < 0) {
		_unlock(&ctxt->ctl_read_excl);
		return -EIO;
	}
	if (!ctxt->ctl_open) {
		printk(KERN_DEBUG "%s: device not opened\n", __func__);
		return -EPERM;
	}
	spin_lock_irqsave(&ctxt->lock, flags);
	for (i = 0; i < NUM_EVENT; i++) {
		if (ctxt->event & (1 << i)) {
			ctxt->event &= ~(1 << i);
			printk(KERN_DEBUG "%s: %s\n", __func__,
				event_string[i]);
			n = strlen(event_string[i]);
			r = copy_to_user(buf, event_string[i], n);
			break;
		}
	}
	if (i == NUM_EVENT)
		goto wait_event;
	spin_unlock_irqrestore(&ctxt->lock, flags);

	_unlock(&ctxt->ctl_read_excl);
	return r? -EFAULT:n;
}

static int mtp_tunnel_ctl_open(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;
	if (_lock(&ctxt->ctl_open_excl))
		return -EBUSY;
	ctxt->ctl_open = 1;
	printk(KERN_INFO "%s\n", __func__);

	return 0;
}

static int mtp_tunnel_ctl_release(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;
	printk(KERN_INFO "%s\n", __func__);
	ctxt->ctl_open = 0;
	_unlock(&ctxt->ctl_open_excl);
	return 0;
}

static struct file_operations mtp_tunnel_ctl_fops = {
	.owner =	THIS_MODULE,
	.read =		mtp_tunnel_ctl_read,
	.open =		mtp_tunnel_ctl_open,
	.ioctl =	mtp_tunnel_ctl_ioctl,
	.release =	mtp_tunnel_ctl_release,
};

static struct miscdevice mtp_tunnel_ctl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_control",
	.fops = &mtp_tunnel_ctl_fops,
};

#define MAX_EVENT_SIZE	30
static ssize_t mtp_event_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("mtp_event_write(%d)\n", count);

	if (_lock(&ctxt->event_excl))
		return -EBUSY;

	if (count > MAX_EVENT_SIZE) {
		printk(KERN_INFO "too large event data(%d)\n", count);
		r = -EINVAL;
		goto err;
	}

	ret = wait_event_interruptible(ctxt->event_wq,
			((req = req_get(ctxt, &ctxt->event_queue)) || ctxt->error));
	if (!req) {
		r = -EIO;
		goto err;
	}

	if (copy_from_user(req->buf, buf, count)) {
		r = -EFAULT;
		req_put(ctxt, &ctxt->event_queue, req);
		goto err;
	}
	req->length = count;

	ret = usb_ept_queue_xfer(ctxt->intr_in, req);
	if (ret) {
		printk(KERN_ERR
		"send_notify_data: cannot queue status request,ret = %d\n",
			ret);
		req_put(ctxt, &ctxt->event_queue, req);
		r = -EIO;
	}
err:
	_unlock(&ctxt->event_excl);
	printk("%s: return %d\n", __func__, r);
	return r;
}
static int mtp_event_open(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;
	if (_lock(&ctxt->event_open_excl))
		return -EBUSY;
	printk(KERN_INFO "%s\n", __func__);

	return 0;
}

static int mtp_event_release(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;
	printk(KERN_INFO "%s\n", __func__);
	_unlock(&ctxt->event_open_excl);
	return 0;
}

static struct file_operations mtp_event_fops = {
	.owner =	THIS_MODULE,
	.write =	mtp_event_write,
	.open =		mtp_event_open,
	.release =	mtp_event_release,
};

static struct miscdevice mtp_event_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_event",
	.fops = &mtp_event_fops,
};

static void mtp_tunnel_unbind(void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	struct usb_request *req;

	printk(KERN_DEBUG "mtp_tunnel_unbind()\n");

	while ((req = req_get(ctxt, &ctxt->rx_idle)))
		usb_ept_free_req(ctxt->out, req);

	while ((req = req_get(ctxt, &ctxt->tx_idle)))
		usb_ept_free_req(ctxt->in, req);

	while ((req = req_get(ctxt, &ctxt->event_queue)))
		usb_ept_free_req(ctxt->intr_in, req);

	usb_ept_free_req(ctxt->ep0out, ctxt->setup_out_req);

	ctxt->online = 0;
	ctxt->error = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->ctl_read_wq);
}

static void mtp_tunnel_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	struct usb_request *req;
	int n;
	ctxt->event = 0;
	ctxt->out = ept[0];
	ctxt->in = ept[1];
	ctxt->intr_in = ept[2];
	ctxt->ep0in = ept[3];
	ctxt->ep0out = ept[4];

	printk(KERN_DEBUG "mtp_tunnel_bind() %p, %p\n", ctxt->out, ctxt->in);

	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, TXN_MAX);
		if (req == 0)
			goto fail;
		req->context = ctxt;
		req->complete = mtp_tunnel_complete_out;
		req_put(ctxt, &ctxt->rx_idle, req);
	}

	for (n = 0; n < TX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->in, TXN_MAX);
		if (req == 0)
			goto fail;
		req->context = ctxt;
		req->complete = mtp_tunnel_complete_in;
		req_put(ctxt, &ctxt->tx_idle, req);
	}

	for (n = 0; n < MAX_INTR_TX; n++) {
		req = usb_ept_alloc_req(ctxt->intr_in, 64);
		if (!req)
			goto fail;
		req->context = ctxt;
		req->complete = mtp_tunnel_complete_notify;
		req_put(ctxt, &ctxt->event_queue, req);
	}
	ctxt->setup_out_req = usb_ept_alloc_req(ctxt->ep0out, 64);

	DBG("mtp_tunnel_bind() allocated %d rx and %d tx requests\n",
	     RX_REQ_MAX, TX_REQ_MAX);

	misc_register(&mtp_tunnel_device);
	misc_register(&mtp_tunnel_enable_device);
	misc_register(&mtp_tunnel_ctl_device);
	misc_register(&mtp_event_device);

	return;

fail:
	printk(KERN_WARNING "mtp_tunnel_bind() could not allocate requests\n");
	mtp_tunnel_unbind(ctxt);
}

static void mtp_tunnel_configure(int configured, void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	struct usb_request *req;

	printk(KERN_INFO "%s: %d\n", __func__, configured);

	if (configured) {
		ctxt->online = 1;
		ctxt->error = 0;

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
		if (enabled_functions & (1 << USB_FUNCTION_MTP_TUNNEL_NUM)) {
			printk(KERN_INFO "usb: mtp online\n");
			mtp_enabled = 1;
			ctxt->event = (1 << EVENT_ONLINE);
		}

	} else {
		ctxt->online = 0;
		ctxt->error = 1;
		if (mtp_enabled) {
			printk(KERN_INFO "usb: mtp offline\n");
			mtp_enabled = 0;
			ctxt->event = (1 << EVENT_OFFLINE);
		}
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->ctl_read_wq);
}
/*
static void cancel_req_complete(unsigned long data)
{
	cancel_req = 0;
}
*/

static void mtp_cancel_req_complete(struct usb_endpoint *ept,
							struct usb_request *req)
{
	struct mtp_tunnel_context *ctxt = &_context;
	unsigned long flags;
	unsigned int len = 0;
	DBG("%s, req->status = %d\n",  __func__, req->status);
	/* Status Stage */
	if (req->status == 0) {
		DBG("receive %d bytes\n", req->actual);
		len = req->actual;
		if (req->actual == 6) {
			unsigned int *uint = (unsigned int *)(req->buf + 2);
			TransactionID = *uint;
			DBG("Transaction ID = 0x%x\n", TransactionID);
		}
		req->length = 0;
		req->complete = 0;
		usb_ept_queue_xfer(ctxt->ep0in, req);
	}
	if (len != 6)
		return;
	DBG("%s: Cancel_Request\n", __func__);

	spin_lock_irqsave(&ctxt->lock, flags);
	ctxt->event |= (1 << EVENT_CANCEL_REQUEST);
	spin_unlock_irqrestore(&ctxt->lock, flags);
	wake_up(&ctxt->ctl_read_wq);

	/*
	ctxt->timer_cancel_req.expires = jiffies +
		msecs_to_jiffies(300);
	ctxt->timer_cancel_req.data = 0;
	ctxt->timer_cancel_req.function = cancel_req_complete;
	add_timer(&ctxt->timer_cancel_req);
	*/
}

static int mtp_tunnel_setup(struct usb_ctrlrequest *ctrl, void *buf,
			int len, void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	int value = -EOPNOTSUPP;
	unsigned long flags;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	DBG("%s\n", __func__);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| PIMA15740_DEVICE_RESET_REQUEST:
		DBG("%s(): PIMA15740_DEVICE_RESET_REQUEST\n", __func__);

		spin_lock_irqsave(&ctxt->lock, flags);
		ctxt->event |= (1 << EVENT_DEVICE_RESET);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		wake_up(&ctxt->ctl_read_wq);

		value = 0;
		break;
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| PIMA15740_CANCEL_REQUEST:
	{
		struct usb_request *req = ctxt->setup_out_req;
		DBG("%s(): PIMA15740_CANCEL_REQUEST\n", __func__);
		if (w_length != 6)
			break;

		cancel_req = 1;
		wake_up(&ctxt->read_wq);

		req->length = w_length;
		req->complete = mtp_cancel_req_complete;
		usb_ept_queue_xfer(ctxt->ep0out, req);
		value = DATA_STAGE_REQUIRED;
		break;
	}
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| PIMA15740_GET_DEVICE_STATUS:
		DBG("%s(): PIMA15740_GET_DEVICE_STATUS(%d)\n", __func__, w_length);

		spin_lock_irqsave(&ctxt->lock, flags);
		ctxt->event |= (1 << EVENT_GET_DEVICE_STATUS);
		spin_unlock_irqrestore(&ctxt->lock, flags);

		value = 4;
		*(u8 *)buf = 4;
		*(u8 *)(buf+1) = 0;
		*(u8 *)(buf+3) = 0x20;
		if (cancel_req)
			*(u8 *)(buf+2) = 0x19;
		else
			*(u8 *)(buf+2) = 0x01;
		DBG("cancel_req = %d\n", cancel_req);
		wake_up(&ctxt->ctl_read_wq);
		break;
	}
	/*
	if (value == -EOPNOTSUPP)
		printk(KERN_ERR
			"%s: unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n", __func__,
			ctrl->bRequestType, ctrl->bRequest,
			le16_to_cpu(ctrl->wValue), w_index, w_length);
	*/
	if (value == -ERESTARTSYS)
		printk(KERN_ERR
			"%s: wain_event_interruptible fail...(0x%X)\n", __func__, ctrl->bRequest);
	return value;
}

static struct usb_function usb_func_mtp_tunnel = {
	.bind =		mtp_tunnel_bind,
	.unbind =	mtp_tunnel_unbind,
	.configure =	mtp_tunnel_configure,
	.setup =	mtp_tunnel_setup,

	.name = MTP_TUNNEL_FUNCTION_NAME,
	.context = &_context,

	.ifc_class = 0xFF,
	.ifc_subclass = 0,
	.ifc_protocol = 0,

	.ifc_name = MTP_TUNNEL_FUNCTION_NAME,

	.ifc_ept_count = 3,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN, EPT_INT_IN },

	/* the mtp_tunnel function is only enabled when its driver file is open */
	.disabled = 1,
	.position_bit = USB_FUNCTION_MTP_TUNNEL_NUM,
	.cdc_desc = NULL,
	.ifc_index = STRING_MTP,
};

static int __init mtp_tunnel_init(void)
{
	struct mtp_tunnel_context *ctxt = &_context;
	DBG("mtp_tunnel_init()\n");

	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);
	init_waitqueue_head(&ctxt->ctl_read_wq);
	init_waitqueue_head(&ctxt->event_wq);

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
	atomic_set(&ctxt->ioctl_excl, 0);

	atomic_set(&ctxt->enable_open_excl, 0);
	atomic_set(&ctxt->ctl_open_excl, 0);
	atomic_set(&ctxt->ctl_read_excl, 0);
	atomic_set(&ctxt->ctl_ioctl_excl, 0);

	atomic_set(&ctxt->event_open_excl, 0);
	atomic_set(&ctxt->event_excl, 0);
	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	INIT_LIST_HEAD(&ctxt->event_queue);

	/* init_timer(&ctxt->timer_cancel_req); */

	return usb_function_register(&usb_func_mtp_tunnel);
}

module_init(mtp_tunnel_init);
