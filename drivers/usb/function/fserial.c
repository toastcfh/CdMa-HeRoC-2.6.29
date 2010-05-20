/* drivers/usb/function/fserial.c
 *
 * Function Driver for USB Serial
 *
 * Copyright (C) 2008 htc, Inc.
 * Author: Arec Kao <arec_kao@htc.com>
 *
 * Based heavily on the serial gadget driver in
 * drivers/usb/gadget/serial.c
 * 
 */ 






#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>

#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/list.h>

#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb_usual.h>

#include "usb_function.h"
#include <mach/board_htc.h>

/* Defines */

#define FS_VERSION_STR			"v2.2"
#define FS_LONG_NAME			"HTC Function Serial Interface"

#define FS_SHORT_NAME			"fserial"

#define FS_MAJOR			127
#define FS_MINOR_START			0

#define FS_NUM_PORTS			1
#define FS_NO_CONFIG_ID			0

#define FS_MAX_DESC_LEN			256

#define FS_DEFAULT_READ_Q_SIZE		32
#define FS_DEFAULT_WRITE_Q_SIZE		32

#define FS_DEFAULT_WRITE_BUF_SIZE	8192

#define FS_CLOSE_TIMEOUT		15

#define FS_DEFAULT_DTE_RATE		9600
#define FS_DEFAULT_DATA_BITS		8
#define FS_DEFAULT_PARITY		USB_CDC_NO_PARITY
#define FS_DEFAULT_CHAR_FORMAT		USB_CDC_1_STOP_BITS
#define MAX_PKT					512

/* debug settings */
#ifdef DEBUG
static int debug = 1;
#else
#define	debug 0
#endif

#define fs_debug(format, arg...) \
	do { if (debug) printk(KERN_DEBUG format, ## arg); } while(0)
#define fs_debug_level(level, format, arg...) \
	do { if (debug>=level) printk(KERN_DEBUG format, ## arg); } while(0)


/* Structures */

struct userial_context;

/* circular buffer */
struct fs_buf {
	unsigned int		buf_size;
	char			*buf_buf;
	char			*buf_get;
	char			*buf_put;
};


/* the port structure holds info for each port, one for each minor number */
struct fs_port {
	struct userial_context		*port_dev;	/* pointer to device struct */
	struct tty_struct	*port_tty;	/* pointer to tty struct */
	spinlock_t		port_lock;
	int			port_num;
	int			port_open_count;
	int			port_in_use;	/* open/close in progress */
	wait_queue_head_t	port_write_wait;/* waiting to write */
	struct fs_buf		*port_write_buf;
	struct usb_cdc_line_coding	port_line_coding;
	u16			port_handshake_bits;
};

struct userial_context{

	spinlock_t dev_lock;
	int			dev_config;	/* configuration number */
	struct usb_endpoint *dev_out_ep;
	struct usb_endpoint *dev_in_ep;

	struct list_head	dev_write_list;	/* list of write requests */
	struct list_head    dev_read_list;
	
	struct fs_port		*dev_port[FS_NUM_PORTS]; /* the ports */
	struct platform_device *pdev;
	int registered;
};
static struct userial_context _context;
struct device fserial_dev;

/* Functions */

/* module */
static int __init fs_module_init(void);
static void __exit fs_module_exit(void);

/* tty driver */
static int fs_open(struct tty_struct *tty, struct file *file);
static void fs_close(struct tty_struct *tty, struct file *file);
static int fs_write(struct tty_struct *tty,
	const unsigned char *buf, int count);
static int fs_put_char(struct tty_struct *tty, unsigned char ch);
static void fs_flush_chars(struct tty_struct *tty);
static int fs_write_room(struct tty_struct *tty);
static int fs_chars_in_buffer(struct tty_struct *tty);
static void fs_throttle(struct tty_struct * tty);
static void fs_unthrottle(struct tty_struct * tty);
static int fs_break(struct tty_struct *tty, int break_state);
static int  fs_ioctl(struct tty_struct *tty, struct file *file,
	unsigned int cmd, unsigned long arg);
static void fs_set_termios(struct tty_struct *tty, struct ktermios *old);

static int fs_send(void);
static int fs_send_packet(struct userial_context *dev, char *packet,	unsigned int size);
static int fs_recv_packet(struct userial_context *dev, char *packet,	unsigned int size);
static void fs_read_complete(struct usb_endpoint *ep, struct usb_request *req);
static void fs_write_complete(struct usb_endpoint *ep, struct usb_request *req);

/* gadget driver */
static void fs_bind(struct usb_endpoint **ept, void *_ctxt);
static void fs_unbind(void *_ctxt);
static int fs_setup( struct usb_ctrlrequest *ctrl, void* buf, int len, void *_ctxt);
//static void fs_setup_complete(struct usb_endpoint *ep, struct usb_request *req);
//static void fs_setup_complete_set_line_coding(struct usb_endpoint *ep,
//	struct usb_request *req);
//static void fs_disconnect(struct usb_gadget *gadget);

static void fs_configure(int configure, void *context);
//static void fs_reset_config(struct userial_context* dev);

static int fs_alloc_ports(struct userial_context *ctxt, gfp_t kmalloc_flags);
static void fs_free_ports(struct userial_context *ctxt);

/* circular buffer */
static struct fs_buf *fs_buf_alloc(unsigned int size, gfp_t kmalloc_flags);
static void fs_buf_free(struct fs_buf *gb);
static void fs_buf_clear(struct fs_buf *gb);
static unsigned int fs_buf_data_avail(struct fs_buf *gb);
static unsigned int fs_buf_space_avail(struct fs_buf *gb);
static unsigned int fs_buf_put(struct fs_buf *gb, const char *buf,
	unsigned int count);
static unsigned int fs_buf_get(struct fs_buf *gb, char *buf,
	unsigned int count);

/* external functions */

/* Globals */

static struct mutex fs_open_close_lock[FS_NUM_PORTS];

static unsigned int read_q_size = FS_DEFAULT_READ_Q_SIZE;
static unsigned int write_q_size = FS_DEFAULT_WRITE_Q_SIZE;

static unsigned int write_buf_size = FS_DEFAULT_WRITE_BUF_SIZE;


/* tty driver struct */
static const struct tty_operations fs_tty_ops = {
	.open =			fs_open,
	.close =		fs_close,
	.write =		fs_write,
	.put_char =		fs_put_char,
	.flush_chars =		fs_flush_chars,
	.write_room =		fs_write_room,
	.ioctl =		fs_ioctl,
	.set_termios =		fs_set_termios,
	.throttle =		fs_throttle,
	.unthrottle =		fs_unthrottle,
	.break_ctl =		fs_break,
	.chars_in_buffer =	fs_chars_in_buffer,
};
static struct tty_driver *fs_tty_driver;

/* function driver struct */
static struct usb_function usb_func_userial = {

	.bind = fs_bind,
	.unbind = fs_unbind,
	.configure = fs_configure,
	.setup = fs_setup,

	.name = FS_SHORT_NAME,
	.context = &_context,

	.ifc_class = USB_CLASS_CDC_DATA,
	.ifc_subclass = 0,
	.ifc_protocol = 0,

	.ifc_name = FS_SHORT_NAME,
	
	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },

	.disabled = 1,
	.position_bit = USB_FUNCTION_FSERIAL_NUM,
	.cdc_desc = NULL,
	.ifc_num = 1,
	.ifc_index = STRING_FSERIAL,
};


/* Module */

#ifdef DEBUG
module_param(debug, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging, 0=off, 1=on");
#endif

module_param(read_q_size, uint, S_IRUGO);
MODULE_PARM_DESC(read_q_size, "Read request queue size, default=32");

module_param(write_q_size, uint, S_IRUGO);
MODULE_PARM_DESC(write_q_size, "Write request queue size, default=32");

module_param(write_buf_size, uint, S_IRUGO);
MODULE_PARM_DESC(write_buf_size, "Write buffer size, default=8192");


module_init(fs_module_init);
module_exit(fs_module_exit);

static void fserial_dev_release (struct device *dev) {}

static int fserial_probe (struct platform_device *pdev)
{
	struct userial_context *ctxt = &_context;
	ctxt->pdev = pdev;
	return 0;
}

static struct platform_driver fserial_driver = {
	.probe = fserial_probe,
	.driver = { .name = FS_SHORT_NAME, },
};

static void fserial_release (struct device *dev) {}

static struct platform_device fserial_device = {
	.name		= FS_SHORT_NAME,
	.id		= -1,
	.dev		= {
		.release	= fserial_release,
	},
};

static ssize_t store_fserial_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long ul;
	if ((buf[0] != '0' && buf[0] != '1') && buf[1] != '\n')
	{
		printk(KERN_WARNING "Can't enable/disable fserial %s\n", buf);
		return -EINVAL;
	}
	ul = simple_strtoul(buf, NULL, 10);
	usb_function_enable(FS_SHORT_NAME, ul);
	return count;
}

static ssize_t show_fserial_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int rc;
	rc = sprintf(buf, "%d", return_usb_function_enabled(FS_SHORT_NAME));
	return rc;

}
static DEVICE_ATTR(fserial_enable, 0644, show_fserial_enable, store_fserial_enable);


/*
*  fs_module_init
*
*  Register as a USB gadget driver and a tty driver.
*/
static int __init fs_module_init(void)
{
	int i;
	int retval;
	
	fs_tty_driver = alloc_tty_driver(FS_NUM_PORTS);
	if (!fs_tty_driver)
		return -ENOMEM;
	fs_tty_driver->owner = THIS_MODULE;
	fs_tty_driver->driver_name = FS_SHORT_NAME;
	fs_tty_driver->name = "ttyfs";
	fs_tty_driver->major = FS_MAJOR;
	fs_tty_driver->minor_start = FS_MINOR_START;
	fs_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	fs_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	fs_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	fs_tty_driver->init_termios = tty_std_termios;
	fs_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	fs_tty_driver->init_termios.c_lflag = 0;		
	fs_tty_driver->init_termios.c_iflag = 0;		
	fs_tty_driver->init_termios.c_oflag = 0;		

	tty_set_operations(fs_tty_driver, &fs_tty_ops);

	for (i=0; i < FS_NUM_PORTS; i++)
		mutex_init(&fs_open_close_lock[i]);

	retval = tty_register_driver(fs_tty_driver);
	if (retval) {
		put_tty_driver(fs_tty_driver);
		printk(KERN_ERR "fs_module_init: cannot register tty driver, ret=%d\n", retval);
		return retval;
	}

	retval = platform_driver_register (&fserial_driver);
	if (retval < 0)
		return retval;
	retval = platform_device_register (&fserial_device);
	if (retval < 0)
		goto err_register_device;
#if 0	
	switch  ( board_mfg_mode())
	{
		case 0: //normal mode
			usb_func_userial.disabled = 1;
			break;
		case 1: //factory mode
			usb_func_userial.disabled = 0;
			break;
		case 2: //recovery mode
			usb_func_userial.disabled = 1;
			break;
		case 3: //charge mode
			usb_func_userial.disabled = 1;
			break;
		default:
			usb_func_userial.disabled = 1;
			break;
	}
#endif
	retval = usb_function_register(&usb_func_userial);
	if (retval) {
		printk(KERN_ERR "fs_module_init: cannot register function driver, ret=%d\n", retval);
		return retval;
	}

	printk(KERN_INFO "fs_module_init: %s %s loaded\n", FS_LONG_NAME, FS_VERSION_STR);
	return 0;

err_register_device:
	platform_driver_unregister(&fserial_driver);
	return retval;

}

/*
* fs_module_exit
*
* Unregister as a tty driver.
*/
static void __exit fs_module_exit(void)
{
	tty_unregister_driver(fs_tty_driver);
	put_tty_driver(fs_tty_driver);

	printk(KERN_INFO "fs_module_exit: %s %s unloaded\n", FS_LONG_NAME, FS_VERSION_STR);
}

/* TTY Driver */

/*
 * fs_open
 */
static int fs_open(struct tty_struct *tty, struct file *file)
{
	int port_num;
	unsigned long flags;
	struct fs_port *port;
	struct userial_context *ctxt;
	struct fs_buf *buf;
	struct mutex *mtx;
	int ret;

	port_num = tty->index;

	fs_debug("fs_open: (%d,%p,%p)\n", port_num, tty, file);

	if (port_num < 0 || port_num >= FS_NUM_PORTS) {
		printk(KERN_ERR "fs_open: (%d,%p,%p) invalid port number\n",
			port_num, tty, file);
		return -ENODEV;
	}

	ctxt = &_context;

	if (ctxt == NULL) {
		printk(KERN_ERR "fs_open: (%d,%p,%p) NULL device pointer\n",
			port_num, tty, file);
		return -ENODEV;
	}

	mtx = &fs_open_close_lock[port_num];
	if (mutex_lock_interruptible(mtx)) {
		printk(KERN_ERR
		"fs_open: (%d,%p,%p) interrupted waiting for mutex\n",
			port_num, tty, file);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&ctxt->dev_lock, flags);

	/*if (ctxt->dev_config == FS_NO_CONFIG_ID) {
		printk(KERN_ERR
			"fs_open: (%d,%p,%p) device is not connected\n",
			port_num, tty, file);
		ret = -ENODEV;
		goto exit_unlock_dev;
	}*/

	port = ctxt->dev_port[port_num];

	if (port == NULL) {
		printk(KERN_ERR "fs_open: (%d,%p,%p) NULL port pointer\n",
			port_num, tty, file);
		ret = -ENODEV;
		goto exit_unlock_dev;
	}

	spin_lock(&port->port_lock);
	spin_unlock(&ctxt->dev_lock);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "fs_open: (%d,%p,%p) port disconnected (1)\n",
			port_num, tty, file);
		ret = -EIO;
		goto exit_unlock_port;
	}

	if (port->port_open_count > 0) {
		++port->port_open_count;
		fs_debug("fs_open: (%d,%p,%p) already open\n",
			port_num, tty, file);
		ret = 0;
		goto exit_unlock_port;
	}

	tty->driver_data = NULL;

	/* mark port as in use, we can drop port lock and sleep if necessary */
	port->port_in_use = 1;

	/* allocate write buffer on first open */
	if (port->port_write_buf == NULL) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		buf = fs_buf_alloc(write_buf_size, GFP_KERNEL);
		spin_lock_irqsave(&port->port_lock, flags);

		/* might have been disconnected while asleep, check */
		if (port->port_dev == NULL) {
			printk(KERN_ERR
				"fs_open: (%d,%p,%p) port disconnected (2)\n",
				port_num, tty, file);
			port->port_in_use = 0;
			ret = -EIO;
			goto exit_unlock_port;
		}

		if ((port->port_write_buf=buf) == NULL) {
			printk(KERN_ERR "fs_open: (%d,%p,%p) cannot allocate port write buffer\n",
				port_num, tty, file);
			port->port_in_use = 0;
			ret = -ENOMEM;
			goto exit_unlock_port;
		}

	}

	/* wait for carrier detect (not implemented) */

	/* might have been disconnected while asleep, check */
	if (port->port_dev == NULL) {
		printk(KERN_ERR "fs_open: (%d,%p,%p) port disconnected (3)\n",
			port_num, tty, file);
		port->port_in_use = 0;
		ret = -EIO;
		goto exit_unlock_port;
	}

	tty->driver_data = port;
	port->port_tty = tty;
	port->port_open_count = 1;
	port->port_in_use = 0;

	fs_debug("fs_open: (%d,%p,%p) completed\n", port_num, tty, file);

	ret = 0;

exit_unlock_port:
	spin_unlock_irqrestore(&port->port_lock, flags);
	mutex_unlock(mtx);
	return ret;

exit_unlock_dev:
	spin_unlock_irqrestore(&ctxt->dev_lock, flags);
	mutex_unlock(mtx);
	return ret;

}

/*
 * fs_close
 */

#define FS_WRITE_FINISHED_EVENT_SAFELY(p)			\
({								\
	int cond;						\
								\
	spin_lock_irq(&(p)->port_lock);				\
	cond = !(p)->port_dev || !fs_buf_data_avail((p)->port_write_buf); \
	spin_unlock_irq(&(p)->port_lock);			\
	cond;							\
})

static void fs_close(struct tty_struct *tty, struct file *file)
{
	struct fs_port *port = tty->driver_data;
	struct mutex *mtx;

	if (port == NULL) {
		printk(KERN_ERR "fs_close: NULL port pointer\n");
		return;
	}

	fs_debug("fs_close: (%d,%p,%p)\n", port->port_num, tty, file);

	mtx = &fs_open_close_lock[port->port_num];
	mutex_lock(mtx);

	spin_lock_irq(&port->port_lock);

	if (port->port_open_count == 0) {
		printk(KERN_ERR
			"fs_close: (%d,%p,%p) port is already closed\n",
			port->port_num, tty, file);
		goto exit;
	}

	if (port->port_open_count > 1) {
		--port->port_open_count;
		goto exit;
	}

	/* free disconnected port on final close */
	if (port->port_dev == NULL) {
		kfree(port);
		goto exit;
	}

	/* mark port as closed but in use, we can drop port lock */
	/* and sleep if necessary */
	port->port_in_use = 1;
	port->port_open_count = 0;

	/* wait for write buffer to drain, or */
	/* at most FS_CLOSE_TIMEOUT seconds */
	if (fs_buf_data_avail(port->port_write_buf) > 0) {
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->port_write_wait,
					FS_WRITE_FINISHED_EVENT_SAFELY(port),
					FS_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
	}

	/* free disconnected port on final close */
	/* (might have happened during the above sleep) */
	if (port->port_dev == NULL) {
		kfree(port);
		goto exit;
	}

	fs_buf_clear(port->port_write_buf);

	tty->driver_data = NULL;
	port->port_tty = NULL;
	port->port_in_use = 0;

	fs_debug("fs_close: (%d,%p,%p) completed\n",
		port->port_num, tty, file);

exit:
	spin_unlock_irq(&port->port_lock);
	mutex_unlock(mtx);
}

/*
 * fs_write
 */
static int fs_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	unsigned long flags;
	struct fs_port *port = tty->driver_data;
	int ret;

	if (port == NULL) {
		printk(KERN_ERR "fs_write: NULL port pointer\n");
		return -EIO;
	}

	fs_debug("fs_write: (%d,%p) writing %d bytes\n", port->port_num, tty,
		count);

	if (count == 0)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "fs_write: (%d,%p) port is not connected\n",
			port->port_num, tty);
		ret = -EIO;
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "fs_write: (%d,%p) port is closed\n",
			port->port_num, tty);
		ret = -EBADF;
		goto exit;
	}

	count = fs_buf_put(port->port_write_buf, buf, count);

	spin_unlock_irqrestore(&port->port_lock, flags);

	fs_send();

	fs_debug("fs_write: (%d,%p) wrote %d bytes\n", port->port_num, tty,
		count);

	return count;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
	return ret;
}

/*
 * fs_put_char
 */
static int fs_put_char(struct tty_struct *tty, unsigned char ch)
{
	unsigned long flags;
	int retval = 1;
	struct fs_port *port = tty->driver_data;

	if (port == NULL) {
		printk(KERN_ERR "fs_put_char: NULL port pointer\n");
		return 0;
	}

	fs_debug("fs_put_char: (%d,%p) char=0x%x, called from %p\n",
		port->port_num, tty, ch, __builtin_return_address(0));

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "fs_put_char: (%d,%p) port is not connected\n",
			port->port_num, tty);
		retval = 0;
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "fs_put_char: (%d,%p) port is closed\n",
			port->port_num, tty);
		retval = 0;
		goto exit;
	}

	fs_buf_put(port->port_write_buf, &ch, 1);

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
	return retval;
}

/*
 * fs_flush_chars
 */
static void fs_flush_chars(struct tty_struct *tty)
{
	unsigned long flags;
	struct fs_port *port = tty->driver_data;

	if (port == NULL) {
		printk(KERN_ERR "fs_flush_chars: NULL port pointer\n");
		return;
	}

	fs_debug("fs_flush_chars: (%d,%p)\n", port->port_num, tty);

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR
			"fs_flush_chars: (%d,%p) port is not connected\n",
			port->port_num, tty);
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "fs_flush_chars: (%d,%p) port is closed\n",
			port->port_num, tty);
		goto exit;
	}

	spin_unlock_irqrestore(&port->port_lock, flags);

	fs_send();

	return;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * fs_write_room
 */
static int fs_write_room(struct tty_struct *tty)
{

	int room = 0;
	unsigned long flags;
	struct fs_port *port = tty->driver_data;


	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	&& port->port_write_buf != NULL)
		room = fs_buf_space_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	fs_debug("fs_write_room: (%d,%p) room=%d\n",
		port->port_num, tty, room);

	return room;
}

/*
 * fs_chars_in_buffer
 */
static int fs_chars_in_buffer(struct tty_struct *tty)
{
	int chars = 0;
	unsigned long flags;
	struct fs_port *port = tty->driver_data;

	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	&& port->port_write_buf != NULL)
		chars = fs_buf_data_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	fs_debug("fs_chars_in_buffer: (%d,%p) chars=%d\n",
		port->port_num, tty, chars);

	return chars;
}

/*
 * fs_throttle
 */
static void fs_throttle(struct tty_struct *tty)
{
}

/*
 * fs_unthrottle
 */
static void fs_unthrottle(struct tty_struct *tty)
{
}

/*
 * fs_break
 */
static int fs_break(struct tty_struct *tty, int break_state)
{
	return 0;
}

/*
 * fs_ioctl
 */
static int fs_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct fs_port *port = tty->driver_data;

	if (port == NULL) {
		printk(KERN_ERR "fs_ioctl: NULL port pointer\n");
		return -EIO;
	}

	fs_debug("fs_ioctl: (%d,%p,%p) cmd=0x%4.4x, arg=%lu\n",
		port->port_num, tty, file, cmd, arg);

	/* handle ioctls */

	/* could not handle ioctl */
	return -ENOIOCTLCMD;
}

/*
 * fs_set_termios
 */
static void fs_set_termios(struct tty_struct *tty, struct ktermios *old)
{
}

/*
* fs_send
*
* This function finds available write requests, calls
* fs_send_packet to fill these packets with data, and
* continues until either there are no more write requests
* available or no more data to send.  This function is
* run whenever data arrives or write requests are available.
*/
static int fs_send(void)
{
	int ret,len;
	unsigned long flags;
	struct usb_endpoint *ep;
	struct usb_request *req;
	struct userial_context *ctxt = &_context;

	if (ctxt == NULL) {
		printk(KERN_ERR "fs_send: NULL device pointer\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&ctxt->dev_lock, flags);

	ep = ctxt->dev_in_ep;

	while(!list_empty(&ctxt->dev_write_list)) {

		req = list_first_entry(&ctxt->dev_write_list,
			struct usb_request, list);
		len = fs_send_packet(ctxt, req->buf, MAX_PKT);

		if (len > 0) {
			fs_debug_level(3, "fs_send: len=%d, 0x%2.2x "
					"0x%2.2x 0x%2.2x ...\n", len,
					*((unsigned char *)req->buf),
					*((unsigned char *)req->buf+1),
					*((unsigned char *)req->buf+2));
			list_del(&req->list);
			req->length = len;
			spin_unlock_irqrestore(&ctxt->dev_lock, flags);
			if ((ret=usb_ept_queue_xfer(ep, req))<0) {
				printk(KERN_ERR
				"fs_send: cannot queue read request, ret=%d\n",
					ret);
				spin_lock_irqsave(&ctxt->dev_lock, flags);
				break;
			}
			spin_lock_irqsave(&ctxt->dev_lock, flags);
		} else {
			break;
		}

	}

	spin_unlock_irqrestore(&ctxt->dev_lock, flags);

	return 0;
}

/*
 * fs_send_packet
 *
 * If there is data to send, a packet is built in the given
 * buffer and the size is returned.  If there is no data to
 * send, 0 is returned.  If there is any error a negative
 * error number is returned.
 *
 * Called during USB completion routine, on interrupt time.
 *
 * We assume that disconnect will not happen until all completion
 * routines have completed, so we can assume that the dev_port
 * array does not change during the lifetime of this function.
 */
static int fs_send_packet(struct userial_context *ctxt, char *packet, unsigned int size)
{
	unsigned int len;
	struct fs_port *port;

	/* TEMPORARY -- only port 0 is supported right now */
	port = ctxt->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR
			"fs_send_packet: port=%d, NULL port pointer\n",
			0);
		return -EIO;
	}

	spin_lock(&port->port_lock);

	len = fs_buf_data_avail(port->port_write_buf);
	if (len < size)
		size = len;

	if (size == 0)
		goto exit;

	size = fs_buf_get(port->port_write_buf, packet, size);

	if (port->port_tty)
		wake_up_interruptible(&port->port_tty->write_wait);

exit:
	spin_unlock(&port->port_lock);
	return size;
}

/*
 * fs_recv_packet
 *
 * Called for each USB packet received.  Reads the packet
 * header and stuffs the data in the appropriate tty buffer.
 * Returns 0 if successful, or a negative error number.
 *
 * Called during USB completion routine, on interrupt time.
 *
 * We assume that disconnect will not happen until all completion
 * routines have completed, so we can assume that the dev_port
 * array does not change during the lifetime of this function.
 */
static int fs_recv_packet(struct userial_context *ctxt, char *packet, unsigned int size)
{
	unsigned int len;
	struct fs_port *port;
	int ret;
	struct tty_struct *tty;

	/* TEMPORARY -- only port 0 is supported right now */
	port = ctxt->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR "fs_recv_packet: port=%d, NULL port pointer\n",
			port->port_num);
		return -EIO;
	}

	spin_lock(&port->port_lock);

	if (port->port_open_count == 0) {
		printk(KERN_ERR "fs_recv_packet: port=%d, port is closed\n",
			port->port_num);
		ret = -EIO;
		goto exit;
	}


	tty = port->port_tty;

	if (tty == NULL) {
		printk(KERN_ERR "fs_recv_packet: port=%d, NULL tty pointer\n",
			port->port_num);
		ret = -EIO;
		goto exit;
	}

	if (port->port_tty->magic != TTY_MAGIC) {
		printk(KERN_ERR "fs_recv_packet: port=%d, bad tty magic\n",
			port->port_num);
		ret = -EIO;
		goto exit;
	}

	len = tty_buffer_request_room(tty, size);
	if (len > 0) {
		tty_insert_flip_string(tty, packet, len);
		tty_flip_buffer_push(port->port_tty);
		wake_up_interruptible(&port->port_tty->read_wait);
	}
	ret = 0;
exit:
	spin_unlock(&port->port_lock);
	return ret;
}

/*
* fs_read_complete
*/
static void fs_read_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	int ret;
	struct userial_context *ctxt = &_context;
	fs_debug_level(3, "fs_read complete data:%c %c %c.... len= %u\n", *((char *)req->buf),
																	  *((char *)req->buf+1),
																	  *((char *)req->buf+2), 
																	  req->actual);

	if (ctxt == NULL) {
		printk(KERN_ERR "fs_read_complete: NULL device pointer\n");
		return;
	}

	switch(req->status) {
	case 0:
		/* normal completion */
		fs_recv_packet(ctxt, req->buf, req->actual);
requeue:
		req->length = MAX_PKT;
		if ((ret=usb_ept_queue_xfer(ctxt->dev_out_ep, req))<0) {
			printk(KERN_ERR
			"fs_read_complete: cannot queue read request, ret=%d\n",
				ret);
		}
		break;

	case -ESHUTDOWN:
		/* disconnect */
		fs_debug("fs_read_complete: shutdown\n");
		//fs_free_req(ep, req);
		usb_ept_free_req(ep, req);
		break;
	case -ENODEV:
		{
			unsigned long flags;
			fs_debug_level(3, "fs_read_complete: nodev\n");
			spin_lock_irqsave(&ctxt->dev_lock, flags);
			list_add_tail(&req->list, &ctxt->dev_read_list);
			spin_unlock_irqrestore(&ctxt->dev_lock, flags);
			break;
		}
	default:
		/* unexpected */
		printk(KERN_ERR
		"fs_read_complete: unexpected status error, status=%d\n",
			req->status);
		goto requeue;
		break;
	}
}

/*
* fs_write_complete
*/
static void fs_write_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	struct userial_context *ctxt = &_context;
	fs_debug_level(3, "fs_write_complete:%c %c %c len:%u\n", *((char *)req->buf), 
														   *((char *)req->buf+1),
														   *((char *)req->buf+2),
														   req->actual);
	if (ctxt == NULL) {
		printk(KERN_ERR "fs_write_complete: NULL device pointer\n");
		return;
	}

	switch(req->status) {
	case 0:
		/* normal completion */
requeue:
		if (req == NULL) {
			printk(KERN_ERR
				"fs_write_complete: NULL request pointer\n");
			return;
		}

		spin_lock(&ctxt->dev_lock);
		list_add_tail(&req->list, &ctxt->dev_write_list);
		spin_unlock(&ctxt->dev_lock);

		fs_send();

		break;

	case -ESHUTDOWN:
		/* disconnect */
		fs_debug("fs_write_complete: shutdown\n");
		usb_ept_free_req(ep, req);
		break;

	default:
		printk(KERN_ERR
		"fs_write_complete: unexpected status error, status=%d\n",
			req->status);
		goto requeue;
		break;
	}
}

/* Function Driver */

/*
 * fs_bind
 *
 * Called on module load.  Allocates and initializes the device
 * structure and a control request.
 */
static void fs_bind(struct usb_endpoint **ept, void *_ctxt)
{

	struct userial_context *ctxt = _ctxt;
	struct usb_endpoint *ep;
	struct usb_request *req;
	int i, ret;

	ctxt->registered = 0;
	ctxt->dev_out_ep = ept[0];
	ctxt->dev_in_ep = ept[1];
	spin_lock_init(&ctxt->dev_lock);
	INIT_LIST_HEAD(&ctxt->dev_write_list);
	INIT_LIST_HEAD(&ctxt->dev_read_list);
	if (fs_alloc_ports(ctxt, GFP_KERNEL) != 0) {
		printk(KERN_ERR "fs_bind: cannot allocate ports\n");
		fs_unbind(ctxt);
		goto fail;
	}

	ep = ctxt->dev_out_ep;
	for (i=0; i<read_q_size ; i++) {
		if ((req=usb_ept_alloc_req(ep, 4096))) {
			req->complete = fs_read_complete;
			req->context = ctxt;
			list_add_tail(&req->list, &ctxt->dev_read_list);
		} else {
			printk(KERN_ERR "fs_bind: cannot allocate read requests\n");
			goto fail;
		}
	}

		/* allocate write requests, and put on free list */
	ep = ctxt->dev_in_ep;
	for (i=0; i<write_q_size; i++) {
		if ((req=usb_ept_alloc_req(ep, 4096))) {
			req->complete = fs_write_complete;
			req->context = ctxt;
			list_add_tail(&req->list, &ctxt->dev_write_list);
		} else {
			printk(KERN_ERR "fs_bind: cannot allocate write requests\n");
			goto fail;
		}
	}

	fserial_dev.release = fserial_dev_release;
	fserial_dev.parent = &ctxt->pdev->dev;
	strcpy(fserial_dev.bus_id, "interface");

	ret = device_register(&fserial_dev);
	if (ret != 0) {
		printk(KERN_WARNING "fserial_dev failed to register device: %d\n", ret);
		goto fail;
	}
	ret = device_create_file(&fserial_dev, &dev_attr_fserial_enable);
	if (ret != 0) {
		printk(KERN_WARNING "fserial_dev device_create_file failed: %d\n", ret);
		device_unregister(&fserial_dev);
		goto fail;
	}
	ctxt->registered = 1;

	printk(KERN_ERR "fs_bind: %s bound\n", FS_LONG_NAME);
	return;

fail:
	printk(KERN_ERR "fs_bind() could not allocate requests\n");
}

/*
 * fs_unbind
 *
 * Called on module unload.  Frees the control request and device
 * structure.
 */
static void fs_unbind(void *_ctxt)
{
	
	struct userial_context *ctxt = _ctxt;
	/* read/write requests already freed, only control request remains */
	if (ctxt != NULL){

		fs_free_ports(ctxt);
	/*	if (dev->dev_in_ep)
			usb_ep_disable(dev->dev_in_ep);
		if (dev->dev_out_ep)
			usb_ep_disable(dev->dev_out_ep);
	*/
	if (ctxt->registered)	{
		device_remove_file(&fserial_dev, &dev_attr_fserial_enable);
		device_unregister(&fserial_dev);
		ctxt->registered = 0;
		}
	}
	printk(KERN_INFO "fs_unbind: userial function unbind\n");
}





/*
 * fs_setup
 *
 * Implements all the control endpoint functionality that's not
 * handled in hardware or the hardware driver.
 *
 * Returns the size of the data sent to the host, or a negative
 * error number.
 */
static int fs_setup(struct usb_ctrlrequest *ctrl, void *buf, int len, void *_ctxt)
{
	int ret = -EOPNOTSUPP;
	struct userial_context * ctxt = _ctxt;
	struct fs_port *port = ctxt->dev_port[0];
	printk(KERN_ERR "fs_setup\n");
	
	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		if (ctrl->wIndex != 2)
			return -1;
		switch (ctrl->bRequest) {
		case USB_CDC_REQ_SET_LINE_CODING:
			//if (ctrl->wLength != sizeof(struct usb_cdc_line_coding))
			//	break;
			//ret = ctrl->wLength;
			//req->complete = fs_setup_complete_set_line_coding;//???????????????
			printk(KERN_WARNING "fs_setup: set_line_coding unuspported\n");
			break;

		case USB_CDC_REQ_GET_LINE_CODING:
			ret = min(ctrl->wLength, (u16)sizeof(struct usb_cdc_line_coding));
			if (port) {
				spin_lock(&port->port_lock);
				memcpy(buf, &port->port_line_coding, ret);
				spin_unlock(&port->port_lock);
			}
			break;

		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
			if (ctrl->wLength != 0)
				break;
			ret = 0;
			if (port) {
			/* REVISIT:  we currently just remember this data.
			 * If we change that, update whatever hardware needs
			 * updating.
			 */
				spin_lock(&port->port_lock);
				port->port_handshake_bits = ctrl->wValue;
				spin_unlock(&port->port_lock);
			}
			break;

		default:
			printk(KERN_ERR "fs_setup: unknown class request, "
				"type=%02x, request=%02x, value=%04x, "
				"index=%04x, length=%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			ctrl->wValue, ctrl->wIndex, ctrl->wLength);
			break;
		}
	}
	else
		printk(KERN_ERR "fs_setup: unknown request, "
				"type=%02x, request=%02x, value=%04x, "
				"index=%04x, length=%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			ctrl->wValue, ctrl->wIndex, ctrl->wLength);
	return ret;
	
}

#if 0
static void fs_setup_complete_set_line_coding(struct usb_endpoint *ep,
		struct usb_request *req)
{
	struct fs_port *port = _context.dev_port[0]; /* ACM only has one port */

	switch (req->status) {
	case 0:
		/* normal completion */
		if (req->actual != sizeof(port->port_line_coding))
//			usb_ep_set_halt(ep); ???????????????????
//			ep0_stall();
			printk(KERN_ERR "set_complete_set_line_coding error");
		else if (port) {
			struct usb_cdc_line_coding	*value = req->buf;

			/* REVISIT:  we currently just remember this data.
			 * If we change that, (a) validate it first, then
			 * (b) update whatever hardware needs updating.
			 */
			spin_lock(&port->port_lock);
			port->port_line_coding = *value;
			spin_unlock(&port->port_lock);
		}
		break;

	case -ESHUTDOWN:
		/* disconnect */
		usb_ept_free_req(ep, req);
		break;

	default:
		/* unexpected */
		break;
	}
	return;
}
#endif


/*
 * fs_disconnect
 *
 * Called when the device is disconnected.  Frees the closed
 * ports and disconnects open ports.  Open ports will be freed
 * on close.  Then reallocates the ports for the next connection.
 */


//move disconnect function to fs_close??????
static void fs_disconnect(struct userial_context *ctxt)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->dev_lock, flags);

	//fs_reset_config(ctxt);

	/* free closed ports and disconnect open ports */
	/* (open ports will be freed when closed) */
	fs_free_ports(ctxt);

	/* re-allocate ports for the next connection */
	if (fs_alloc_ports(ctxt, GFP_ATOMIC) != 0)
		printk(KERN_ERR "fs_disconnect: cannot re-allocate ports\n");

	spin_unlock_irqrestore(&ctxt->dev_lock, flags);

	printk(KERN_INFO "fs_disconnect: %s disconnected\n", FS_LONG_NAME);
}


static void fs_configure(int configured, void *context)
{
	struct userial_context * ctxt = context;
	struct usb_endpoint *ep;
	struct usb_request *req, *temp;
	int i = 0;
	fs_debug_level(3,"fs_configure:configure %d\n", configured);
	if (configured){
		ep = ctxt->dev_out_ep;
		list_for_each_entry_safe(req, temp, &ctxt->dev_read_list, list){
			list_del(&req->list);
			req->length = MAX_PKT;
			i++;
			if (usb_ept_queue_xfer(ep, req)<0) {
					printk(KERN_ERR "fs_configure: cannot queue read request\n");
			}
		}
		printk(KERN_ERR "fs_configure queue %d request in read list\n", i);
	}
	else{
		fs_disconnect(ctxt);
	}
	
}

/*
 * fs_reset_config
 *
 * Mark the device as not configured, disable all endpoints,
 * which forces completion of pending I/O and frees queued
 * requests, and free the remaining write requests on the
 * free list.
 *
 * The device lock must be held when calling this function.
 */
#if 0
static void fs_reset_config(struct userial_context* dev)
{
	struct usb_request *req;
	printk(KERN_ERR "in the fs_reset_config\n");
	if (dev == NULL) {
		printk(KERN_ERR "fs_reset_config: NULL device pointer\n");
		return;
	}

	/* free write requests on the free list */
	while(!list_empty(&dev->dev_write_list)) {
		req = list_entry(dev->dev_write_list.next, struct usb_request, list);
		list_del(&req->list);
		usb_ept_free_req(dev->dev_in_ep, req);
	}

	/* disable endpoints, forcing completion of pending i/o; */
	/* completion handlers free their requests in this case */
	//if (dev->dev_in_ep) {
	//	usb_ep_disable(dev->dev_in_ep);
	//	dev->dev_in_ep = NULL;
	//}
	//if (dev->dev_out_ep) {
	//	usb_ep_disable(dev->dev_out_ep);
	//	dev->dev_out_ep = NULL;
	//}
}
#endif

/*
 * fs_alloc_ports
 *
 * Allocate all ports and set the fs_dev struct to point to them.
 * Return 0 if successful, or a negative error number.
 *
 * The device lock is normally held when calling this function.
 */
static int fs_alloc_ports(struct userial_context *ctxt, gfp_t kmalloc_flags)
{
	int i;
	struct fs_port *port;

	if (ctxt == NULL)
		return -EIO;

	for (i=0; i<FS_NUM_PORTS; i++) {
		if ((port=kzalloc(sizeof(struct fs_port), kmalloc_flags)) == NULL)
			return -ENOMEM;

		port->port_dev = ctxt;
		port->port_num = i;
		port->port_line_coding.dwDTERate = cpu_to_le32(FS_DEFAULT_DTE_RATE);
		port->port_line_coding.bCharFormat = FS_DEFAULT_CHAR_FORMAT;
		port->port_line_coding.bParityType = FS_DEFAULT_PARITY;
		port->port_line_coding.bDataBits = FS_DEFAULT_DATA_BITS;
		spin_lock_init(&port->port_lock);
		init_waitqueue_head(&port->port_write_wait);

		ctxt->dev_port[i] = port;
	}

	return 0;
}

/*
 * fs_free_ports
 *
 * Free all closed ports.  Open ports are disconnected by
 * freeing their write buffers, setting their device pointers
 * and the pointers to them in the device to NULL.  These
 * ports will be freed when closed.
 *
 * The device lock is normally held when calling this function.
*/

 static void fs_free_ports(struct userial_context * ctxt)
{
	int i;
	unsigned long flags;
	struct fs_port *port;

	if (ctxt == NULL)
		return;

	for (i=0; i<FS_NUM_PORTS; i++) {
		if ((port=ctxt->dev_port[i]) != NULL) {
			ctxt->dev_port[i] = NULL;

			spin_lock_irqsave(&port->port_lock, flags);

			if (port->port_write_buf != NULL) {
				fs_buf_free(port->port_write_buf);
				port->port_write_buf = NULL;
			}

			if (port->port_open_count > 0 || port->port_in_use) {
				port->port_dev = NULL;
				wake_up_interruptible(&port->port_write_wait);
				if (port->port_tty) {
					wake_up_interruptible(&port->port_tty->read_wait);
					wake_up_interruptible(&port->port_tty->write_wait);
				}
				spin_unlock_irqrestore(&port->port_lock, flags);
			} else {
				spin_unlock_irqrestore(&port->port_lock, flags);
				kfree(port);
			}

		}
	}
}

/* Circular Buffer */

/*
 * fs_buf_alloc
 *
 * Allocate a circular buffer and all associated memory.
 */
static struct fs_buf *fs_buf_alloc(unsigned int size, gfp_t kmalloc_flags)
{
	struct fs_buf *gb;

	if (size == 0)
		return NULL;

	gb = kmalloc(sizeof(struct fs_buf), kmalloc_flags);
	if (gb == NULL)
		return NULL;

	gb->buf_buf = kmalloc(size, kmalloc_flags);
	if (gb->buf_buf == NULL) {
		kfree(gb);
		return NULL;
	}

	gb->buf_size = size;
	gb->buf_get = gb->buf_put = gb->buf_buf;

	return gb;
}

/*
 * fs_buf_free
 *
 * Free the buffer and all associated memory.
 */
static void fs_buf_free(struct fs_buf *gb)
{
	if (gb) {
		kfree(gb->buf_buf);
		kfree(gb);
	}
}

/*
 * fs_buf_clear
 *
 * Clear out all data in the circular buffer.
 */
static void fs_buf_clear(struct fs_buf *gb)
{
	if (gb != NULL)
		gb->buf_get = gb->buf_put;
		/* equivalent to a get of all data available */
}

/*
 * fs_buf_data_avail
 *
 * Return the number of bytes of data available in the circular
 * buffer.
 */
static unsigned int fs_buf_data_avail(struct fs_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_put - gb->buf_get) % gb->buf_size;
	else
		return 0;
}

/*
 * fs_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */
static unsigned int fs_buf_space_avail(struct fs_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_get - gb->buf_put - 1) % gb->buf_size;
	else
		return 0;
}

/*
 * fs_buf_put
 *
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 *
 * Return the number of bytes copied.
 */
static unsigned int
fs_buf_put(struct fs_buf *gb, const char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len  = fs_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf+len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else /* count == len */
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

/*
 * fs_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */
static unsigned int
fs_buf_get(struct fs_buf *gb, char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len = fs_buf_data_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf+len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else /* count == len */
			gb->buf_get = gb->buf_buf;
	}

	return count;
}
