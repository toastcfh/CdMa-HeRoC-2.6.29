#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach-types.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>

#include <linux/miscdevice.h>
//#include <linux/cdev.h>
#include <asm/uaccess.h>

#include <linux/platform_device.h>

#include <mach/msm_tssc.h>

#define MAGIC                   0x81

#define TP_IOC_GETMFG        _IOR(MAGIC, 1, unsigned long)

extern char *get_tp_cal_ram(void);
int tp_get_cal(unsigned char *data)
{
        unsigned char *cal, i;

        cal = get_tp_cal_ram();
	
	    for(i=0; i<32; i++)
	    printk("%2x ", *cal++);
	    printk("\n");

        if (copy_to_user(data, cal, 32))
                return -EFAULT;
        return 0;
}

/* calibration read func */
static int tp_calibration_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
   	unsigned char *cal, i, data1='1';
   	int len;

    printk("tp_calibration_read():\n");
	cal = get_tp_cal_ram();
	
    for(i=0; i<32; i++)
    printk("%2x ", *cal++);
    printk("\n");
	    	
	if (cal) {
		//memcpy((void *)page, (void *) cal, 32+0x40);
		//len = sprintf(page, "%x\n", cal);
		len = sprintf(page, "%c\n", data1);
	}
	else {
	   	printk("%s - no calibration data\n", __FUNCTION__);
		return -EIO;
	}
	return 0;
}

static int tp_calibration_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	printk("%s do nothing\n", __FUNCTION__);
   	return 0;
}

#if 0
/* most content happens here */
static int tp_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
   	int error = 0;
	int retval = 0;

	switch(cmd) {
		case TP_IOC_GETMFG:
			retval = get_tp_cal_ram(req.buf);
			break;	
		default:
			printk("tp: unknown command\n");
			return -ENOTTY;
	};

	return retval;
}

int tp_read(struct file *filp, char *buf, size_t count,loff_t *f_pos)
{

   	unsigned char *cal, i;

    printk("tp_read():\n");
	cal = get_tp_cal_ram();
	
    for(i=0; i<32; i++)
    printk("%2x ", *cal++);
    printk("\n");
    
    if(
    copy_to_user(buf, cal ,count)    
    )
    {
     printk("tp_read():err\n");
    }
    else
    {
      printk("tp_read():ok\n");
    }

    return 0;
}
#endif

/*
 * Display the coordinates of the calibration screen points.
 */
ssize_t calibration_mfg_show(char *buf)
{
   	unsigned char *cal, i;

    typedef struct _TP_CAL_IN_FLASH
    {
        unsigned short x1;
        unsigned short y1;
        unsigned short x2;
        unsigned short y2;
        unsigned short x3;
        unsigned short y3;
        unsigned short x4;
        unsigned short y4;
        unsigned short x5;
        unsigned short y5;
        unsigned long crc;
    }TP_CAL_IN_FLASH, *PTP_CAL_IN_FLASH;
	PTP_CAL_IN_FLASH pTpCal;
	pTpCal = (PTP_CAL_IN_FLASH) get_tp_cal_ram();

    printk("calibration_mfg_show():\n");
	cal = get_tp_cal_ram();
    
    for(i=0; i<32; i++)
    printk("%2x ", *cal++);
    printk("\n");

    printk("calibration_mfg_show(): %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lx\n",
    pTpCal->x1,
    pTpCal->y1,
    pTpCal->x2,
    pTpCal->y2,
    pTpCal->x3,
    pTpCal->y3,
    pTpCal->x4,
    pTpCal->y4,
    pTpCal->x5,
    pTpCal->y5,
    pTpCal->crc);	
        	
	//return sprintf(buf, "%d,",*cal);
	//return sprintf(buf, "%d,",pTpCal->x1);
	#if 0
	int n=sprintf(buf, "%d\n",i);
	printk(buf, "%d\n",i);
	return n;
	#endif
	
	#if 1
    return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lx\n",
    pTpCal->x1,
    pTpCal->y1,
    pTpCal->x2,
    pTpCal->y2,
    pTpCal->x3,
    pTpCal->y3,
    pTpCal->x4,
    pTpCal->y4,
    pTpCal->x5,
    pTpCal->y5,
    pTpCal->crc);	
    #endif
}

/*
 * Store the coordinates of the calibration screen points.
 */
void calibration_mfg_store(const char *buf)
{
    printk("Writing to calibration_mfg is not supported\n");
}

static int tp_open(struct inode *inode, struct file *filp)
{
	printk("tp_open():\n");
	return nonseekable_open(inode, filp);
}

static int tp_release(struct inode *inode, struct file *filp)
{
    printk("tp_release():\n");
	return 0;
}

static struct file_operations tp_fops = {
	.owner		= THIS_MODULE,
//	.ioctl		= tp_ioctl,
	.open		= tp_open,
	.release	= tp_release, 
};

static struct miscdevice tp_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "tp",
	.fops	= &tp_fops,	
};

static int tp_init_module(void)
{
	int rc;
    struct proc_dir_entry   *proc_tp;
    
    printk("tp_init_module():\n");
    printk("TP_IOC_GETMFG=%d\n\n", TP_IOC_GETMFG);
	rc = misc_register(&tp_device);
	if (rc) {
		printk(KERN_ERR "misc_register(): fail\n");
		return rc;
	}
	else
	    printk(KERN_ERR "misc_register(): OK\n");
	
	proc_tp = create_proc_entry("tp_cal", 0644, NULL);
	if (!proc_tp) {
	   	printk(KERN_ERR "tp: alloc calibration error\n");
		rc = PTR_ERR(proc_tp);
		if (misc_deregister(&tp_device))
			printk(KERN_ERR "tp: deregister fail\n");
		return rc;
	}

    proc_tp->read_proc = &tp_calibration_read;
    proc_tp->write_proc = &tp_calibration_write;

	return 0;
}

static void tp_exit_module(void)
{

	if (misc_deregister(&tp_device))
		printk("tp: deregister fail\n");
    else		
        printk("tp_exit_module: OK");		

	//remove_proc_entry("tp_calibration", NULL);
}

module_init(tp_init_module);
module_exit(tp_exit_module);

MODULE_AUTHOR("zion_huang@htc.com");
MODULE_LICENSE("HTC");
