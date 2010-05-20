#ifndef _DRIVER_DIAG_H_
#define _DRIVER_DIAG_H_

/* please refer: Documentation/ioctl-number.txt and Documentation/ioctl/
 * and choice magic-number */
#define USB_DIAG_IOC_MAGIC 0xFF

#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)
#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)
#define USB_DIAG_FUNC_IOC_AMR_SET	_IOW(USB_DIAG_IOC_MAGIC, 4, int)

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_DEBUG x)
#endif

#define TXN_MAX 8192
#define RXN_MAX 8192

#define HDLC_MAX 8192
#define ARM9_MAX 8192
#define TABLE_SIZE 10

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4

#define DIAG_FUNCTION_NAME "diag"

struct diag_context {
	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t enable_excl;
	atomic_t open_arm9_excl;
	atomic_t read_arm9_excl;
	atomic_t write_arm9_excl;

	spinlock_t lock;
	spinlock_t lock_reg_num;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;
	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;
	struct list_head tx_qdsp_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	wait_queue_head_t read_arm9_wq;


	smd_channel_t *ch;
	smd_channel_t *chqdsp;
	int in_busy;
#if defined(CONFIG_MSM_N_WAY_SMD)
	int in_busy_qdsp;
#endif
	int isRead;
	int is2ARM11;
	char is7E;
	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;

	unsigned char id_table[TABLE_SIZE];
	unsigned char toARM9_buf[ARM9_MAX];

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;

	struct usb_request *read_arm9_req;
	unsigned char *read_arm9_buf;
	unsigned read_arm9_count;
	struct platform_device *pdev;
	u64 tx_count; /* To ARM9*/
	u64 rx_count; /* From ARM9*/
};

static struct diag_context _context;

struct device diag_device;
enum {
	DIAG_DISABLED = 0,
	DIAG_ENABLED
};
enum data_access {
	data_set_clear = 0,
	data_set_rx,
	data_get_rx,
	data_set_tx,
	data_get_tx,
};

static void smd_try_to_send(struct diag_context *ctxt);
#if defined(CONFIG_MSM_N_WAY_SMD)
static void diag_smd_qdsp_send_req(struct diag_context *ctxt);
#endif
#endif
