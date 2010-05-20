/*
 * Definitions for tpa2018d1 speaker amp chip.
 */
#ifndef TPA2018D1_H
#define TPA2018D1_H

#include <linux/ioctl.h>

#define TPA2018D1_I2C_NAME "tpa2018d1"

struct tpa2018d1_platform_data {
        uint32_t gpio_tpa2018_spk_en;
};

#define TPA2018_IOCTL_MAGIC 'a'
#define TPA2018_SET_CONFIG	_IOW(TPA2018_IOCTL_MAGIC, 0x01,	unsigned)
#define TPA2018_READ_CONFIG	_IOW(TPA2018_IOCTL_MAGIC, 0x02, unsigned)

void set_speaker_amp(int on);

#endif

