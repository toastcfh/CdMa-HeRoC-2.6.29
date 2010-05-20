#ifndef _MSM_HDMI_H_
#define _MSM_HDMI_H_

struct hdmi_platform_data {
	struct resource hdmi_res;
	/* power on hdmi chip */
	int (*power)(int on); /* mandatory */
	void (*hdmi_gpio_on)(void); /* optional */
	void (*hdmi_gpio_off)(void); /* optional */
};
#endif
