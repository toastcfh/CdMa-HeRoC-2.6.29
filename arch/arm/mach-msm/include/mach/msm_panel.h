#ifndef _MSM_PANEL_H_
#define _MSM_PANEL_H_

struct panel_platform_data {
	struct resource *fb_res;
	int (*power)(int on);
	int (*gpio_switch)(int on);
};
#endif
