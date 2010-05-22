obj-y += io.o irq.o timer.o dma.o memory.o
obj-y += devices.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += pmic.o
obj-y += remote_spinlock.o
obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock-arm11.o idle.o

obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += drv_callback.o
obj-y += htc_port_list.o
obj-$(CONFIG_ARCH_QSD8X50) += pmic.o htc_wifi_nvs.o htc_bluetooth.o 

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += smem_log.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_ADSP_COMP) += qdsp5_comp/
obj-$(CONFIG_QSD_AUDIO) += qdsp6/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_HTC_ACOUSTIC_QSD) += htc_acoustic_qsd.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-panel.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-keypad.o fish_battery.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o htc_acoustic.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o board-trout-wifi.o
obj-$(CONFIG_MACH_TROUT) += devices_htc.o

obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-panel.o board-sapphire-keypad.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o board-sapphire-wifi.o
# obj-$(CONFIG_MACH_SAPPHIRE) +=  msm_vibrator.o
# obj-$(CONFIG_MACH_SAPPHIRE) += devices_htc.o

#MSM7200A boards
obj-$(CONFIG_MACH_HERO) += board-hero.o board-hero-panel.o board-hero-keypad.o
obj-$(CONFIG_MACH_HERO) += board-hero-rfkill.o
obj-$(CONFIG_MACH_HERO) += board-hero-mmc.o board-hero-wifi.o htc_wifi_nvs.o htc_awb_cal.o

# MSM7600 boards
obj-$(CONFIG_MACH_DESIREC) += board-desirec.o board-desirec-panel.o board-desirec-keypad.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-rfkill.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-mmc.o board-desirec-wifi.o htc_wifi_nvs.o
obj-$(CONFIG_MACH_HEROC) += board-heroc.o board-heroc-panel.o board-heroc-keypad.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-rfkill.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-mmc.o board-heroc-wifi.o htc_wifi_nvs.o
obj-$(CONFIG_ARCH_MSM7X00A) += msm_vibrator.o devices_htc.o
obj-$(CONFIG_ARCH_MSM_FLASHLIGHT) += msm_flashlight.o
# obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o
# obj-$(CONFIG_TROUT_BATTCHG) += htc_power_supply.o

obj-$(CONFIG_TROUT_BATTCHG) += htc_battery_trout.o
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
# obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
# obj-$(CONFIG_HTC_35MM_JACK) += htc_35mm_jack.o

obj-$(CONFIG_HTC_HEADSET_V1) += htc_headset_v1.o
obj-$(CONFIG_HTC_35MM_REMOTE_V1) += htc_35mm_remote_v1.o
obj-$(CONFIG_HTC_AUDIOJACK_V1) += htc_audio_jack_v1.o
obj-$(CONFIG_HTC_HEADSET) += htc_35mm_remote.o htc_headset_common.o
obj-$(CONFIG_HTC_AUDIOJACK) += htc_audio_jack.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_TOUCHSCREEN_TSSC_MANAGER) += htc_tp_cal.o

