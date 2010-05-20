  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000

ifeq ($(CONFIG_ARCH_MSM7X00A),y)
  zreladdr-y            := 0x19208000
params_phys-y           := 0x19200100
initrd_phys-y           := 0x19A00000
  zreladdr-$(CONFIG_MACH_DESIREC)            := 0x11208000
params_phys-$(CONFIG_MACH_DESIREC)           := 0x11200100
initrd_phys-$(CONFIG_MACH_DESIREC)           := 0x11A00000
endif
