# 2012-07-20: File changed by Sony Corporation
GCOV_PROFILE := y
#
# Makefile for the video capture/playback device drivers.
#

ifeq ($(CONFIG_MACH_TXS03),y)
	CFLAGS_ov9726.o += -DENABLE_DPC
	obj-$(CONFIG_VIDEO_OV9726)	+= txs03_ov9726.o
else
	obj-$(CONFIG_VIDEO_OV9726)	+= ov9726.o
endif

obj-y				+= avp/
obj-$(CONFIG_TEGRA_MEDIASERVER)	+= mediaserver/
obj-$(CONFIG_TEGRA_NVAVP)	+= nvavp/
obj-$(CONFIG_TEGRA_DTV)		+= tegra_dtv.o
obj-$(CONFIG_TEGRA_CAMERA)	+= tegra_camera.o
obj-$(CONFIG_VIDEO_AR0832)	+= ar0832_main.o
obj-$(CONFIG_VIDEO_OV5650)	+= ov5650.o
obj-$(CONFIG_VIDEO_OV14810)	+= ov14810.o
obj-$(CONFIG_VIDEO_OV2710)	+= ov2710.o
obj-$(CONFIG_VIDEO_SOC380)	+= soc380.o
obj-$(CONFIG_TORCH_SSL3250A)	+= ssl3250a.o
obj-$(CONFIG_TORCH_TPS61050)	+= tps61050.o
obj-$(CONFIG_VIDEO_SH532U)	+= sh532u.o
obj-$(CONFIG_VIDEO_AD5820)	+= ad5820.o
obj-$(CONFIG_VIDEO_IUS006)	+= ius006.o
obj-$(CONFIG_VIDEO_CMV59DX)	+= cmv59dx.o
obj-$(CONFIG_IOEXPANDER_TC35892)	+= tc35892.o
obj-$(CONFIG_VIDEO_OV8820)	+= ov8820.o ov8820-i2c.o
obj-$(CONFIG_VIDEO_OV8820_FOCUSER)	+= ov8820_focuser.o
