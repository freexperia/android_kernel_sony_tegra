# 2012-09-04: File changed by Sony Corporation
#
# Makefile for the kernel mmc core.
#

obj-$(CONFIG_MMC)		+= mmc_core.o
mmc_core-y			:= core.o bus.o host.o \
				   mmc.o mmc_ops.o sd.o sd_ops.o \
				   sdio.o sdio_ops.o sdio_bus.o \
				   sdio_cis.o sdio_io.o sdio_irq.o \
				   quirks.o

mmc_core-$(CONFIG_DEBUG_FS)	+= debugfs.o

ccflags-$(CONFIG_MMC_SDIO_REMOVABLE) = -DMMC_SDIO_REMOVABLE

ifeq ($(CONFIG_MACH_TXS03),y)
CFLAGS_bus.o += -DUNSUPPORT_MMC_CARD
endif