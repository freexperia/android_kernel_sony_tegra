# 2012-07-20: File changed by Sony Corporation
GCOV_PROFILE := y

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += ahb.o
obj-y                                   += apbio.o
obj-y                                   += common.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += common-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += common-t3.o
obj-y                                   += io.o
obj-y                                   += irq.o
obj-$(CONFIG_TEGRA_GRHOST)              += syncpt.o
obj-y                                   += clock.o
obj-y                                   += timer.o
ifeq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
obj-y                                   += tegra2_clocks.o
obj-y                                   += timer-t2.o
else
obj-y                                   += tegra3_clocks.o
obj-y                                   += timer-t3.o
endif
obj-y                                   += gpio.o
obj-y                                   += pinmux.o
obj-y                                   += devices.o
obj-y                                   += delay.o
obj-y                                   += powergate.o
obj-y                                   += pm.o
obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
obj-$(CONFIG_PM_SLEEP)                  += pm-irq.o
obj-y                                   += gic.o
obj-y                                   += sleep.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += sleep-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += sleep-t3.o
obj-y                                   += fuse.o
obj-y                                   += kfuse.o
obj-y                                   += csi.o
obj-$(CONFIG_TEGRA_SILICON_PLATFORM)    += tegra_odm_fuses.o
obj-y                                   += i2c_error_recovery.o
obj-$(CONFIG_TEGRA_LEGACY_AUDIO)        += tegra_i2s_audio.o
obj-$(CONFIG_TEGRA_LEGACY_AUDIO)        += tegra_spdif_audio.o
obj-y                                   += mc.o
obj-$(CONFIG_TEGRA_STAT_MON)            += tegra2_statmon.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
obj-$(CONFIG_FIQ)                       += fiq.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o
obj-$(CONFIG_TEGRA_PWM)                 += pwm.o
obj-$(CONFIG_TEGRA_ARB_SEMAPHORE)       += arb_sema.o

ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-y                                   += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += latency_allowance.o
obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
endif
ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_actmon.o
endif
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_emc.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_emc.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += wakeups-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += wakeups-t3.o
obj-y                                   += wakeups.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += pm-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pm-t3.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += pinmux-t2-tables.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pinmux-t3-tables.o
obj-$(CONFIG_LOCAL_TIMERS)              += localtimer.o
obj-$(CONFIG_SMP)                       += platsmp.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
obj-y                                   += headsmp.o
obj-y                                   += reset.o
obj-$(CONFIG_TEGRA_SYSTEM_DMA)          += dma.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpu-tegra3.o
endif
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-y                                   += cpuidle.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += cpuidle-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuidle-t3.o
endif
endif
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_throttle.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_throttle.o
endif
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_thermal.o
obj-$(CONFIG_TEGRA_IOVMM)               += iovmm.o
obj-$(CONFIG_TEGRA_IOVMM_GART)          += iovmm-gart.o
obj-$(CONFIG_TEGRA_IOVMM_SMMU)          += iovmm-smmu.o
obj-$(CONFIG_DEBUG_ICEDCC)              += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL)     += sysfs-cluster.o
ifeq ($(CONFIG_TEGRA_MC_PROFILE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_mc.o
endif
obj-$(CONFIG_SENSORS_TEGRA_TSENSOR)     += tegra3_tsensor.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o
obj-$(CONFIG_TEGRA_USB_MODEM_POWER)     += tegra_usb_modem_power.o
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-kbc.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-panel.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-pinmux.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-pcie.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-power.o

obj-${CONFIG_MACH_PAZ00}                += board-paz00.o
obj-${CONFIG_MACH_PAZ00}                += board-paz00-pinmux.o

obj-${CONFIG_MACH_SEABOARD}             += board-seaboard.o
obj-${CONFIG_MACH_SEABOARD}             += board-seaboard-pinmux.o

obj-${CONFIG_MACH_TRIMSLICE}            += board-trimslice.o
obj-${CONFIG_MACH_TRIMSLICE}            += board-trimslice-pinmux.o

obj-${CONFIG_MACH_P852}                 += p852/

obj-${CONFIG_MACH_VENTANA}              += board-ventana.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-pinmux.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-sdhci.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-power.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-panel.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-sensors.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-memory.o

obj-${CONFIG_MACH_ARUBA}                += board-aruba.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-panel.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-pinmux.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-power.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-sdhci.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-sensors.o

obj-${CONFIG_MACH_WHISTLER}             += board-whistler.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-pinmux.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-sdhci.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-power.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-panel.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-sensors.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-kbc.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-baseband.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-memory.o

obj-${CONFIG_MACH_CARDHU}               += board-cardhu.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-kbc.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-panel.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-pinmux.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-power.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-pm298-power-rails.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-pm299-power-rails.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-sdhci.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-sensors.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-memory.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-powermon.o

obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-panel.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-pinmux.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-sdhci.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-memory.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-power.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-baseband.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-kbc.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-sensors.o

obj-${CONFIG_TEGRA_BB_XMM_POWER}        += baseband-xmm-power.o
obj-${CONFIG_TEGRA_BB_XMM_POWER2}       += baseband-xmm-power2.o

obj-${CONFIG_MACH_NBX02}                += board-nbx02.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-pinmux.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-sdhci.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-power.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-panel.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-sensors.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-kbc.o
obj-${CONFIG_MACH_NBX02}                += board-nbx02-memory.o
obj-${CONFIG_MACH_NBX02}                += wireless_power_control_nbx02.o
obj-${CONFIG_MACH_NBX02}                += sysfs-wirelesspower.o
obj-${CONFIG_MACH_NBX02}                += board-nbx-common.o
obj-${CONFIG_MACH_NBX02}                += a1026.o
obj-${CONFIG_MACH_NBX02}                += suspend_notify.o
ccflags-${CONFIG_MACH_NBX02} += -DNBX_WORKAROUND_DVFS_AP20H_VOLTABLE

obj-${CONFIG_MACH_NBX03}                += board-nbx03.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-pinmux.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-sdhci.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-power.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-panel.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-sensors.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-kbc.o
obj-${CONFIG_MACH_NBX03}                += board-nbx03-memory.o
obj-${CONFIG_MACH_NBX03}                += wireless_power_control_nbx03.o
obj-${CONFIG_MACH_NBX03}                += sysfs-wirelesspower.o
obj-${CONFIG_MACH_NBX03}                += hci-event.o
obj-${CONFIG_MACH_NBX03}                += board-nbx-common.o
obj-${CONFIG_MACH_NBX03}                += a1026.o
ccflags-${CONFIG_MACH_NBX03} += -DNBX_WORKAROUND_DVFS_AP20H_VOLTABLE

obj-${CONFIG_MACH_TXS03}               += board-txs03.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-kbc.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-panel.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-pinmux.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-power.o
#obj-${CONFIG_MACH_TXS03}               += board-txs03-pm298-power-rails.o
#obj-${CONFIG_MACH_TXS03}               += board-txs03-pm299-power-rails.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-sdhci.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-sensors.o
obj-${CONFIG_MACH_TXS03}               += board-txs03-memory.o
#obj-${CONFIG_MACH_TXS03}               += board-txs03-powermon.o
obj-${CONFIG_MACH_TXS03}               += txs_acc_det.o txs_acc_dcout.o txs_acc_5v0_hdmi.o txs_acc_dock.o txs_acc_dockspeaker.o
obj-${CONFIG_MACH_TXS03}               += board-touch.o
obj-${CONFIG_MACH_TXS03}               += board-touch-atmel_maxtouch.o
obj-${CONFIG_MACH_TXS03}               += wireless_power_control_txs03.o
obj-${CONFIG_MACH_TXS03}               += sysfs-wirelesspower.o
obj-${CONFIG_MACH_TXS03}               += hci-event.o
obj-${CONFIG_MACH_TXS03}               += sysfs-cabc.o
obj-${CONFIG_MACH_TXS03}               += board-nbx-common.o
obj-${CONFIG_MACH_TXS03}               += sysfs-throttling.o
ifeq ($(SF_BUILD_WIRELESS_HWCONFIG),3G)
ccflags-${CONFIG_MACH_TXS03} += -DCONFIG_INPUT_SX8636
endif
ifeq ($(CONFIG_MACH_TXS03),y)
CFLAGS_tegra3_thermal.o += -DTXS03_THROTTLE_TEMPERATURE
endif