# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_INRRTHAT)		+= INR_RTHAT.o
INR_RTHAT-objs +=rt_hat.o 
obj-$(CONFIG_INRRTHAT) 		+= INR_spi.o
INR_spi-objs +=realtime.o
INR_spi-objs +=spi.o
INR_spi-objs +=rt_procfs.o 
ifdef CONFIG_INRRTHAT
obj-$(CONFIG_FIXED_PHY)		+= rt_hat_bdinfo.o
endif
