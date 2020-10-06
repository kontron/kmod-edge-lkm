#!/bin/bash

CONFIG_MODULE_SIG=y
CONFIG_MODULE_SIG_ALL=y
GIT_COMMIT := "n/a"

ifeq ($(KDIR),)
KDIR := $(KERNEL_SRC)
endif
ifeq ($(KDIR),)
KDIR := /lib/modules/$(shell uname -r)/build
endif

ifndef target
  target := pcie
  $(info Defaulting to PCIe host interface driver)
endif

edge_lkm_mod   := edgx_lkm

obj-m += $(edge_lkm_mod).o

EXTRA_CFLAGS=-I$(PWD)

$(edge_lkm_mod)-objs := \
		   edge_ac.o                 \
		   edge_br_fdb.o             \
		   edge_br_vlan.o            \
		   edge_bridge.o             \
		   edge_com.o                \
		   edge_com_dma.o            \
		   edge_com_xmii.o           \
		   edge_com_ts.o             \
		   edge_link.o               \
		   edge_preempt.o            \
		   edge_psfp.o               \
		   edge_sched.o              \
		   edge_sched_hw.o           \
		   edge_stat.o               \
		   edge_fqtss.o              \
		   edge_br_sid.o             \
		   edge_frer.o               \
		   edge_mdio.o               \
		   edge_port.o               \
		   edge_time.o               \
		   tsnic_vpd.o               \

ifeq ($(target),platform)
$(edge_lkm_mod)-objs += edge_platform.o
endif

ifeq ($(target),pcie)
$(edge_lkm_mod)-objs += edge_pcie.o
endif

ifdef CONFIG_GPIOLIB
$(edge_lkm_mod)-objs += altera_pio.o
endif

all :
	@echo Building via $(KDIR)
	KCPPFLAGS="-D_EDGX_GIT="$(GIT_COMMIT) $(MAKE) -C $(KDIR) M=$(PWD) modules

modules_install :
	@echo Building via $(KDIR)
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

coccicheck:
	@echo Coccichecking via $(KDIR)
	$(MAKE) -C $(KDIR) M=$(PWD) MODE=report coccicheck

rebuild : clean all

clean :
	@echo Cleaning
	$(MAKE) -C $(KDIR) M=$(PWD) clean
