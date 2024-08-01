# Makefile for building the tm16xx kernel module and updating the device tree

# Kernel module parameters
obj-m += tm16xx.o

# Path to the kernel source tree
KDIR ?= /lib/modules/$(shell uname -r)/build

debug: CFLAGS += -g -DDEBUG

CC += ${CFLAGS}

# Module name
MODULE_NAME = tm16xx

# Installation path for the module
INSTALL_MOD_PATH ?= /

# Device Tree parameters
DTB_FILE = meson-gxl-s905x-p212.dtb
ORIGINAL_DTS = meson-gxl-s905x-p212.dts
OVERLAY_DTS = overlay.dts
PREPROCESS_DTS = overlay.preprocess.dts
OVERLAY_DTBO = overlay.dtbo
NEW_DTB = /boot/dtb/amlogic/meson-gxl-s905x-p212.dtb

# Make targets
all: module

module:
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C $(KDIR) M=$(shell pwd) clean
	$(RM) $(PREPROCESS_DTS) $(OVERLAY_DTBO)

install: module
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)
	depmod -a

dtbo:
	$(CPP) -I $(KDIR)/include -undef -x assembler-with-cpp $(OVERLAY_DTS) -o $(PREPROCESS_DTS)
	dtc -I dts -O dtb -i $(ORIGINAL_DTS) $(PREPROCESS_DTS) -o $(OVERLAY_DTBO)

mergedtb: dtbo
	fdtoverlay -i $(DTB_FILE) -o $(NEW_DTB) $(OVERLAY_DTBO)

.PHONY: all module clean install dtbo mergedtb
