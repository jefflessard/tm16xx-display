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
ORIGINAL_DTB = original.dtb
ORIGINAL_DTS = original.dts
OVERLAY_DTS = overlay.dts
PREPROCESS_DTS = overlay.preprocess.dts
OVERLAY_DTBO = overlay.dtbo
MERGED_DTB = updated.dtb

# Make targets
all: module

module:
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C $(KDIR) M=$(shell pwd) clean
	$(RM) $(PREPROCESS_DTS) $(OVERLAY_DTBO)

install-module: module
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)
	depmod -a

extract-dtb:
	dtc -I fs -O dtb /sys/firmware/devicetree/base -o $(ORIGINAL_DTB)
	dtc -I fs -O dts /sys/firmware/devicetree/base -o $(ORIGINAL_DTS)

overlay:
	$(CPP) -I $(KDIR)/include -undef -x assembler-with-cpp $(OVERLAY_DTS) -o $(PREPROCESS_DTS)
	dtc -I dts -O dtb -i $(ORIGINAL_DTS) $(PREPROCESS_DTS) -o $(OVERLAY_DTBO)

merge-overlay: overlay
	fdtoverlay -i $(ORIGINAL_DTB) -o $(MERGED_DTB) $(OVERLAY_DTBO)

install-service:
	modprobe -a ledtrig_timer ledtrig_netdev tm16xx
	echo "softdep tm16xx pre: ledtrig_timer ledtrig_netdev" > /etc/modprobe.d/tm16xx.conf
	cp display-service /usr/sbin/
	cp display.service /lib/systemd/system/
	systemctl daemon-reload
	systemctl enable display
	systemctl start display

.PHONY: all module install-module extract-dtb overlay merge-overlay install-service clean
