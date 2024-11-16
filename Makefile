# Makefile for building the tm16xx kernel module and updating the device tree

# Kernel module parameters
MODULE_NAME = tm16xx
obj-m += tm16xx.o
INSTALL_MOD_PATH ?= /

# Path to the kernel source tree
KDIR ?= /lib/modules/$(shell uname -r)/build

# Device Tree parameters
ORIGINAL_DTB = original.dtb

# Build and release directories
RELEASE_DIR = release

# dts cpp preprocessor flags
DTSFLAGS = -I $(KDIR)/include -undef -x assembler-with-cpp

# Make targets

all: module

debug: CCFLAGS += -g -DDEBUG
debug: module

module:
	make EXTRA_CFLAGS="$(CCFLAGS)" -C $(KDIR) M=$(PWD) modules

clean:
	rm -f tm16xx.mod* tm16xx.ko Module.symvers modules.order tm16xx.o .tm* .module* .Module*
	rm -Rf $(RELEASE_DIR)

module-install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)
	depmod -a

service-install:
	modprobe -a ledtrig_timer ledtrig_netdev tm16xx
	cp display-service display-utils /usr/sbin/
	cp display.service /lib/systemd/system/
	systemctl daemon-reload
	systemctl enable display
	systemctl restart display

install: module module-install service-install

$(RELEASE_DIR):
	mkdir -p $(RELEASE_DIR)

extract-dtb:
	dtc -I fs -O dtb /sys/firmware/devicetree/base -o $(ORIGINAL_DTB)

%.dtbo: devices/%.dtso $(RELEASE_DIR)
	$(CPP) -I $(KDIR)/include -I $(PWD) -undef -x assembler-with-cpp -E $< -o /dev/stdout | dtc -I dts -O dtb -o $(RELEASE_DIR)/$@

%.dtb: %.dtbo
	fdtoverlay -i $(ORIGINAL_DTB) $(RELEASE_DIR)/$< -o $(RELEASE_DIR)/$@

.PHONY: module module-install service-install install extract-dtb clean
