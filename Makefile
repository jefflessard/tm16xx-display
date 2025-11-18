# Makefile for building the tm16xx kernel module and updating the device tree

# Path to the kernel source tree
KDIR ?= /lib/modules/$(shell uname -r)/build

# Module source directory
MDIR := drivers/auxdisplay

# Build configuration string
# tm16xx core module
CONFIG += CONFIG_TM16XX=m
CCFLAGS += -DCONFIG_TM16XX
# keypad support
CONFIG += CONFIG_TM16XX_KEYPAD=y
CCFLAGS += -DCONFIG_TM16XX_KEYPAD
# tm16xx-i2c module
CONFIG += CONFIG_TM16XX_I2C=m
CCFLAGS += -DCONFIG_TM16XX_I2C
# tm16xx-spi module
CONFIG += CONFIG_TM16XX_SPI=m
# linedisp module
CONFIG += CONFIG_LINEDISP=m
CCFLAGS += -DCONFIG_TM16XX_SPI
# if custom initial display value is wanted:
# CCFLAGS += -DCONFIG_PANEL_BOOT_MESSAGE=\\\"boot\\\"
# CCFLAGS += -DCONFIG_PANEL_BOOT_MESSAGE=\\\"\\\"

# backward compatibility
CCFLAGS += -include $(PWD)/$(MDIR)/tm16xx_compat.h
# needed for linux/timer_types.h (<6.8)
CCFLAGS += -I$(PWD)/include/

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
	make EXTRA_CFLAGS="$(CCFLAGS)" -C $(KDIR) M=$(PWD)/$(MDIR) $(CONFIG) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD)/$(MDIR) $(CONFIG) clean
	rm -Rf $(RELEASE_DIR)

module-install:
	$(MAKE) -C $(KDIR) M=$(PWD)/$(MDIR) $(CONFIG) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)

service-install:
	modprobe tm16xx
	cp display-service /usr/sbin/
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
