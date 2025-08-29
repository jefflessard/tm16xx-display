# Makefile for building the tm16xx kernel module and updating the device tree

# linedisp module
# if custom initial display value is wanted:
# CCFLAGS += -DCONFIG_PANEL_BOOT_MESSAGE=\\\"boot\\\"
# CCFLAGS += -DCONFIG_PANEL_BOOT_MESSAGE=\\\"\\\"
obj-m += line-display.o

# tm16xx core module
CCFLAGS += -DCONFIG_TM16XX
obj-m += tm16xx.o
tm16xx-objs += tm16xx_core.o

# keypad support
CCFLAGS += -DCONFIG_TM16XX_KEYPAD
tm16xx-objs += tm16xx_keypad.o

# tm16xx-i2c module
CCFLAGS += -DCONFIG_TM16XX_I2C
obj-m += tm16xx_i2c.o

# tm16xx-spi module
CCFLAGS += -DCONFIG_TM16XX_SPI
obj-m += tm16xx_spi.o

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
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -Rf $(RELEASE_DIR)

module-install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)

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
