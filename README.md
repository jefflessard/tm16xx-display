# tm16xx-display
Linux kernel driver for auxiliary displays based on led controllers such as tm16xx family and alike

## Implemented devices
* Shenzhen TITAN MICRO Electronics
  * TM1628 (untested)
  * TM1650 (untested)
* FUDA HISI MICROELECTRONICS
  * FD6551 (tested)

# Installation Instructions

## Prerequisites
* Linux kernel headers installed

## Download
```sh
git clone https://github.com/jefflessard/tm16xx-display.git
```

## Kernel module
1. Build and install module
```sh
make install
```

2. Load module
```sh
modprobe tm16xx
```

3  Check module logs
```sh
dmesg | grep tm16xx
```

## Configure the device tree
You can refer to https://github.com/arthur-liberman/vfd-configurations/ to find your specific device OpenVFD configuration and use the corresponding values.

1. Edit the `overlay.dts` according to your device
  * Option 1 : SPI device
    * `compatible = "spi-gpio"`
    * TBC
  * Option 2 : I2C device
    * `compatible = "i2c-gpio"`
    * `sda-gpios` 
    * `scl-gpios`
  * `led-controller`
    * `compatible`: your display controller chip
  * `led@X,Y` nodes: X=grid index, Y=segment index
    * `reg`: must match <X Y> above
    * 'function`: sysfs name of the led

2. Copy your current dtb file
  * **KEEP A BACKUP OF YOUR CUREENT DTB**
  * Copy to `original.dtb`

3. Decompile your dtb (binary blob) to dts (source)
```sh
make dts
```

4. Compile the device tree binary overlay
```sh
make overlay
```

5. Option 1: Use the `overlay.dtbo` binary overlay directly, if supported

6. Option 2: Merge the overlay with your current dtb
```sh
make mergedtbo
```
the  replace your current dtb with `updated.dtb`

7. Reboot to apply changes

## Install the display service
```sh
./install.sh
```

# Usage

## Customize display service
Just edit the bash script at `/sbin/display-service`

## Restart display service
```sh
systemctl restart display
```

# Manually change the display
See `/sys/class/led/display/` and `/sys/class/led/display::*`
TBC
