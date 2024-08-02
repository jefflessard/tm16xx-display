# tm16xx-display
Linux kernel driver for auxiliary displays based on led controllers such as tm16xx family and alike

## Implemented devices
* Shenzhen TITAN MICRO Electronics
  * TM1628 (FD628 compatible, untested)
  * TM1650 (FD650 compaatible, untested)
* FUDA HISI MICROELECTRONICS
  * FD6551 (tested)

# Installation Instructions

## Prerequisites
* Linux kernel headers installed
```sh
apt-get install "linux-headers-$(uname -r | sed -E 's/^[^-]+-//')"
```

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

3. Check module logs
```sh
dmesg | grep tm16xx
```

## Configure the device tree
:warning: **KEEP A BACKUP OF YOUR CUREENT DTB**

You can refer to https://github.com/arthur-liberman/vfd-configurations/ to find your specific device OpenVFD configuration and use the corresponding values.

1. Edit the `overlay.dts` according to your device
  * `display-client`
    * Option 1 : SPI device
      * `compatible = "spi-gpio"`
      * `mosi-gpios`: data gpio pin
      * `gpio-sck`: clock gpio pin
      * `cs-gpios`: chip select gpio pin
    * Option 2 : I2C device
      * `compatible = "i2c-gpio"`
      * `sda-gpios`: data gpio pin
      * `scl-gpios`: clock gpio pin
  * `display-controller`
    * `compatible`: your display controller chip
    * `titan,digits`: variable lengh byte array determining the number of text grid cells and their index position 
    * `titan,segment-mapping`: array of 7 bytes specifying which bit of a grid digit should be used for each ascii map segment
  * `led@X,Y`
    * X: grid cell index
    * Y: segment index
    * `reg`: must match `<X Y>` above
    * `function`: sysfs name of the led

2. Copy your current dtb file to `original.dtb`

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

## Customize display from shell
```sh
# turn on display
cat /sys/class/leds/display/max_brightness > /sys/class/leds/display/brightness

# dim brightness display (devices may not implement this)
# value between 1 and max_brightness (usually 8)
echo 1 > /sys/class/leds/display/brightness

# turn off display
echo 0 > /sys/class/leds/display/brightness

# write text on the display (supports 7-segment ascii mapping)
echo "boot" > /sys/class/leds/display/display_value

# clear the display text
echo > /sys/class/leds/display/display_value

# list available leds/symbols
ls /sys/class/leds/display\:\:*

# turn on a specific led/symbol
echo 1 > /sys/class/leds/display\:\:lan/brightness

# turn off a specific led/symbol
echo 1 > /sys/class/leds/display\:\:lan/brightness

# automatically turn on/off usb led when usb device is connected on a specific port
echo usbport > /sys/class/leds/display::usb/trigger
echo 1 > /sys/class/leds/display::usb/ports/usb1-port1

# turn on led on wifi connect + blink on activity (requires ledtrig-netdev module)
echo netdev > /sys/class/leds/display::wlan/trigger
echo wlan0 > /sys/class/leds/display::wlan/device_name
echo 1 > /sys/class/leds/display::wlan/link
echo 1 > /sys/class/leds/display::wlan/rx
echo 1 > /sys/class/leds/display::wlan/tx
```
