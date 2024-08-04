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
armbian-config
```
Then go to Software -> Headers

## Download
```sh
git clone https://github.com/jefflessard/tm16xx-display.git
```

## Configure the device tree
:warning: **KEEP A BACKUP OF YOUR CUREENT DTB**

You can refer to https://github.com/arthur-liberman/vfd-configurations/ to find your specific device OpenVFD configuration and use the corresponding values.

1. Edit `display.dtso` according to your device
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

2. If needed, edit `t95-display.dtsi` (or create a new one a change the /include/ in `display.dtso`)
  * `display-controller`
    * `titan,digits`: variable lengh byte array determining the number of text grid digits and their index position 
    * `titan,segment-mapping`: array of 7 bytes specifying which bit of a grid digit should be used for each ascii map segment
  * `led@X,Y`
    * X: grid cell index
    * Y: segment index
    * `reg`: must match `<X Y>` above
    * `function`: sets the sysfs name of the led

3. Update your dtb
  * Option 1: Use device tree overlay, if supported
```sh
# This will create the overlay in release/display.dtbo
make display.dtbo 
```

  * Option 2: Create an updated dtb
    * Copy your current dtb file to `original.dtb`:
```sh
# run this command only once.
# we must always start from the
# original dtb when merging overlay
make extract-dtb ORIGINAL_DTB=original.dtb
```

    * Merge the display dtb overlay with your current dtb
```sh
# This will create the dtb in release/display.dtb
make display.dtb ORIGINAL_DTB=original.dtb

# Replace your current dtb with the new dtb, for example:
#cp release/display.dtb /boot/dtb/{YOUR_DTB_PATH}.dtb
```

4. Reboot to apply changes
```sh
reboot
```

## Kernel module and display service
Builds then installs module and service
```sh
make install
```

# Usage

## Restart display service
```sh
systemctl restart display
```

## Customize display service
Just edit the bash script at `/sbin/display-service`

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
