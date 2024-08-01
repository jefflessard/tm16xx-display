// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Titan Micro Electronics TM16XX LED display driver chips
 *
 * Copyright (C) 2024 Jean-François Lessard
 *
 * This driver supports TM16XX family chips, including TM1628 and TM1650.
 * It provides support for both I2C and SPI interfaces.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/map_to_7segment.h>

#define TM16XX_DRIVER_NAME "tm16xx"
#define TM16XX_DEVICE_NAME "display"

struct tm16xx_display;

struct tm16xx_chip_info {
	u8 cmd_init;
	u8 cmd_write_mode;
	u8 cmd_base_addr;
	u8 (*brightness_map)(int brightness);
	u8 max_brightness;
};

struct tm16xx_led {
	struct led_classdev cdev;
	u8 grid;
	u8 segment;
};

struct tm16xx_digit {
	u8 grid;
	char value;
};

struct tm16xx_display {
	struct device *dev;
	const struct tm16xx_chip_info *chip_info;
	union {
		struct i2c_client *i2c;
		struct spi_device *spi;
	} client;
	struct led_classdev main_led;
	struct tm16xx_led *leds;
	int num_leds;
	struct tm16xx_digit *digits;
	int num_digits;
	u8 *segment_mapping;
	int num_segments;
	u8 *display_data;
	size_t display_data_len;
	struct mutex lock;
	int (*client_write)(struct tm16xx_display *display, u8 *data, size_t len);
};

static u8 tm1628_brightness_map(int i){
	static const u8 ON_FLAG = 1<<3, BR_MASK = 7, BR_SHIFT = 0, CMD_FLAG = 1<<7;
	return CMD_FLAG | ((i && 1) * (((i-1) & BR_MASK) << BR_SHIFT | ON_FLAG));
}

static u8 tm1650_brightness_map(int i){
	static const u8 ON_FLAG = 1, BR_MASK = 7, BR_SHIFT = 4, SEG7_FLAG = 1<<3;
	return (i && 1) * ((i & BR_MASK) << BR_SHIFT | SEG7_FLAG | ON_FLAG);
}

static u8 fd6551_brightness_map(int i){
	static const u8 ON_FLAG = 1, BR_MASK = 7, BR_SHIFT = 1;
	return (i && 1) * ((~(i-1) & BR_MASK) << BR_SHIFT | ON_FLAG);
}

static const struct tm16xx_chip_info tm1628_chip_info = {
	.cmd_init = 1<<1|1,
	.cmd_write_mode = 0x40,
	//.cmd_read_mode = 0x40 | 1 << 2,
	.cmd_base_addr = 0xC0,
	.brightness_map = tm1628_brightness_map,
	.max_brightness = 8,
};

static const struct tm16xx_chip_info tm1650_chip_info = {
	.cmd_init = 0,
	.cmd_write_mode = 0x48,
	//.cmd_read_mode = 0x48 | 1,
	.cmd_base_addr = 0x68,
	.brightness_map = tm1650_brightness_map,
	.max_brightness = 8,
};

static const struct tm16xx_chip_info fd6551_chip_info = {
	.cmd_init = 0,
	.cmd_write_mode = 0x48,
	//.cmd_read_mode = 0x48 | 1,
	.cmd_base_addr = 0x66,
	.brightness_map = fd6551_brightness_map,
	.max_brightness = 8,
};

static u8 tm16xx_ascii_to_segments(struct tm16xx_display *display, char c)
{
	static SEG7_CONVERSION_MAP(map_seg7, MAP_ASCII7SEG_ALPHANUM);
	u8 standard_segments, mapped_segments = 0;

	standard_segments = map_to_seg7(&map_seg7, c);

	for (int i = 0; i < 7; i++) {
		if (standard_segments & BIT(i))
			mapped_segments |= BIT(display->segment_mapping[i]);
	}

	return mapped_segments;
}

static int tm16xx_i2c_write(struct tm16xx_display *display, u8 *data, size_t len)
{
	dev_dbg(display->dev, "i2c_write %*ph", (char)len, data);

	struct i2c_msg msg = {
		.addr = data[0] >> 1,
		.flags = 0,
		.len = len - 1,
		.buf = &data[1],
	};
	int ret;

	ret = i2c_transfer(display->client.i2c->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return (ret == 1) ? len : -EIO;
}

static int tm16xx_spi_write(struct tm16xx_display *display, u8 *data, size_t len)
{
	struct spi_device *spi = display->client.spi;
	return spi_write(spi, data, len);
}

static int tm16xx_display_set_brightness(struct tm16xx_display *display, u8 brightness)
{
	u8 cmd[] = {
		display->chip_info->cmd_write_mode,
		display->chip_info->brightness_map(brightness),
	};
	int ret;

	ret = display->client_write(display, cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(display->dev, "Failed to set brightness: %d\n", ret);

	return ret;
}

static int tm16xx_display_write_data(struct tm16xx_display *display, u8 *data, size_t len)
{
	u8 cmd[2];
	int i;
	int ret;

	for(i=0; i<len; i++) {
		cmd[0] = display->chip_info->cmd_base_addr + i * sizeof(u8) * 2;
		cmd[1] = data[i];

		ret = display->client_write(display, cmd, sizeof(cmd));
		if (ret < 0) {
			dev_err(display->dev, "Failed to write display data: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int tm16xx_display_init(struct tm16xx_display *display)
{
	u8 cmd = display->chip_info->cmd_init;
	int ret;

	if (cmd) {
		ret = display->client_write(display, &cmd, sizeof(cmd));
		if (ret < 0)
			return ret;
	}

	ret = tm16xx_display_set_brightness(display, display->main_led.brightness);
	if (ret < 0)
		return ret;

	memset(display->display_data, 0xFF, display->display_data_len);
	ret = tm16xx_display_write_data(display, display->display_data, display->display_data_len);
	memset(display->display_data, 0x00, display->display_data_len);

	return ret;
	if (ret < 0)
		dev_err(display->dev, "Failed to initialize TM16xx chip: %d\n", ret);
	else
		dev_info(display->dev, "TM16xx chip initialized successfully\n");

	return ret;
}

static void tm16xx_display_remove(struct tm16xx_display *display)
{
	int ret;

	memset(display->display_data, 0x00, display->display_data_len);
	ret = tm16xx_display_write_data(display, display->display_data, display->display_data_len);

	if (ret >= 0)
		ret = tm16xx_display_set_brightness(display, LED_OFF);

	if (ret < 0)
		dev_err(display->dev, "Failed to turn off display: %d\n", ret);
	else
		dev_info(display->dev, "Display turned off\n");
}

static void tm16xx_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	mutex_lock(&display->lock);
	led_cdev->brightness = brightness;
	tm16xx_display_set_brightness(display, brightness);
	mutex_unlock(&display->lock);
}

static void tm16xx_led_set(struct led_classdev *led_cdev, enum led_brightness status)
{
	struct tm16xx_led *led = container_of(led_cdev, struct tm16xx_led, cdev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	mutex_lock(&display->lock);

	if (status)
		display->display_data[led->grid] |= (1 << led->segment);
	else
		display->display_data[led->grid] &= ~(1 << led->segment);

	tm16xx_display_write_data(display, display->display_data, display->display_data_len);

	mutex_unlock(&display->lock);
}

static ssize_t tm16xx_display_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);
	int i;

	mutex_lock(&display->lock);
	for (i = 0; i < display->num_digits && i < PAGE_SIZE - 1; i++) {
		buf[i] = display->digits[i].value;
	}
	buf[i++] = '\n';
	mutex_unlock(&display->lock);

	return i;
}

static ssize_t tm16xx_display_value_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);
	struct tm16xx_digit *digit;
	int i;
	u8 data;

	mutex_lock(&display->lock);

	for (i = 0; i < display->num_digits; i++) {
		digit = &display->digits[i];

		if (i < count && buf[i] != '\n') {
			digit->value = buf[i];
			data = tm16xx_ascii_to_segments(display, digit->value);
		} else {
			digit->value = 0;
			data = 0;
		}

		display->display_data[digit->grid] = data;
	}

	tm16xx_display_write_data(display, display->display_data, display->display_data_len);

	mutex_unlock(&display->lock);

	return count;
}

static DEVICE_ATTR(display_value, 0644, tm16xx_display_value_show, tm16xx_display_value_store);

static struct attribute *tm16xx_main_led_attrs[] = {
	&dev_attr_display_value.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tm16xx_main_led);

static int tm16xx_parse_dt(struct device *dev, struct tm16xx_display *display)
{
	struct fwnode_handle *child;
	int ret, i, max_grid = 0;
	u8 *digits;

	dev_info(dev, "Parsing device tree for TM16XX display\n");

	ret = device_property_count_u8(dev, "titan,digits");
	if (ret < 0) {
		dev_err(dev, "Failed to count 'titan,digits' property: %d\n", ret);
		return ret;
	}

	display->num_digits = ret;
	dev_info(dev, "Number of digits: %d\n", display->num_digits);

	digits = devm_kcalloc(dev, display->num_digits, sizeof(*digits), GFP_KERNEL);
	if (!digits) {
		dev_err(dev, "Failed to allocate memory for digits\n");
		return -ENOMEM;
	}

	ret = device_property_read_u8_array(dev, "titan,digits", digits, display->num_digits);
	if (ret < 0) {
		dev_err(dev, "Failed to read 'titan,digits' property: %d\n", ret);
		return ret;
	}

	display->digits = devm_kcalloc(dev, display->num_digits, sizeof(*display->digits), GFP_KERNEL);
	if (!display->digits) {
		dev_err(dev, "Failed to allocate memory for display digits\n");
		return -ENOMEM;
	}

	for (i = 0; i < display->num_digits; i++) {
		display->digits[i].grid = digits[i];
		max_grid = umax(max_grid, digits[i]);
	}

	devm_kfree(dev, digits);

	display->num_segments = device_property_count_u8(dev, "titan,segment-mapping");
	if (display->num_segments < 0) {
		dev_err(dev, "Failed to count 'titan,segment-mapping' property: %d\n", display->num_segments);
		return display->num_segments;
	}

	dev_info(dev, "Number of segments: %d\n", display->num_segments);

	display->segment_mapping = devm_kcalloc(dev, display->num_segments, sizeof(*display->segment_mapping), GFP_KERNEL);
	if (!display->segment_mapping) {
		dev_err(dev, "Failed to allocate memory for segment mapping\n");
		return -ENOMEM;
	}

	ret = device_property_read_u8_array(dev, "titan,segment-mapping", display->segment_mapping, display->num_segments);
	if (ret < 0) {
		dev_err(dev, "Failed to read 'titan,segment-mapping' property: %d\n", ret);
		return ret;
	}

	display->num_leds = 0;
	device_for_each_child_node(dev, child) {
		u32 reg[2];

		ret = fwnode_property_read_u32_array(child, "reg", reg, 2);
		if (ret < 0) {
			dev_err(dev, "Failed to read 'reg' property of led node: %d\n", ret);
			return ret;
		}

		max_grid = umax(max_grid, reg[0]);
		display->num_leds++;
	}

	dev_info(dev, "Number of LEDs: %d\n", display->num_leds);

	display->display_data_len = max_grid + 1;
	dev_info(dev, "Number of display grids: %zu\n", display->display_data_len);

	display->display_data = devm_kcalloc(dev, display->display_data_len, sizeof(*display->display_data), GFP_KERNEL);
	if (!display->display_data) {
		dev_err(dev, "Failed to allocate memory for display data\n");
		return -ENOMEM;
	}

	dev_info(dev, "Device tree parsing complete\n");
	return 0;
}

static int tm16xx_probe(struct tm16xx_display *display)
{
	struct device *dev = display->dev;
	struct fwnode_handle *child;
	int ret, i;

	mutex_init(&display->lock);

	ret = tm16xx_parse_dt(dev, display);
	if (ret < 0) {
		dev_err(dev, "Failed to parse device tree: %d\n", ret);
		return ret;
	}

	display->main_led.name = TM16XX_DEVICE_NAME;
	display->main_led.brightness = display->chip_info->max_brightness;
	display->main_led.max_brightness = display->chip_info->max_brightness;
	display->main_led.brightness_set = tm16xx_brightness_set;
	display->main_led.groups = tm16xx_main_led_groups;
	display->main_led.flags = LED_RETAIN_AT_SHUTDOWN;

	ret = devm_led_classdev_register(dev, &display->main_led);
	if (ret < 0) {
		dev_err(dev, "Failed to register main LED: %d\n", ret);
		return ret;
	}

	display->leds = devm_kcalloc(dev, display->num_leds, sizeof(*display->leds), GFP_KERNEL);
	if (!display->leds)
		return -ENOMEM;

	i = 0;
	device_for_each_child_node(dev, child) {
		struct tm16xx_led *led = &display->leds[i];
		struct led_init_data led_init = {
			.fwnode = child,
			.devicename = display->main_led.name,
			.devname_mandatory =  true,
		};
		u32 reg[2];

		ret = fwnode_property_read_u32_array(child, "reg", reg, 2);
		if (ret < 0) {
			dev_err(dev, "Failed to read LED reg property: %d\n", ret);
			return ret;
		}

		led->grid = reg[0];
		led->segment = reg[1];

		led->cdev.name = fwnode_get_name(child);
		led->cdev.max_brightness = 1;
		led->cdev.brightness_set = tm16xx_led_set;
		led->cdev.flags = LED_RETAIN_AT_SHUTDOWN;

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &led_init);
		if (ret < 0) {
			dev_err(dev, "Failed to register LED %s: %d\n", led->cdev.name, ret);
			return ret;
		}

		i++;
	}

	ret = tm16xx_display_init(display);
	if (ret < 0) {
		dev_err(display->dev, "Failed to initialize TM16xx chip: %d\n", ret);
		return ret;
	}

	dev_info(display->dev, "TM16xx chip initialized successfully\n");
	return 0;
}

// SPI specific code
static int tm16xx_spi_probe(struct spi_device *spi)
{
	const struct tm16xx_chip_info *chip_info;
	struct tm16xx_display *display;
	int ret;

	chip_info = of_device_get_match_data(&spi->dev);
	if (!chip_info)
		return -EINVAL;

	display = devm_kzalloc(&spi->dev, sizeof(*display), GFP_KERNEL);
	if (!display)
		return -ENOMEM;

	display->client.spi = spi;
	display->dev = &spi->dev;
	display->chip_info = chip_info;
	display->client_write = tm16xx_spi_write;

	spi_set_drvdata(spi, display);

	ret = tm16xx_probe(display);
	if (ret)
		return ret;

	return 0;
}

static void tm16xx_spi_remove(struct spi_device *spi)
{
	struct tm16xx_display *display = spi_get_drvdata(spi);
	tm16xx_display_remove(display);
}

static const struct of_device_id tm16xx_spi_of_match[] = {
	{ .compatible = "titan,tm1628", .data = &tm1628_chip_info },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_spi_of_match);

static const struct spi_device_id tm16xx_spi_id[] = {
	{ "tm1628", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, tm16xx_spi_id);

static struct spi_driver tm16xx_spi_driver = {
	.driver = {
		.name = TM16XX_DRIVER_NAME,
		.of_match_table = tm16xx_spi_of_match,
	},
	.probe = tm16xx_spi_probe,
	.remove = tm16xx_spi_remove,
	.shutdown = tm16xx_spi_remove,
	.id_table = tm16xx_spi_id,
};

// I2C specific code
static int tm16xx_i2c_probe(struct i2c_client *client)
{
	const struct tm16xx_chip_info *chip_info;
	struct tm16xx_display *display;
	int ret;

	chip_info = of_device_get_match_data(&client->dev);
	if (!chip_info)
		return -EINVAL;

	display = devm_kzalloc(&client->dev, sizeof(*display), GFP_KERNEL);
	if (!display)
		return -ENOMEM;

	display->client.i2c = client;
	display->dev = &client->dev;
	display->chip_info = chip_info;
	display->client_write = tm16xx_i2c_write;

	i2c_set_clientdata(client, display);

	ret = tm16xx_probe(display);
	if (ret)
		return ret;

	return 0;
}

static void tm16xx_i2c_remove(struct i2c_client *client)
{
	struct tm16xx_display *display = i2c_get_clientdata(client);
	tm16xx_display_remove(display);
}

static const struct of_device_id tm16xx_i2c_of_match[] = {
	{ .compatible = "titan,tm1650", .data = &tm1650_chip_info },
	{ .compatible = "fuda,fd6551", .data = &fd6551_chip_info },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_i2c_of_match);

static const struct i2c_device_id tm16xx_i2c_id[] = {
	{ "tm1650", 0 },
	{ "fd6551", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tm16xx_i2c_id);

static struct i2c_driver tm16xx_i2c_driver = {
	.driver = {
		.name = TM16XX_DRIVER_NAME,
		.of_match_table = tm16xx_i2c_of_match,
	},
	.probe = tm16xx_i2c_probe,
	.remove = tm16xx_i2c_remove,
	.shutdown = tm16xx_i2c_remove,
	.id_table = tm16xx_i2c_id,
};

static int __init tm16xx_init(void)
{
	int ret;

	ret = spi_register_driver(&tm16xx_spi_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&tm16xx_i2c_driver);
	if (ret) {
		spi_unregister_driver(&tm16xx_spi_driver);
		return ret;
	}

	return 0;
}

static void __exit tm16xx_exit(void)
{
	i2c_del_driver(&tm16xx_i2c_driver);
	spi_unregister_driver(&tm16xx_spi_driver);
}

module_init(tm16xx_init);
module_exit(tm16xx_exit);

MODULE_AUTHOR("Jean-François Lessard");
MODULE_DESCRIPTION("Driver for Titan Micro Electronics TM16XX LED display driver chips");
MODULE_LICENSE("GPL");
