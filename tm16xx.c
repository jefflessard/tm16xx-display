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

struct tm16xx_display;

struct tm16xx_chip_info {
	const char *name;
	u8 max_brightness;
	u8 base_addr;
	int (*init)(struct device *dev);
	int (*set_brightness)(struct device *dev, u8 brightness);
	int (*write_display)(struct device *dev, u8 *data, size_t len);
	void (*remove)(struct tm16xx_display *display);
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
	u8 brightness;
	u8 *display_data;
	size_t display_data_len;
	struct mutex lock;
	int (*transfer)(struct tm16xx_display *display, u8 *data, size_t len);
};

static int tm16xx_i2c_transfer(struct tm16xx_display *display, u8 *data, size_t len)
{
	struct i2c_client *client = display->client.i2c;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len,
		.buf = data,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;
	return (ret == 1) ? len : -EIO;
}

static int tm16xx_spi_transfer(struct tm16xx_display *display, u8 *data, size_t len)
{
	struct spi_device *spi = display->client.spi;
	return spi_write(spi, data, len);
}

static int tm165x_cmd_init(struct device *dev)
{
	struct tm16xx_display *display = dev_get_drvdata(dev);
	u8 cmd[] = {display->chip_info->base_addr, 0x01}; // Enable display, normal mode
	int ret;

	ret = display->transfer(display, cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(dev, "Failed to initialize TM165x chip: %d\n", ret);
	else
		dev_info(dev, "TM165x chip initialized successfully\n");

	return ret;
}

static int tm16xx_cmd_init(struct device *dev)
{
	struct tm16xx_display *display = dev_get_drvdata(dev);
	u8 cmd[] = {display->chip_info->base_addr, 0x00 | (display->num_segments - 1)}; // Set display mode
	int ret;

	ret = display->transfer(display, cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(dev, "Failed to initialize TM16xx chip: %d\n", ret);
	else
		dev_info(dev, "TM16xx chip initialized successfully\n");

	return ret;
}

static int tm16xx_set_brightness(struct device *dev, u8 brightness)
{
	struct tm16xx_display *display = dev_get_drvdata(dev);
	u8 cmd[] = {display->chip_info->base_addr | 0x80, brightness};
	int ret;

	ret = display->transfer(display, cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(dev, "Failed to set brightness: %d\n", ret);

	return ret;
}

static int tm16xx_write_display(struct device *dev, u8 *data, size_t len)
{
	struct tm16xx_display *display = dev_get_drvdata(dev);
	u8 *cmd;
	int ret;

	cmd = devm_kcalloc(dev, len + 1, sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd[0] = display->chip_info->base_addr | 0xC0;
	memcpy(&cmd[1], data, len);

	ret = display->transfer(display, cmd, len + 1);
	if (ret < 0)
		dev_err(dev, "Failed to write display data: %d\n", ret);

	devm_kfree(dev, cmd);
	return ret;
}

static void tm16xx_remove(struct tm16xx_display *display)
{
	struct device *dev = display->dev;
	u8 cmd[] = {display->chip_info->base_addr | 0x80, 0x00}; // Turn off display
	int ret;

	ret = display->transfer(display, cmd, sizeof(cmd));
	if (ret < 0)
		dev_err(dev, "Failed to turn off display during removal: %d\n", ret);
	else
		dev_info(dev, "Display turned off successfully during removal\n");
}

static const struct tm16xx_chip_info tm16xx_chip_info[] = {
	{
		.name = "tm1628",
		.max_brightness = 7,
		.base_addr = 0x00,
		.init = tm16xx_cmd_init,
		.set_brightness = tm16xx_set_brightness,
		.write_display = tm16xx_write_display,
		.remove = tm16xx_remove,
	},
	{
		.name = "tm1650",
		.max_brightness = 7,
		.base_addr = 0x48,
		.init = tm165x_cmd_init,
		.set_brightness = tm16xx_set_brightness,
		.write_display = tm16xx_write_display,
		.remove = tm16xx_remove,
	},
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

static void tm16xx_update_display(struct tm16xx_display *display)
{
	int i;
	u8 segment_data;

	mutex_lock(&display->lock);

	for (i = 0; i < display->num_digits; i++) {
		segment_data = tm16xx_ascii_to_segments(display, display->digits[i].value);
	}

	display->chip_info->write_display(display->dev, display->display_data, display->display_data_len);

	mutex_unlock(&display->lock);
}

static void tm16xx_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	mutex_lock(&display->lock);
	display->brightness = brightness;
	display->chip_info->set_brightness(display->dev, brightness);
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
	display->chip_info->write_display(display->dev, display->display_data, display->display_data_len);
	mutex_unlock(&display->lock);
}

static ssize_t tm16xx_display_value_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);
	int i;

	mutex_lock(&display->lock);

	for (i = 0; i < display->num_digits && i < count; i++)
		display->digits[i].value = buf[i];

	for (; i < display->num_digits; i++)
		display->digits[i].value = ' ';

	tm16xx_update_display(display);

	mutex_unlock(&display->lock);

	return count;
}

static DEVICE_ATTR_WO(tm16xx_display_value);

static struct attribute *tm16xx_main_led_attrs[] = {
	&dev_attr_tm16xx_display_value.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tm16xx_main_led);

static int tm16xx_parse_dt(struct device *dev, struct tm16xx_display *display)
{
	struct fwnode_handle *child;
	int ret, i, max_grid = 0;
	u32 *digits;

	ret = device_property_count_u32(dev, "titan,digits");
	if (ret < 0)
		return ret;

	display->num_digits = ret;
	digits = devm_kcalloc(dev, display->num_digits, sizeof(*digits), GFP_KERNEL);
	if (!digits)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, "titan,digits", digits, display->num_digits);
	if (ret < 0)
		return ret;

	display->digits = devm_kcalloc(dev, display->num_digits, sizeof(*display->digits), GFP_KERNEL);
	if (!display->digits)
		return -ENOMEM;

	for (i = 0; i < display->num_digits; i++) {
		display->digits[i].grid = digits[i];
		max_grid = umax(max_grid, digits[i]);
	}

	devm_kfree(dev, digits);

	display->num_segments = device_property_count_u8(dev, "titan,segment-mapping");
	if (display->num_segments < 0)
		return display->num_segments;

	display->segment_mapping = devm_kcalloc(dev, display->num_segments, sizeof(*display->segment_mapping), GFP_KERNEL);
	if (!display->segment_mapping)
		return -ENOMEM;

	ret = device_property_read_u8_array(dev, "titan,segment-mapping", display->segment_mapping, display->num_segments);
	if (ret < 0)
		return ret;

	device_for_each_child_node(dev, child) {
		u32 reg[2];

		ret = fwnode_property_read_u32_array(child, "reg", reg, 2);
		if (ret < 0)
			return ret;

		max_grid = umax(max_grid, reg[0]);
		display->num_leds++;
	}

	display->display_data_len = max_grid + 1;
	display->display_data = devm_kcalloc(dev, display->display_data_len, sizeof(*display->display_data), GFP_KERNEL);
	if (!display->display_data)
		return -ENOMEM;

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

	display->main_led.name = TM16XX_DRIVER_NAME;
	display->main_led.max_brightness = display->chip_info->max_brightness;
	display->main_led.brightness_set = tm16xx_brightness_set;
	display->main_led.groups = tm16xx_main_led_groups;

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

		ret = devm_led_classdev_register(dev, &led->cdev);
		if (ret < 0) {
			dev_err(dev, "Failed to register LED %s: %d\n", led->cdev.name, ret);
			return ret;
		}

		i++;
	}

	ret = display->chip_info->init(dev);
	if (ret < 0)
		return ret;

	ret = display->chip_info->set_brightness(dev, display->brightness);
	if (ret < 0)
		return ret;

	for (i = 0; i < display->num_digits; i++)
		display->digits[i].value = ' ';

	tm16xx_update_display(display);

	dev_info(dev, "TM16XX display driver probed successfully\n");

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
	display->transfer = tm16xx_spi_transfer;

	spi_set_drvdata(spi, display);

	ret = tm16xx_probe(display);
	if (ret)
		return ret;

	return 0;
}

static void tm16xx_spi_remove(struct spi_device *spi)
{
	struct tm16xx_display *display = spi_get_drvdata(spi);
	display->chip_info->remove(display);
}

static const struct of_device_id tm16xx_spi_of_match[] = {
	{ .compatible = "titan,tm1628", .data = &tm16xx_chip_info[0] },
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
	display->transfer = tm16xx_i2c_transfer;

	i2c_set_clientdata(client, display);

	ret = tm16xx_probe(display);
	if (ret)
		return ret;

	return 0;
}

static void tm16xx_i2c_remove(struct i2c_client *client)
{
	struct tm16xx_display *display = i2c_get_clientdata(client);
	display->chip_info->remove(display);
}

static const struct of_device_id tm16xx_i2c_of_match[] = {
	{ .compatible = "titan,tm1650", .data =  &tm16xx_chip_info[1] },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_i2c_of_match);

static const struct i2c_device_id tm16xx_i2c_id[] = {
	{ "tm1650", 0 },
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
