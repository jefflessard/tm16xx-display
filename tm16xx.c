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
#include <linux/workqueue.h>
#include <linux/map_to_7segment.h>

#define TM16XX_DRIVER_NAME "tm16xx"
#define TM16XX_DEVICE_NAME "display"

struct tm16xx_display;

struct tm16xx_controller {
	u8 max_brightness;
	int (*init)(struct tm16xx_display *display, u8 **cmd);
	int (*brightness)(struct tm16xx_display *display, u8 **cmd);
	int (*data)(struct tm16xx_display *display, u8 **cmd, int data_index);
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
	const struct tm16xx_controller *controller;
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
	struct work_struct flush_brightness;
	struct work_struct flush_display;
	int (*client_write)(struct tm16xx_display *display, u8 *data, size_t len);
};

static int tm1628_cmd_init(struct tm16xx_display *display, u8 **cmd) {
	// 01b mode is 5 grids with minimum of 7 segments
	//    tm1618 : 5 digits, 7 segments
	//    tm1620 : 5 digits, 9 segments
	//    tm1628 : 5 digits, 12 segments
	static const u8 MODE_CMD = 0<<7|0<<6, MODE=0<<1|1<<0;
	static const u8 DATA_CMD = 0<<7|1<<6, DATA_ADDR_MODE = 0<<2, DATA_WRITE_MODE = 0<<1|0<<0;

	static u8 cmds[] = {
		MODE_CMD | MODE,
		DATA_CMD | DATA_ADDR_MODE | DATA_WRITE_MODE,
	};

	*cmd = cmds;
	return sizeof(cmds);
}

static int tm1628_cmd_brightness(struct tm16xx_display *display, u8 **cmd) {
	static u8 cmds[1];
	static const u8 CTRL_CMD = 1<<7|0<<6, ON_FLAG = 1<<3, BR_MASK = 7, BR_SHIFT = 0;

	int i = display->main_led.brightness;
	cmds[0] = CTRL_CMD | ((i && 1) * (((i-1) & BR_MASK) << BR_SHIFT | ON_FLAG));

	*cmd = cmds;
	return sizeof(cmds);
}

static int tm1618_cmd_data(struct tm16xx_display *display, u8 **cmd, int i) {
	static const u8 ADDR_CMD = 1<<7|1<<6;
	static const u8 BYTE1_MASK = 0x1F, BYTE1_RSHIFT = 0;
	static const u8 BYTE2_MASK = ~BYTE1_MASK, BYTE2_RSHIFT = 5-3;
	static u8 cmds[3];

	cmds[0] = ADDR_CMD + i * 2;
	cmds[1] = (display->display_data[i] & BYTE1_MASK) >> BYTE1_RSHIFT;
	cmds[2] = (display->display_data[i] & BYTE2_MASK) >> BYTE2_RSHIFT;

	*cmd = cmds;
	return sizeof(cmds);
}

static int tm1628_cmd_data(struct tm16xx_display *display, u8 **cmd, int i) {
	static const u8 ADDR_CMD = 1<<7|1<<6;
	static u8 cmds[3];

	cmds[0] = ADDR_CMD + i * 2;
	cmds[1] = display->display_data[i]; // SEG 1 to 8
	cmds[2] = 0; // SEG 9 to 14

	*cmd = cmds;
	return sizeof(cmds);
}

static int tm1650_cmd_brightness(struct tm16xx_display *display, u8 **cmd) {
	static u8 cmds[2];
	static const u8 ON_FLAG = 1, BR_MASK = 7, BR_SHIFT = 4, SEG7_MODE = 1<<3;

	int i = display->main_led.brightness;
	cmds[0]	= 0x48;
	cmds[1] = (i && 1) * ((i & BR_MASK) << BR_SHIFT | SEG7_MODE | ON_FLAG);

	*cmd = cmds;
	return sizeof(cmds);
}

static int tm1650_cmd_data(struct tm16xx_display *display, u8 **cmd, int i) {
	static const u8 BASE_ADDR = 0x68;
	static u8 cmds[2];

	cmds[0] = BASE_ADDR + i * 2;
	cmds[1] = display->display_data[i]; // SEG 1 to 8

	*cmd = cmds;
	return sizeof(cmds);
}

static int fd655_cmd_brightness(struct tm16xx_display *display, u8 **cmd) {
	static u8 cmds[2];
	static const u8 ON_FLAG = 1, BR_MASK = 3, BR_SHIFT = 5;
	
	int i = display->main_led.brightness;
	cmds[0]	= 0x48;
	cmds[1] = (i && 1) * (((i%3) & BR_MASK) << BR_SHIFT | ON_FLAG);

	*cmd = cmds;
	return sizeof(cmds);
}

static int fd655_cmd_data(struct tm16xx_display *display, u8 **cmd, int i) {
	static const u8 BASE_ADDR = 0x66;
	static u8 cmds[2];

	cmds[0] = BASE_ADDR + i * 2;
	cmds[1] = display->display_data[i]; // SEG 1 to 8

	*cmd = cmds;
	return sizeof(cmds);
}

static int fd6551_cmd_brightness(struct tm16xx_display *display, u8 **cmd) {
	static u8 cmds[2];
	static const u8 ON_FLAG = 1, BR_MASK = 7, BR_SHIFT = 1;

	int i = display->main_led.brightness;
	cmds[0]	= 0x48;
	cmds[1] = (i && 1) * ((~(i-1) & BR_MASK) << BR_SHIFT | ON_FLAG);

	*cmd = cmds;
	return sizeof(cmds);
}

static const struct tm16xx_controller tm1618_controller = {
	.max_brightness = 8,
	.init = tm1628_cmd_init,
	.brightness = tm1628_cmd_brightness,
	.data = tm1618_cmd_data,
};

static const struct tm16xx_controller tm1628_controller = {
	.max_brightness = 8,
	.init = tm1628_cmd_init,
	.brightness = tm1628_cmd_brightness,
	.data = tm1628_cmd_data,
};

static const struct tm16xx_controller tm1650_controller = {
	.max_brightness = 8,
	.init = NULL,
	.brightness = tm1650_cmd_brightness,
	.data = tm1650_cmd_data,
};

static const struct tm16xx_controller fd655_controller = {
	.max_brightness = 3,
	.init = NULL,
	.brightness = fd655_cmd_brightness,
	.data = fd655_cmd_data,
};

static const struct tm16xx_controller fd6551_controller = {
	.max_brightness = 8,
	.init = NULL,
	.brightness = fd6551_cmd_brightness,
	.data = fd655_cmd_data,
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
	dev_dbg(display->dev, "spi_write %*ph", (char)len, data);

	struct spi_device *spi = display->client.spi;
	return spi_write(spi, data, len);
}

static void tm16xx_display_flush_brightness(struct work_struct * work) {
	struct tm16xx_display *display = container_of(work, struct tm16xx_display, flush_brightness);
	u8 *cmd;
      	int len=-1, ret;

	if (display->controller->brightness) {
		len = display->controller->brightness(display, &cmd);
	}

	if (len>0) {
		mutex_lock(&display->lock);
		ret = display->client_write(display, cmd, len);
		mutex_unlock(&display->lock);
		if (ret < 0)
			dev_err(display->dev, "Failed to set brightness: %d\n", ret);
	}
}

static void tm16xx_display_flush_data(struct work_struct * work) {
	struct tm16xx_display *display = container_of(work, struct tm16xx_display, flush_display);

	u8 *cmd;
	int i, len=-1, ret;

	mutex_lock(&display->lock);

	for(i=0; i<display->display_data_len; i++) {
		if (display->controller->data) {
			len = display->controller->data(display, &cmd, i);
		}

		if (len > 0) {
			ret = display->client_write(display, cmd, len);
			if (ret < 0) {
				dev_err(display->dev, "Failed to write display data: %d\n", ret);
			}
		}
	}

	mutex_unlock(&display->lock);
}

static int tm16xx_display_init(struct tm16xx_display *display)
{
	u8 *cmd;
	int len=-1, ret;

	if (display->controller->init) {
		len = display->controller->init(display, &cmd);
	}

	if (len > 0) {
		mutex_lock(&display->lock);
		ret = display->client_write(display, cmd, len);
		mutex_unlock(&display->lock);
		if (ret < 0)
			return ret;
	}

	schedule_work(&display->flush_brightness);
	flush_work(&display->flush_brightness);

	memset(display->display_data, 0xFF, display->display_data_len);
	schedule_work(&display->flush_display);
	flush_work(&display->flush_display);
	memset(display->display_data, 0x00, display->display_data_len);

	return ret;
}

static void tm16xx_display_remove(struct tm16xx_display *display)
{
	memset(display->display_data, 0x00, display->display_data_len);
	schedule_work(&display->flush_display);
	flush_work(&display->flush_display);

	display->main_led.brightness = LED_OFF;
	schedule_work(&display->flush_brightness);
	flush_work(&display->flush_brightness);

	dev_info(display->dev, "Display turned off\n");
}

static void tm16xx_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	led_cdev->brightness = brightness;
	schedule_work(&display->flush_brightness);
}

static void tm16xx_led_set(struct led_classdev *led_cdev, enum led_brightness status)
{
	struct tm16xx_led *led = container_of(led_cdev, struct tm16xx_led, cdev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	if (status)
		display->display_data[led->grid] |= (1 << led->segment);
	else
		display->display_data[led->grid] &= ~(1 << led->segment);

	schedule_work(&display->flush_display);
}

static ssize_t tm16xx_display_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);
	int i;

	for (i = 0; i < display->num_digits && i < PAGE_SIZE - 1; i++) {
		buf[i] = display->digits[i].value;
	}
	buf[i++] = '\n';

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

	schedule_work(&display->flush_display);

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

	ret = device_property_count_u8(dev, "tm16xx,digits");
	if (ret < 0) {
		dev_err(dev, "Failed to count 'tm16xx,digits' property: %d\n", ret);
		return ret;
	}

	display->num_digits = ret;
	dev_info(dev, "Number of digits: %d\n", display->num_digits);

	digits = devm_kcalloc(dev, display->num_digits, sizeof(*digits), GFP_KERNEL);
	if (!digits) {
		dev_err(dev, "Failed to allocate memory for digits\n");
		return -ENOMEM;
	}

	ret = device_property_read_u8_array(dev, "tm16xx,digits", digits, display->num_digits);
	if (ret < 0) {
		dev_err(dev, "Failed to read 'tm16xx,digits' property: %d\n", ret);
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

	display->num_segments = device_property_count_u8(dev, "tm16xx,segment-mapping");
	if (display->num_segments < 0) {
		dev_err(dev, "Failed to count 'tm16xx,segment-mapping' property: %d\n", display->num_segments);
		return display->num_segments;
	}

	dev_info(dev, "Number of segments: %d\n", display->num_segments);

	display->segment_mapping = devm_kcalloc(dev, display->num_segments, sizeof(*display->segment_mapping), GFP_KERNEL);
	if (!display->segment_mapping) {
		dev_err(dev, "Failed to allocate memory for segment mapping\n");
		return -ENOMEM;
	}

	ret = device_property_read_u8_array(dev, "tm16xx,segment-mapping", display->segment_mapping, display->num_segments);
	if (ret < 0) {
		dev_err(dev, "Failed to read 'tm16xx,segment-mapping' property: %d\n", ret);
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

	return 0;
}

static int tm16xx_probe(struct tm16xx_display *display)
{
	struct device *dev = display->dev;
	struct fwnode_handle *child;
	int ret, i;

	ret = tm16xx_parse_dt(dev, display);
	if (ret < 0) {
		dev_err(dev, "Failed to parse device tree: %d\n", ret);
		return ret;
	}

	display->main_led.name = TM16XX_DEVICE_NAME;
	display->main_led.brightness = display->controller->max_brightness;
	display->main_led.max_brightness = display->controller->max_brightness;
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

	mutex_init(&display->lock);
	mutex_init(&display->lock);
	INIT_WORK(&display->flush_brightness, tm16xx_display_flush_brightness);
	INIT_WORK(&display->flush_display, tm16xx_display_flush_data);

	ret = tm16xx_display_init(display);
	if (ret < 0) {
		dev_err(display->dev, "Failed to initialize display: %d\n", ret);
		return ret;
	}
	
	dev_info(display->dev, "Display initialized successfully\n");
	return 0;
}

// SPI specific code
static int tm16xx_spi_probe(struct spi_device *spi)
{
	const struct tm16xx_controller *controller;
	struct tm16xx_display *display;
	int ret;

	controller = of_device_get_match_data(&spi->dev);
	if (!controller)
		return -EINVAL;

	display = devm_kzalloc(&spi->dev, sizeof(*display), GFP_KERNEL);
	if (!display)
		return -ENOMEM;

	display->client.spi = spi;
	display->dev = &spi->dev;
	display->controller = controller;
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
	{ .compatible = "titanmec,tm1618", .data = &tm1618_controller },
	{ .compatible = "titanmec,tm1620", .data = &tm1628_controller },
	{ .compatible = "titanmec,tm1628", .data = &tm1628_controller },
	{ .compatible = "fdhisi,fd620", .data = &tm1628_controller },
	{ .compatible = "fdhisi,fd628", .data = &tm1628_controller },
	{ .compatible = "princeton,pt6964", .data = &tm1628_controller },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_spi_of_match);

static const struct spi_device_id tm16xx_spi_id[] = {
	{ "tm1618", 0 },
	{ "tm1620", 0 },
	{ "tm1628", 0 },
	{ "fd620", 0 },
	{ "fd628", 0 },
	{ "pt6964", 0 },
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
	const struct tm16xx_controller *controller;
	struct tm16xx_display *display;
	int ret;

	controller = of_device_get_match_data(&client->dev);
	if (!controller)
		return -EINVAL;

	display = devm_kzalloc(&client->dev, sizeof(*display), GFP_KERNEL);
	if (!display)
		return -ENOMEM;

	display->client.i2c = client;
	display->dev = &client->dev;
	display->controller = controller;
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
	{ .compatible = "titanmec,tm1650", .data = &tm1650_controller },
	{ .compatible = "fdhisi,fd650", .data = &tm1650_controller },
	{ .compatible = "fdhisi,fd6551", .data = &fd6551_controller },
	{ .compatible = "fdhisi,fd655", .data = &fd655_controller },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_i2c_of_match);

static const struct i2c_device_id tm16xx_i2c_id[] = {
	{ "tm1650", 0 },
	{ "fd650", 0 },
	{ "fd6551", 0 },
	{ "fd655", 0 },
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
