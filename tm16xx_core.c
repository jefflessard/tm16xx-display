// SPDX-License-Identifier: GPL-2.0
/*
 * TM16xx and compatible LED display/keypad controller driver
 * Supports TM16xx, FD6xx, PT6964, HBS658, AIP16xx and related chips.
 *
 * Copyright (C) 2024 Jean-François Lessard
 */

#include <linux/map_to_7segment.h>
#include <linux/module.h>
#include <linux/property.h>

#include "tm16xx.h"


#ifdef CONFIG_PANEL_BOOT_MESSAGE
static const char *tm16xx_init_value = CONFIG_PANEL_BOOT_MESSAGE;
#else
static const char *tm16xx_init_value = NULL;
#endif

static SEG7_CONVERSION_MAP(map_seg7, MAP_ASCII7SEG_ALPHANUM);

/* main display */
/**
 * tm16xx_display_flush_init() - Workqueue to configure controller and set brightness
 * @work: pointer to work_struct
 */
static void tm16xx_display_flush_init(struct work_struct *work)
{
	struct tm16xx_display *display = container_of(work,
						      struct tm16xx_display,
						      flush_init);
	int ret;

	if (display->controller->init) {
		mutex_lock(&display->lock);
		dev_dbg(display->dev, "Configuring controller\n");
		ret = display->controller->init(display);
		display->flush_status = ret;
		mutex_unlock(&display->lock);
		if (ret < 0)
			dev_err(display->dev,
				"Failed to configure controller: %d\n", ret);
	}
}

/**
 * tm16xx_display_flush_data() - Workqueue to update display data to controller
 * @work: pointer to work_struct
 */
static void tm16xx_display_flush_data(struct work_struct *work)
{
	struct tm16xx_display *display = container_of(work,
						      struct tm16xx_display,
						      flush_display);
	int i, ret = 0;
	unsigned int grid;

	mutex_lock(&display->lock);
	dev_dbg(display->dev, "Sending data to controller\n");

	if (display->controller->data) {
		for (i = 0; i < display->num_grids; i++) {
			grid = tm16xx_get_grid(display, i);
			ret = display->controller->data(display, i, grid);
			if (ret < 0) {
				dev_err(display->dev,
					"Failed to write display data: %d\n",
					ret);
				break;
			}
		}
	}

	display->flush_status = ret;
	mutex_unlock(&display->lock);
}

/**
 * tm16xx_brightness_set() - Set display main LED brightness
 * @led_cdev: pointer to led_classdev
 * @brightness: new brightness value
 */
static void tm16xx_brightness_set(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	dev_dbg(display->dev, "Setting brightness to %d\n", brightness);
	led_cdev->brightness = brightness;
	schedule_work(&display->flush_init);
}

/**
 * tm16xx_led_set() - Set state of an individual LED icon
 * @led_cdev: pointer to led_classdev
 * @value: new brightness (0/1)
 */
static void tm16xx_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct tm16xx_led *led = container_of(led_cdev, struct tm16xx_led, cdev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	dev_dbg(display->dev, "Setting led %u,%u to %d\n", led->grid,
		led->segment, value);

	tm16xx_set_seg(display, led->grid, led->segment, value);
	schedule_work(&display->flush_display);
}

/**
 * tm16xx_value_show() - Sysfs: show current display digit values
 * @dev: pointer to device
 * @attr: device attribute (unused)
 * @buf: output buffer
 *
 * Return: number of bytes written to output buffer
 */
static ssize_t tm16xx_value_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);
	int i;

	for (i = 0; i < display->num_digits && i < PAGE_SIZE - 1; i++)
		buf[i] = display->digits[i].value;

	buf[i++] = '\n';
	return i;
}

/**
 * tm16xx_value_store() - Sysfs: set display digit values
 * @dev: pointer to device
 * @attr: device attribute (unused)
 * @buf: new digit values (ASCII chars)
 * @count: buffer length
 *
 * Return: number of bytes written or negative error code
 */
static ssize_t tm16xx_value_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);
	struct tm16xx_digit *digit;
	struct tm16xx_digit_segment *ds;
	int i, j;
	int seg_pattern;
	bool val;

	dev_dbg(display->dev, "Setting value to %s\n", buf);

	for (i = 0; i < display->num_digits && i < count; i++) {
		digit = &display->digits[i];
		digit->value = buf[i];
		seg_pattern = map_to_seg7(&map_seg7, digit->value);

		for (j = 0; j < TM16XX_DIGIT_SEGMENTS; j++) {
			ds = &digit->segments[j];
			val = seg_pattern & BIT(j);
			tm16xx_set_seg(display, ds->grid, ds->segment, val);
		}
	}

	for (; i < display->num_digits; i++) {
		digit = &display->digits[i];
		digit->value = 0;
		for (j = 0; j < TM16XX_DIGIT_SEGMENTS; j++) {
			ds = &digit->segments[j];
			tm16xx_set_seg(display, ds->grid, ds->segment, 0);
		}
	}

	schedule_work(&display->flush_display);
	return count;
}

/**
 * tm16xx_num_digits_show() - Sysfs: show number of digits on display
 * @dev: pointer to device
 * @attr: device attribute (unused)
 * @buf: output buffer
 *
 * Return: number of bytes written
 */
static ssize_t tm16xx_num_digits_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	return sprintf(buf, "%u\n", display->num_digits);
}

/**
 * tm16xx_map_seg7_show() - Sysfs: show current 7-segment character map (binary blob)
 * @dev: pointer to device
 * @attr: device attribute (unused)
 * @buf: output buffer
 *
 * Return: size of map_seg7
 */
static ssize_t tm16xx_map_seg7_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	memcpy(buf, &map_seg7, sizeof(map_seg7));
	return sizeof(map_seg7);
}

/**
 * tm16xx_map_seg7_store() - Sysfs: set 7-segment character map (binary blob)
 * @dev: pointer to device
 * @attr: device attribute (unused)
 * @buf: new mapping (must match size of map_seg7)
 * @cnt: buffer length
 *
 * Return: cnt on success, negative errno on failure
 */
static ssize_t tm16xx_map_seg7_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t cnt)
{
	if (cnt != sizeof(map_seg7))
		return -EINVAL;
	memcpy(&map_seg7, buf, cnt);
	return cnt;
}

static DEVICE_ATTR(value, 0644, tm16xx_value_show, tm16xx_value_store);
static DEVICE_ATTR(num_digits, 0444, tm16xx_num_digits_show, NULL);
static DEVICE_ATTR(map_seg7, 0644, tm16xx_map_seg7_show, tm16xx_map_seg7_store);

static struct attribute *tm16xx_main_led_attrs[] = {
	&dev_attr_value.attr,
	&dev_attr_num_digits.attr,
	&dev_attr_map_seg7.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tm16xx_main_led);

/**
 * tm16xx_display_init() - Initialize display hardware and state
 * @display: pointer to tm16xx_display
 *
 * Return: 0 on success, negative error code on failure
 */
static int tm16xx_display_init(struct tm16xx_display *display)
{
	unsigned int nbits = tm16xx_led_nbits(display);

	dev_dbg(display->dev, "Initializing display\n");
	schedule_work(&display->flush_init);
	flush_work(&display->flush_init);
	if (display->flush_status < 0)
		return display->flush_status;

	if (tm16xx_init_value) {
		tm16xx_value_store(display->main_led.dev, NULL,
				   tm16xx_init_value,
				   strlen(tm16xx_init_value));
	} else {
		bitmap_fill(display->state, nbits);
		schedule_work(&display->flush_display);
		flush_work(&display->flush_display);
		bitmap_zero(display->state, nbits);
		if (display->flush_status < 0)
			return display->flush_status;
	}

	dev_info(display->dev, "Display turned on\n");

	return 0;
}

/**
 * tm16xx_parse_dt() - Parse device tree for digit and LED mapping
 * @dev: pointer to struct device
 * @display: pointer to tm16xx_display
 *
 * Return: 0 on success, negative error code on failure
 */
static int tm16xx_parse_dt(struct device *dev, struct tm16xx_display *display)
{
	struct fwnode_handle *leds_node, *digits_node, *child;
	struct tm16xx_led *led;
	struct tm16xx_digit *digit;
	int max_grid = 0, max_segment = 0;
	int ret, i, j;
	u32 segments[TM16XX_DIGIT_SEGMENTS * 2];
	u32 reg[2];

	/* parse digits */
	digits_node = device_get_named_child_node(dev, "digits");
	if (digits_node) {
		display->num_digits = 0;
		fwnode_for_each_child_node(digits_node, child)
			display->num_digits++;

		dev_dbg(dev, "Number of digits: %u\n", display->num_digits);

		if (display->num_digits) {
			display->digits = devm_kcalloc(dev, display->num_digits,
						       sizeof(*display->digits),
						       GFP_KERNEL);
			if (!display->digits) {
				fwnode_handle_put(digits_node);
				return -ENOMEM;
			}

			i = 0;
			fwnode_for_each_child_node(digits_node, child) {
				digit = &display->digits[i];

				ret = fwnode_property_read_u32(child, "reg",
							       reg);
				if (ret < 0) {
					fwnode_handle_put(child);
					fwnode_handle_put(digits_node);
					return ret;
				}

				ret = fwnode_property_read_u32_array(child,
								     "segments",
								     segments,
								     TM16XX_DIGIT_SEGMENTS * 2);
				if (ret < 0) {
					fwnode_handle_put(child);
					fwnode_handle_put(digits_node);
					return ret;
				}

				for (j = 0; j < TM16XX_DIGIT_SEGMENTS; ++j) {
					digit->segments[j].grid = segments[2 * j];
					digit->segments[j].segment = segments[2 * j + 1];
					max_grid = umax(max_grid,
							digit->segments[j].grid);
					max_segment = umax(max_segment,
							   digit->segments[j].segment);
				}
				digit->value = 0;
				i++;
			}

			fwnode_handle_put(digits_node);
		}
	}

	/* parse leds */
	leds_node = device_get_named_child_node(dev, "leds");
	if (leds_node) {
		display->num_leds = 0;
		fwnode_for_each_child_node(leds_node, child)
			display->num_leds++;

		dev_dbg(dev, "Number of LEDs: %u\n", display->num_leds);

		if (display->num_leds) {
			display->leds = devm_kcalloc(dev, display->num_leds,
						     sizeof(*display->leds),
						     GFP_KERNEL);
			if (!display->leds) {
				fwnode_handle_put(leds_node);
				return -ENOMEM;
			}

			i = 0;
			fwnode_for_each_child_node(leds_node, child) {
				led = &display->leds[i];
				ret = fwnode_property_read_u32_array(child,
								     "reg", reg,
								     2);
				if (ret < 0) {
					fwnode_handle_put(child);
					fwnode_handle_put(leds_node);
					return ret;
				}

				led->grid = reg[0];
				led->segment = reg[1];
				max_grid = umax(max_grid, led->grid);
				max_segment = umax(max_segment, led->segment);
				i++;
			}
		}

		fwnode_handle_put(leds_node);
	}

	if (max_grid >= display->controller->max_grids) {
		dev_err(dev, "grid %u exceeds controller max_grids %u\n",
			max_grid, display->controller->max_grids);
		return -EINVAL;
	}

	if (max_segment >= display->controller->max_segments) {
		dev_err(dev, "segment %u exceeds controller max_segments %u\n",
			max_segment, display->controller->max_segments);
		return -EINVAL;
	}

	display->num_grids = max_grid + 1;
	display->num_segments = max_segment + 1;

	dev_dbg(dev, "Number of grids: %u\n", display->num_grids);
	dev_dbg(dev, "Number of segments: %u\n", display->num_segments);

	return 0;
}

/**
 * tm16xx_probe() - Probe and initialize display device, register LEDs
 * @display: pointer to tm16xx_display
 *
 * Return: 0 on success, negative error code on failure
 */
int tm16xx_probe(struct tm16xx_display *display)
{
	struct device *dev = display->dev;
	struct led_classdev *main = &display->main_led;
	struct fwnode_handle *leds_node, *child;
	unsigned int nbits;
	int ret, i;

	dev_dbg(dev, "Probing device\n");
	ret = tm16xx_parse_dt(dev, display);
	if (ret < 0) {
		dev_err(dev, "Failed to parse device tree: %d\n", ret);
		return ret;
	}

	nbits = tm16xx_led_nbits(display);
	display->state = devm_bitmap_zalloc(display->dev, nbits, GFP_KERNEL);
	if (!display->state)
		return -ENOMEM;

	mutex_init(&display->lock);
	INIT_WORK(&display->flush_init, tm16xx_display_flush_init);
	INIT_WORK(&display->flush_display, tm16xx_display_flush_data);

	main->name = TM16XX_DEVICE_NAME;
	main->brightness = display->controller->max_brightness;
	main->max_brightness = display->controller->max_brightness;
	device_property_read_string(dev, "label", &main->name);
	device_property_read_u32(dev, "max-brightness", &main->max_brightness);
	if (main->max_brightness > display->controller->max_brightness)
		main->max_brightness = display->controller->max_brightness;
	device_property_read_u32(dev, "default-brightness", &main->brightness);
	if (main->brightness > main->max_brightness)
		main->brightness = main->max_brightness;
	main->brightness_set = tm16xx_brightness_set;
	main->groups = tm16xx_main_led_groups;
	main->flags = LED_RETAIN_AT_SHUTDOWN | LED_CORE_SUSPENDRESUME;

	ret = devm_led_classdev_register(dev, &display->main_led);
	if (ret < 0) {
		dev_err(dev, "Failed to register main LED: %d\n", ret);
		return ret;
	}

	i = 0;
	leds_node = device_get_named_child_node(dev, "leds");
	fwnode_for_each_child_node(leds_node, child) {
		struct tm16xx_led *led = &display->leds[i];
		struct led_init_data led_init = {
			.fwnode = child,
			.devicename = dev_name(display->main_led.dev),
			.devname_mandatory = true,
			.default_label = "led",
		};
		led->cdev.max_brightness = 1;
		led->cdev.brightness_set = tm16xx_led_set;
		led->cdev.flags = LED_RETAIN_AT_SHUTDOWN |
				  LED_CORE_SUSPENDRESUME;

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &led_init);
		if (ret < 0) {
			fwnode_handle_put(child);
			dev_err(dev, "Failed to register LED %s: %d\n",
				led->cdev.name, ret);
			return ret;
		}

		i++;
	}

	ret = tm16xx_display_init(display);
	if (ret < 0) {
		dev_err(display->dev, "Failed to initialize display: %d\n", ret);
		return ret;
	}

	ret = tm16xx_keypad_probe(display);
	if (ret < 0)
		dev_warn(display->dev, "Failed to initialize keypad: %d\n", ret);

	return 0;
}
EXPORT_SYMBOL_NS(tm16xx_probe, TM16XX);

/**
 * tm16xx_remove() - Remove display, unregister LEDs, blank output
 * @display: pointer to tm16xx_display
 */
void tm16xx_remove(struct tm16xx_display *display)
{
	unsigned int nbits = tm16xx_led_nbits(display);
	struct tm16xx_led *led;

	dev_dbg(display->dev, "Removing display\n");

	for (int i = 0; i < display->num_leds; i++) {
		led = &display->leds[i];
		devm_led_classdev_unregister(display->dev, &led->cdev);
	}
	devm_led_classdev_unregister(display->dev, &display->main_led);

	bitmap_zero(display->state, nbits);
	schedule_work(&display->flush_display);
	flush_work(&display->flush_display);

	display->main_led.brightness = LED_OFF;
	schedule_work(&display->flush_init);
	flush_work(&display->flush_init);

	dev_info(display->dev, "Display turned off\n");
}
EXPORT_SYMBOL_NS(tm16xx_remove, TM16XX);

MODULE_AUTHOR("Jean-François Lessard");
MODULE_DESCRIPTION("TM16xx LED Display Controllers");
MODULE_LICENSE("GPL");
