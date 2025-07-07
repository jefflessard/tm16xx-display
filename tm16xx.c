
// SPDX-License-Identifier: GPL-2.0
/*
 * Auxiliary Display Driver for TM16XX and compatible LED controllers
 *
 * Copyright (C) 2024 Jean-François Lessard
 *
 * This driver supports various LED controller chips, including
 * TM16XX family, FD6XX family, PT6964, and HBS658.
 * It provides support for both I2C and SPI interfaces.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/map_to_7segment.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>

#define TM16XX_DRIVER_NAME "tm16xx"
#define TM16XX_DEVICE_NAME "display"
#define TM16XX_DIGIT_SEGMENTS 7
#define TM16XX_LED_MAX_GRIDS  8
#define TM16XX_LED_MAX_SEGS  16
#define TM16XX_LED_BITS (TM16XX_LED_MAX_GRIDS * TM16XX_LED_MAX_SEGS)
#define TM16XX_KEY_MAX_ROWS   8
#define TM16XX_KEY_MAX_COLS  16
#define TM16XX_KEY_BITS (TM16XX_KEY_MAX_ROWS * TM16XX_KEY_MAX_COLS)

/* Common bit field definitions */

/* Command type bits (bits 7-6) */
#define TM16XX_CMD_TYPE_MASK    GENMASK(7, 6)
#define TM16XX_CMD_MODE         0
#define TM16XX_CMD_DATA         BIT(6)
#define TM16XX_CMD_CTRL         BIT(7)
#define TM16XX_CMD_ADDR         (BIT(7) | BIT(6))
#define TM16XX_CMD_READ         (TM16XX_CMD_DATA | TM16XX_DATA_WRITE)

/* Mode command grid settings (bits 1-0) */
#define TM16XX_MODE_GRID_MASK   GENMASK(1, 0)
#define TM16XX_MODE_4GRIDS      0
#define TM16XX_MODE_5GRIDS      BIT(0)
#define TM16XX_MODE_6GRIDS      BIT(1)
#define TM16XX_MODE_7GRIDS      (BIT(1) | BIT(0))

/* Data command settings */
#define TM16XX_DATA_ADDR_MASK   GENMASK(2, 2)
#define TM16XX_DATA_ADDR_AUTO   0
#define TM16XX_DATA_ADDR_FIXED  BIT(2)
#define TM16XX_DATA_MODE_MASK   GENMASK(1, 0)
#define TM16XX_DATA_WRITE       0
#define TM16XX_DATA_READ        BIT(1)

/* Control command settings */
#define TM16XX_CTRL_ON          BIT(3)
#define TM16XX_CTRL_BR_MASK     GENMASK(2, 0)

/* TM1618 specific constants */
#define TM1618_BYTE1_MASK       GENMASK(4, 0)
#define TM1618_BYTE1_RSHIFT     0
#define TM1618_BYTE2_MASK       GENMASK(7, 5)
#define TM1618_BYTE2_RSHIFT     2

/* TM1628 specific constants */
#define TM1628_BYTE1_MASK       GENMASK(7, 0)
#define TM1628_BYTE1_RSHIFT     0
#define TM1628_BYTE2_MASK       GENMASK(13, 8)
#define TM1628_BYTE2_RSHIFT     2

/* FD620 specific constants */
#define FD620_BYTE1_MASK        GENMASK(6, 0)
#define FD620_BYTE1_RSHIFT      0
#define FD620_BYTE2_MASK        GENMASK(7, 7)
#define FD620_BYTE2_RSHIFT      2

/* I2C controller addresses and control settings */
#define TM1650_CMD_CTRL         0x48
#define TM1650_CMD_READ         0x4F
#define TM1650_CMD_ADDR         0x68
#define TM1650_CTRL_BR_MASK     GENMASK(6, 4)
#define TM1650_CTRL_ON          BIT(0)
#define TM1650_CTRL_SLEEP       BIT(2)
#define TM1650_CTRL_SEG_MASK    GENMASK(3, 3)
#define TM1650_CTRL_SEG8_MODE   0
#define TM1650_CTRL_SEG7_MODE   BIT(3)
#define TM1650_KEY_ROW_MASK     GENMASK(1, 0)
#define TM1650_KEY_COL_MASK     GENMASK(5, 3)
#define TM1650_KEY_DOWN_MASK    GENMASK(6, 6)
#define TM1650_KEY_COMBINED     GENMASK(5, 3)

#define FD655_CMD_CTRL          0x48
#define FD655_CMD_ADDR          0x66
#define FD655_CTRL_BR_MASK      GENMASK(6, 5)
#define FD655_CTRL_ON           BIT(0)

#define FD6551_CMD_CTRL         0x48
#define FD6551_CTRL_BR_MASK     GENMASK(3, 1)
#define FD6551_CTRL_ON          BIT(0)

#define HBS658_KEY_COL_MASK     GENMASK(7, 5)

#define TM16XX_CTRL_BRIGHTNESS(enabled, value, prefix) \
	((enabled) ? \
		(FIELD_PREP(prefix##_CTRL_BR_MASK, (value)) | prefix##_CTRL_ON) : \
		0)

#define TM16XX_SET_LED(display, grid, segment, val) \
	(assign_bit( \
		((grid) * TM16XX_LED_MAX_SEGS + (segment)), \
		((display)->data), (val)))

#define TM16XX_SET_KEY(bitmap, row, col, val) \
	(assign_bit( \
		((row) * TM16XX_KEY_MAX_COLS + (col)), \
		(bitmap), (val)))

static char *default_value;
module_param(default_value, charp, 0644);
MODULE_PARM_DESC(default_value, "Default display value to initialize");

static SEG7_CONVERSION_MAP(map_seg7, MAP_ASCII7SEG_ALPHANUM);

/* Forward declarations */
struct tm16xx_display;

/**
 * DOC: struct tm16xx_controller - Controller-specific operations
 * @max_grids: Maximum number of grids supported by the controller
 * @max_segments: Maximum number of segments supported by the controller
 * @max_brightness: Maximum brightness level supported by the controller
 * @max_key_rows: Maximum number of key input rows supported by the controller
 * @max_key_cols: Maximum number of key input columns supported by the controller
 * @init: Configures the controller mode and brightness
 * @data: Writes display data to the controller
 * @keys: Reads controller key state into bitmap
 *
 * This structure holds function pointers for controller-specific operations.
 */
struct tm16xx_controller {
	const u8 max_grids;
	const u8 max_segments;
	const u8 max_brightness;
	const u8 max_key_rows;
	const u8 max_key_cols;
	int (* const init)(struct tm16xx_display *display);
	int (* const data)(struct tm16xx_display *display, u8 index, u16 data);
	int (* const keys)(struct tm16xx_display *display, unsigned long *bitmap);
};

/**
 * struct tm16xx_led - LED information
 * @cdev: LED class device
 * @grid: Controller grid index of the LED
 * @segment: Controller segment index of the LED
 */
struct tm16xx_led {
	struct led_classdev cdev;
	u8 grid;
	u8 segment;
};

/**
 * struct tm16xx_digit_segment - digit 7-segment to controller addressing
 * @grid: Controller grid index of the digit segment
 * @segment: Controller segment index of the digit segment
 */
struct tm16xx_digit_segment {
	u8 grid;
	u8 segment;
};

/**
 * struct tm16xx_digit - Digit information
 * @segments: Array of digit 7-segments to controller mapping
 * @value: Current char value of the digit
 */
struct tm16xx_digit {
	struct tm16xx_digit_segment segments[TM16XX_DIGIT_SEGMENTS];
	char value;
};

/**
 * struct tm16xx_display - Main driver structure
 * @dev: Pointer to device structure
 * @controller: Pointer to controller-specific operations
 * @client: Union of I2C and SPI client structures
 * @num_grids: Number of controller grids used
 * @num_segments: Number of controller segments used
 * @main_led: LED class device for the entire display
 * @leds: Array of individual LED icons
 * @num_leds: Number of individual LED icons
 * @digits: Array of 7-segments digits
 * @num_digits: Number of 7-segment digits
 * @flush_init: Work structure for brightness update
 * @flush_display: Work structure for display update
 * @flush_status: Result of the last flush work
 * @lock: Mutex for concurrent access protection
 * @data: Display data bitmap
 */
struct tm16xx_display {
	struct device *dev;
	const struct tm16xx_controller *controller;
	union {
		struct i2c_client *i2c;
		struct spi_device *spi;
	} client;
	u8 num_grids;
	u8 num_segments;
	struct led_classdev main_led;
	struct tm16xx_led *leds;
	u8 num_leds;
	struct tm16xx_digit *digits;
	u8 num_digits;
	struct work_struct flush_init;
	struct work_struct flush_display;
	int flush_status;
	struct mutex lock; /* prevents concurrent work operations */
	DECLARE_BITMAP(data, TM16XX_KEY_BITS);
};

struct tm16xx_keypad {
	struct tm16xx_display *display;
	struct input_dev *input;
	u8 row_shift;
	DECLARE_BITMAP(last_state, TM16XX_KEY_BITS);
};

static void tm16xx_keypad_poll(struct input_dev *input)
{
	struct tm16xx_keypad *keypad = input_get_drvdata(input);
	const unsigned short *keycodes = keypad->input->keycode;
	DECLARE_BITMAP(new_state, TM16XX_KEY_BITS) = {0};
	DECLARE_BITMAP(change, TM16XX_KEY_BITS);
	unsigned int bit_pos;
	u8 row, col;
	bool pressed;
	int ret;

	mutex_lock(&keypad->display->lock);
	ret = keypad->display->controller->keys(keypad->display, new_state);
	mutex_unlock(&keypad->display->lock);

	if (ret < 0) {
		dev_err(keypad->display->dev, "Reading failed: %d\n", ret);
		return;
	}

	bitmap_xor(change, new_state, keypad->last_state, TM16XX_KEY_BITS);

	for_each_set_bit(bit_pos, change, TM16XX_KEY_BITS) {
		row = bit_pos / TM16XX_KEY_MAX_COLS;
		col = bit_pos % TM16XX_KEY_MAX_COLS;
		pressed = test_bit(bit_pos, new_state);
		u16 scancode = MATRIX_SCAN_CODE(row, col, keypad->row_shift);

		dev_dbg(keypad->display->dev, "key changed: %u, row=%u col=%u down=%d\n", bit_pos, row, col, pressed);

		input_event(keypad->input, EV_MSC, MSC_SCAN, scancode);
		input_report_key(keypad->input, keycodes[scancode], pressed);
		input_sync(keypad->input);
	}

	bitmap_copy(keypad->last_state, new_state, TM16XX_KEY_BITS);
}

static int tm16xx_keypad_probe(struct tm16xx_display *display)
{
	const u8 rows = display->controller->max_key_rows;
	const u8 cols = display->controller->max_key_cols;
	struct tm16xx_keypad *keypad;
	struct input_dev *input;
	unsigned int poll_interval;
	int ret = 0;

	dev_dbg(display->dev, "Configuring keypad\n");

#ifdef TM16XX_I2C_NOSTART
	if (!i2c_check_functionality(display->client.i2c->adapter, I2C_FUNC_PROTOCOL_MANGLING)) {
		dev_err(display->dev, "No I2C_FUNC_NOPROTOCOL_MANGLING support, consider using i2c-gpio\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(display->client.i2c->adapter, I2C_FUNC_NOSTART)) {
		dev_err(display->dev, "No I2C_FUNC_NOSTART support, consider using i2c-gpio\n");
		return -EINVAL;
	}
#endif

	ret = device_property_read_u32(display->dev, "poll-interval", &poll_interval);
	if (ret < 0) {
		dev_err(display->dev, "Failed to read poll-interval: %d\n", ret);
		return ret;
	}
	dev_dbg(display->dev, "Polling interval: %u ms\n", poll_interval);

	keypad = devm_kzalloc(display->dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;
	keypad->display = display;

	input = devm_input_allocate_device(display->dev);
	if (!input) {
		dev_err(display->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto free_keypad;
	}
	input->name = TM16XX_DRIVER_NAME"-keypad";
	input_setup_polling(input, tm16xx_keypad_poll);
	input_set_poll_interval(input, poll_interval);
	keypad->input = input;
	input_set_drvdata(input, keypad);

	keypad->row_shift = get_count_order(cols);
	dev_dbg(display->dev, "Number of keypad rows=%u, cols=%u\n", rows, cols);

	ret = matrix_keypad_build_keymap(NULL, NULL, rows, cols, NULL, input);
	if (ret < 0) {
		dev_err(display->dev, "Failed to build keymap: %d\n", ret);
		goto free_input;
	}

	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(display->dev, "Failed to register input device: %d\n", ret);
		goto free_input;
	}

	return 0;

free_input:
	input_free_device(input);
free_keypad:
	devm_kfree(display->dev, keypad);
	return ret;
}

/**
 * tm16xx_display_flush_init() - Controller configuration Work function
 * @work: Pointer to work_struct
 */
static void tm16xx_display_flush_init(struct work_struct *work)
{
	struct tm16xx_display *display = container_of(work,
			struct tm16xx_display, flush_init);
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
 * tm16xx_display_flush_data() - Work function to update display data
 * @work: Pointer to work_struct
 */
static void tm16xx_display_flush_data(struct work_struct *work)
{
	struct tm16xx_display *display = container_of(work,
			struct tm16xx_display, flush_display);
	int i, ret = 0;
	u16 *data = (u16 *)display->data;

	mutex_lock(&display->lock);
	dev_dbg(display->dev, "Sending data to controller\n");

	if (display->controller->data) {
		for (i = 0; i < display->num_grids; i++) {
			ret = display->controller->data(display, i, data[i]);
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
 * tm16xx_display_remove() - Remove the display
 * @display: Pointer to tm16xx_display structure
 */
static void tm16xx_display_remove(struct tm16xx_display *display)
{
	dev_dbg(display->dev, "Removing display\n");

	memset(display->data, 0x00, sizeof(display->data));
	schedule_work(&display->flush_display);
	flush_work(&display->flush_display);

	display->main_led.brightness = LED_OFF;
	schedule_work(&display->flush_init);
	flush_work(&display->flush_init);

	dev_info(display->dev, "Display turned off\n");
}

/**
 * tm16xx_brightness_set() - Set brightness of the display
 * @led_cdev: Pointer to led_classdev
 * @brightness: Brightness value to set
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
 * tm16xx_led_set() - Set state of an individual LED
 * @led_cdev: Pointer to led_classdev
 * @value: Value to set (on/off)
 */
static void tm16xx_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct tm16xx_led *led = container_of(led_cdev, struct tm16xx_led, cdev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	dev_dbg(display->dev, "Setting led %u,%u to %d\n", led->grid, led->segment, value);

	TM16XX_SET_LED(display, led->grid, led->segment, value);
	schedule_work(&display->flush_display);
}

/**
 * tm16xx_value_show() - Show current display value
 * @dev: Pointer to device structure
 * @attr: Pointer to device attribute structure
 * @buf: Buffer to write the display value
 *
 * Return: Number of bytes written to buffer
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
 * tm16xx_value_store() - Store new display value
 * @dev: Pointer to device structure
 * @attr: Pointer to device attribute structure
 * @buf: Buffer containing the new display value
 * @count: Number of bytes in buffer
 *
 * Return: Number of bytes written or negative error code
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
			TM16XX_SET_LED(display, ds->grid, ds->segment, val);
		}
	}

	schedule_work(&display->flush_display);
	return count;
}

/**
 * tm16xx_num_digits_show() - Show the number of digits in the display
 * @dev: The device struct
 * @attr: The device_attribute struct
 * @buf: The output buffer
 *
 * This function returns the number of digits in the display.
 *
 * Return: Number of bytes written to buf
 */
static ssize_t tm16xx_num_digits_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct tm16xx_display *display = dev_get_drvdata(led_cdev->dev->parent);

	return sprintf(buf, "%u\n", display->num_digits);
}

/**
 * tm16xx_map_seg7_show() - Show the current 7-segment character map
 * @dev: The device struct
 * @attr: The device_attribute struct
 * @buf: The output buffer
 *
 * This function returns the current 7-segment character map.
 *
 * Return: Number of bytes written to buf
 */
static ssize_t tm16xx_map_seg7_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	memcpy(buf, &map_seg7, sizeof(map_seg7));
	return sizeof(map_seg7);
}

/**
 * tm16xx_map_seg7_store() - Set a new 7-segment character map
 * @dev: The device struct
 * @attr: The device_attribute struct
 * @buf: The input buffer
 * @cnt: Number of bytes in buf
 *
 * This function sets a new 7-segment character map.
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
 * tm16xx_display_init() - Initialize the display
 * @display: Pointer to tm16xx_display structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int tm16xx_display_init(struct tm16xx_display *display)
{
	dev_dbg(display->dev, "Initializing display\n");
	schedule_work(&display->flush_init);
	flush_work(&display->flush_init);
	if (display->flush_status < 0)
		return display->flush_status;

	if (default_value && strlen(default_value) > 0) {
		tm16xx_value_store(display->main_led.dev, NULL, default_value,
				   strlen(default_value));
	} else {
		memset(display->data, 0xFF, sizeof(display->data));
		schedule_work(&display->flush_display);
		flush_work(&display->flush_display);
		memset(display->data, 0x00, sizeof(display->data));
		if (display->flush_status < 0)
			return display->flush_status;
	}

	dev_info(display->dev, "Display turned on\n");

	return 0;
}

/**
 * tm16xx_parse_dt() - Parse device tree data
 * @dev: Pointer to device structure
 * @display: Pointer to tm16xx_display structure
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

				ret = fwnode_property_read_u32(child, "reg", reg);
				if (ret < 0) {
					fwnode_handle_put(child);
					fwnode_handle_put(digits_node);
					return ret;
				}

				ret = fwnode_property_read_u32_array(child,
						"segments", segments,
						TM16XX_DIGIT_SEGMENTS * 2);
				if (ret < 0) {
					fwnode_handle_put(child);
					fwnode_handle_put(digits_node);
					return ret;
				}

				for (j = 0; j < TM16XX_DIGIT_SEGMENTS; ++j) {
					digit->segments[j].grid = segments[2 * j];
					digit->segments[j].segment = segments[2 * j + 1];
					max_grid = umax(max_grid, digit->segments[j].grid);
					max_segment = umax(max_segment, digit->segments[j].segment);
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
				ret = fwnode_property_read_u32_array(child, "reg", reg, 2);
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
		dev_err(dev, "grid %u exceeds controller max_grids %u\n", max_grid,
			display->controller->max_grids);
		return -EINVAL;
	}

	if (max_segment >= display->controller->max_segments) {
		dev_err(dev, "segment %u exceeds controller max_segments %u\n", max_segment,
			display->controller->max_segments);
		return -EINVAL;
	}

	display->num_grids = max_grid + 1;
	display->num_segments = max_segment + 1;

	dev_dbg(dev, "Number of grids: %u\n", display->num_grids);
	dev_dbg(dev, "Number of segments: %u\n", display->num_segments);

	return 0;
}

/**
 * tm16xx_probe() - Probe function for tm16xx devices
 * @display: Pointer to tm16xx_display structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int tm16xx_probe(struct tm16xx_display *display)
{
	struct device *dev = display->dev;
	struct fwnode_handle *leds_node, *child;
	int ret, i;

	dev_dbg(dev, "Probing device\n");
	ret = tm16xx_parse_dt(dev, display);
	if (ret < 0) {
		dev_err(dev, "Failed to parse device tree: %d\n", ret);
		return ret;
	}

	mutex_init(&display->lock);
	INIT_WORK(&display->flush_init, tm16xx_display_flush_init);
	INIT_WORK(&display->flush_display, tm16xx_display_flush_data);

	display->main_led.name = TM16XX_DEVICE_NAME;
	display->main_led.brightness = display->controller->max_brightness;
	display->main_led.max_brightness = display->controller->max_brightness;
	display->main_led.brightness_set = tm16xx_brightness_set;
	display->main_led.groups = tm16xx_main_led_groups;
	display->main_led.flags = LED_RETAIN_AT_SHUTDOWN | LED_CORE_SUSPENDRESUME;

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
			.devicename = display->main_led.name,
			.devname_mandatory = true,
		};
		led->cdev.max_brightness = 1;
		led->cdev.brightness_set = tm16xx_led_set;
		led->cdev.flags = LED_RETAIN_AT_SHUTDOWN | LED_CORE_SUSPENDRESUME;

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &led_init);
		if (ret < 0) {
			fwnode_handle_put(child);
			dev_err(dev,
				"Failed to register LED %s: %d\n",
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

	if (display->controller->keys && display->controller->max_key_rows &&
	    display->controller->max_key_cols) {
		ret = tm16xx_keypad_probe(display);
		if (ret < 0)
			dev_warn(display->dev,
				 "Failed to initialize keypad: %d\n", ret);
	}

	return 0;
}

/* SPI specific code */
#if IS_ENABLED(CONFIG_SPI_MASTER)
static int tm16xx_spi_probe(struct spi_device *spi)
{
	const struct tm16xx_controller *controller;
	struct tm16xx_display *display;
	int ret;

	controller = spi_get_device_match_data(spi);
	if (!controller)
		return -EINVAL;

	display = devm_kzalloc(&spi->dev, sizeof(*display), GFP_KERNEL);
	if (!display)
		return -ENOMEM;

	display->client.spi = spi;
	display->dev = &spi->dev;
	display->controller = controller;

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

static int tm16xx_spi_read(struct tm16xx_display *display, u8 cmd, u8 *data, size_t len)
{
	dev_dbg(display->dev, "spi_write %*ph", (char)len, data);

	struct spi_device *spi = display->client.spi;

	return spi_write_then_read(spi, &cmd, 1, data, len);
}

/**
 * tm16xx_spi_write() - Write data to SPI client
 * @display: Pointer to tm16xx_display structure
 * @data: Data to write
 * @len: Length of data
 *
 * Return: Number of bytes written or negative error code
 */
static int tm16xx_spi_write(struct tm16xx_display *display, u8 *data, size_t len)
{
	dev_dbg(display->dev, "spi_write %*ph", (char)len, data);

	struct spi_device *spi = display->client.spi;

	return spi_write(spi, data, len);
}

/* SPI controller-specific functions */
static int tm1628_init(struct tm16xx_display *display)
{
	const enum led_brightness brightness = display->main_led.brightness;
	const u8 num_grids = display->num_grids;
	u8 cmd;
	int ret;

	/* Set mode command based on grid count */
	cmd = TM16XX_CMD_MODE;
	if (num_grids <= 4)
		cmd |= TM16XX_MODE_4GRIDS;
	else if (num_grids == 5)
		cmd |= TM16XX_MODE_5GRIDS;
	else if (num_grids == 6)
		cmd |= TM16XX_MODE_6GRIDS;
	else
		cmd |= TM16XX_MODE_7GRIDS;

	ret = tm16xx_spi_write(display, &cmd, 1);
	if (ret < 0)
		return ret;

	/* Set data command */
	cmd = TM16XX_CMD_DATA | TM16XX_DATA_ADDR_AUTO | TM16XX_DATA_WRITE;
	ret = tm16xx_spi_write(display, &cmd, 1);
	if (ret < 0)
		return ret;

	/* Set control command with brightness */
	cmd = TM16XX_CMD_CTRL |
	      TM16XX_CTRL_BRIGHTNESS(brightness, brightness - 1, TM16XX);
	ret = tm16xx_spi_write(display, &cmd, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int tm1618_data(struct tm16xx_display *display, u8 index, u16 data)
{
	u8 cmds[3];

	cmds[0] = TM16XX_CMD_ADDR + index * 2;
	cmds[1] = (data & TM1618_BYTE1_MASK) >> TM1618_BYTE1_RSHIFT;
	cmds[2] = (data & TM1618_BYTE2_MASK) >> TM1618_BYTE2_RSHIFT;

	return tm16xx_spi_write(display, cmds, ARRAY_SIZE(cmds));
}

static int tm1628_data(struct tm16xx_display *display, u8 index, u16 data)
{
	u8 cmds[3];

	cmds[0] = TM16XX_CMD_ADDR + index * 2;
	cmds[1] = (data & TM1628_BYTE1_MASK) >> TM1628_BYTE1_RSHIFT;
	cmds[2] = (data & TM1628_BYTE2_MASK) >> TM1628_BYTE2_RSHIFT;

	return tm16xx_spi_write(display, cmds, ARRAY_SIZE(cmds));
}

static int tm1628_keys(struct tm16xx_display *display, unsigned long *bitmap)
{
	u8 keycodes[5];
	int ret, i;

	ret = tm16xx_spi_read(display, TM16XX_CMD_READ, keycodes, sizeof(keycodes));
	if (ret)
		return ret;

	for (i = 0; i < sizeof(keycodes); i++) {
		TM16XX_SET_KEY(bitmap, 0, i * 2, keycodes[i] & BIT(0));
		TM16XX_SET_KEY(bitmap, 1, i * 2, keycodes[i] & BIT(1));
		TM16XX_SET_KEY(bitmap, 0, i * 2 + 1, keycodes[i] & BIT(3));
		TM16XX_SET_KEY(bitmap, 1, i * 2 + 1, keycodes[i] & BIT(4));
	}

	return 0;
}

static int tm1618_keys(struct tm16xx_display *display, unsigned long *bitmap)
{
	u8 keycodes[3];
	int ret;

	ret = tm16xx_spi_read(display, TM16XX_CMD_READ, keycodes, sizeof(keycodes));
	if (ret)
		return ret;

	TM16XX_SET_KEY(bitmap, 0, 0, keycodes[0] & BIT(1));
	TM16XX_SET_KEY(bitmap, 0, 1, keycodes[0] & BIT(4));
	TM16XX_SET_KEY(bitmap, 0, 2, keycodes[1] & BIT(1));
	TM16XX_SET_KEY(bitmap, 0, 3, keycodes[1] & BIT(4));
	TM16XX_SET_KEY(bitmap, 0, 4, keycodes[2] & BIT(1));

	return 0;
}

static int fd620_data(struct tm16xx_display *display, u8 index, u16 data)
{
	u8 cmds[3];

	cmds[0] = TM16XX_CMD_ADDR + index * 2;
	cmds[1] = (data & FD620_BYTE1_MASK) >> FD620_BYTE1_RSHIFT;
	cmds[2] = (data & FD620_BYTE2_MASK) >> FD620_BYTE2_RSHIFT;

	return tm16xx_spi_write(display, cmds, ARRAY_SIZE(cmds));
}

static int fd620_keys(struct tm16xx_display *display, unsigned long *bitmap)
{
	u8 keycodes[4];
	int ret;

	ret = tm16xx_spi_read(display, TM16XX_CMD_READ, keycodes, sizeof(keycodes));
	if (ret)
		return ret;

	TM16XX_SET_KEY(bitmap, 0, 0, keycodes[0] & BIT(0));
	TM16XX_SET_KEY(bitmap, 0, 1, keycodes[0] & BIT(3));
	TM16XX_SET_KEY(bitmap, 0, 2, keycodes[1] & BIT(0));
	TM16XX_SET_KEY(bitmap, 0, 3, keycodes[1] & BIT(3));
	TM16XX_SET_KEY(bitmap, 0, 4, keycodes[2] & BIT(0));
	TM16XX_SET_KEY(bitmap, 0, 5, keycodes[2] & BIT(3));
	TM16XX_SET_KEY(bitmap, 0, 6, keycodes[3] & BIT(0));

	return 0;
}


/* SPI controller definitions */
static const struct tm16xx_controller tm1618_controller = {
	.max_grids = 7,
	.max_segments = 8,
	.max_brightness = 8,
	.max_key_rows = 1,
	.max_key_cols = 5,
	.init = tm1628_init,
	.data = tm1618_data,
	.keys = tm1618_keys,
};

static const struct tm16xx_controller tm1620_controller = {
	.max_grids = 6,
	.max_segments = 10,
	.max_brightness = 8,
	.max_key_rows = 0,
	.max_key_cols = 0,
	.init = tm1628_init,
	.data = tm1628_data,
	.keys = NULL,
};

static const struct tm16xx_controller tm1628_controller = {
	.max_grids = 7,
	.max_segments = 14, // seg 11 unused
	.max_brightness = 8,
	.max_key_rows = 2,
	.max_key_cols = 10,
	.init = tm1628_init,
	.data = tm1628_data,
	.keys = tm1628_keys,
};

static const struct tm16xx_controller fd620_controller = {
	.max_grids = 5,
	.max_segments = 8,
	.max_brightness = 8,
	.max_key_rows = 1,
	.max_key_cols = 7,
	.init = tm1628_init,
	.data = fd620_data,
	.keys = fd620_keys,
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tm16xx_spi_of_match[] = {
	{ .compatible = "titanmec,tm1618",  .data = &tm1618_controller },
	{ .compatible = "titanmec,tm1620",  .data = &tm1620_controller },
	{ .compatible = "titanmec,tm1628",  .data = &tm1628_controller },
	{ .compatible = "fdhisi,fd620",     .data = &fd620_controller },
	{ .compatible = "fdhisi,fd628",     .data = &tm1628_controller },
	{ .compatible = "icore,aip1618",    .data = &tm1618_controller },
	{ .compatible = "icore,aip1628",    .data = &tm1628_controller },
	{ .compatible = "princeton,pt6964", .data = &tm1628_controller },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_spi_of_match);
#endif

static const struct spi_device_id tm16xx_spi_id[] = {
	{ "tm1618",  (kernel_ulong_t)&tm1618_controller },
	{ "tm1620",  (kernel_ulong_t)&tm1620_controller },
	{ "tm1628",  (kernel_ulong_t)&tm1628_controller },
	{ "fd620",   (kernel_ulong_t)&fd620_controller },
	{ "fd628",   (kernel_ulong_t)&tm1628_controller },
	{ "aip1618", (kernel_ulong_t)&tm1618_controller },
	{ "aip1628", (kernel_ulong_t)&tm1628_controller },
	{ "pt6964",  (kernel_ulong_t)&tm1628_controller },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, tm16xx_spi_id);

static struct spi_driver tm16xx_spi_driver = {
	.driver = {
		.name = TM16XX_DRIVER_NAME,
		.of_match_table = of_match_ptr(tm16xx_spi_of_match),
	},
	.probe = tm16xx_spi_probe,
	.remove = tm16xx_spi_remove,
	.shutdown = tm16xx_spi_remove,
	.id_table = tm16xx_spi_id,
};

static int tm16xx_spi_register(void)
{
	return spi_register_driver(&tm16xx_spi_driver);
}

static void tm16xx_spi_unregister(void)
{
	spi_unregister_driver(&tm16xx_spi_driver);
}
#else
static int tm16xx_spi_register(void)
{
	return 0;
}

static void tm16xx_spi_unregister(void)
{
}
#endif /* CONFIG_SPI_MASTER */

/* I2C specific code */
#if IS_ENABLED(CONFIG_I2C)
static int tm16xx_i2c_probe(struct i2c_client *client)
{
	const struct tm16xx_controller *controller;
	struct tm16xx_display *display;
	int ret;

	controller = i2c_get_match_data(client);
	if (!controller)
		return -EINVAL;

	display = devm_kzalloc(&client->dev, sizeof(*display), GFP_KERNEL);
	if (!display)
		return -ENOMEM;

	display->client.i2c = client;
	display->dev = &client->dev;
	display->controller = controller;

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

/**
 * tm16xx_i2c_write() - Write data to I2C client
 * @display: Pointer to tm16xx_display structure
 * @data: Data to write
 * @len: Length of data
 *
 * Return: Number of bytes written or negative error code
 */
static int tm16xx_i2c_write(struct tm16xx_display *display, u8 *data, size_t len)
{
	dev_dbg(display->dev, "i2c_write %*ph", (char)len, data);

	/* expected sequence: S Command [A] Data [A] P */
#ifdef TM16XX_I2C_NOSTART
	struct i2c_msg msg = {
		.flags = I2C_M_NOSTART,
		.len = len,
		.buf = data,
	};
#else
	struct i2c_msg msg = {
		.addr = data[0] >> 1,
		.flags = 0,
		.len = len - 1,
		.buf = &data[1],
	};
#endif
	int ret;

	ret = i2c_transfer(display->client.i2c->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return (ret == 1) ? len : -EIO;
}

static int tm16xx_i2c_read(struct tm16xx_display *display, u8 cmd, u8 *data, size_t len)
{
	/* expected sequence: S Command [A] [Data] [A] P */
#ifdef TM16XX_I2C_NOSTART
	struct i2c_msg msgs[2] = {
		{
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &cmd,
		}, {
			.flags = I2C_M_NOSTART | I2C_M_RD | I2C_M_NO_RD_ACK,
			.len = len,
			.buf = data,
		}
	};
#else
	struct i2c_msg msgs[1] = {
		{
			.addr = cmd >> 1,
			.flags = I2C_M_RD | I2C_M_NO_RD_ACK,
			.len = len,
			.buf = data,
		}
	};
#endif
	int ret;

	ret = i2c_transfer(display->client.i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	dev_dbg(display->dev, "i2c_read %ph: %*ph\n", &cmd, (char)len, data);

	return (ret == ARRAY_SIZE(msgs)) ? len : -EIO;
}

/* I2C controller-specific functions */
static int tm1650_init(struct tm16xx_display *display)
{
	u8 cmds[2];
	const enum led_brightness brightness = display->main_led.brightness;

	cmds[0] = TM1650_CMD_CTRL;
	cmds[1] = TM16XX_CTRL_BRIGHTNESS(brightness, brightness, TM1650) |
		  TM1650_CTRL_SEG8_MODE;

	return tm16xx_i2c_write(display, cmds, ARRAY_SIZE(cmds));
}

static int tm1650_data(struct tm16xx_display *display, u8 index, u16 data)
{
	u8 cmds[2];

	cmds[0] = TM1650_CMD_ADDR + index * 2;
	cmds[1] = data; // SEG 1 to 8

	return tm16xx_i2c_write(display, cmds, ARRAY_SIZE(cmds));
}

static int tm1650_keys(struct tm16xx_display *display, unsigned long *bitmap)
{
	u8 keycode, row, col;
	bool pressed;
	int ret;

	ret = tm16xx_i2c_read(display, TM1650_CMD_READ, &keycode, 1);
	if (ret)
		return ret;

	if (keycode == 0x00 || keycode == 0xFF) {
		return -EINVAL;
	}

	row = FIELD_GET(TM1650_KEY_ROW_MASK, keycode);
	pressed = FIELD_GET(TM1650_KEY_DOWN_MASK, keycode) != 0;
	if ((keycode & TM1650_KEY_COMBINED) == TM1650_KEY_COMBINED) {
		TM16XX_SET_KEY(bitmap, row, 0, pressed);
		TM16XX_SET_KEY(bitmap, row, 1, pressed);
	} else {
		col = FIELD_GET(TM1650_KEY_COL_MASK, keycode);
		TM16XX_SET_KEY(bitmap, row, col, pressed);
	}

	return 0;
}

static int fd655_init(struct tm16xx_display *display)
{
	u8 cmds[2];
	const enum led_brightness brightness = display->main_led.brightness;

	cmds[0] = FD655_CMD_CTRL;
	cmds[1] = TM16XX_CTRL_BRIGHTNESS(brightness, brightness % 3, FD655);

	return tm16xx_i2c_write(display, cmds, ARRAY_SIZE(cmds));
}

static int fd655_data(struct tm16xx_display *display, u8 index, u16 data)
{
	u8 cmds[2];

	cmds[0] = FD655_CMD_ADDR + index * 2;
	cmds[1] = data; // SEG 1 to 8

	return tm16xx_i2c_write(display, cmds, ARRAY_SIZE(cmds));
}

static int fd6551_init(struct tm16xx_display *display)
{
	u8 cmds[2];
	const enum led_brightness brightness = display->main_led.brightness;

	cmds[0] = FD6551_CMD_CTRL;
	cmds[1] = TM16XX_CTRL_BRIGHTNESS(brightness, ~(brightness - 1), FD6551);

	return tm16xx_i2c_write(display, cmds, ARRAY_SIZE(cmds));
}

static void hbs658_swap_nibbles(u8 *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
		data[i] = (data[i] << 4) | (data[i] >> 4);
}

static int hbs658_init(struct tm16xx_display *display)
{
	const enum led_brightness brightness = display->main_led.brightness;
	u8 cmd;
	int ret;

	/* Set data command */
	cmd = TM16XX_CMD_DATA | TM16XX_DATA_ADDR_AUTO | TM16XX_DATA_WRITE;
	hbs658_swap_nibbles(&cmd, 1);
	ret = tm16xx_i2c_write(display, &cmd, 1);
	if (ret < 0)
		return ret;

	/* Set control command with brightness */
	cmd = TM16XX_CMD_CTRL |
	      TM16XX_CTRL_BRIGHTNESS(brightness, brightness - 1, TM16XX);
	hbs658_swap_nibbles(&cmd, 1);
	ret = tm16xx_i2c_write(display, &cmd, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int hbs658_data(struct tm16xx_display *display, u8 index, u16 data)
{
	u8 cmds[2];

	cmds[0] = TM16XX_CMD_ADDR + index * 2;
	cmds[1] = data;

	hbs658_swap_nibbles(cmds, ARRAY_SIZE(cmds));
	return tm16xx_i2c_write(display, cmds, ARRAY_SIZE(cmds));
}

static int hbs658_keys(struct tm16xx_display *display, unsigned long *bitmap)
{
	u8 cmd, keycode, col;
	int ret;

	cmd = TM16XX_CMD_READ;
	hbs658_swap_nibbles(&cmd, 1);
	ret = tm16xx_i2c_read(display, cmd, &keycode, 1);
	if (ret)
		return ret;

	hbs658_swap_nibbles(&keycode, 1);

	if (keycode != 0xFF) {
		col = FIELD_GET(HBS658_KEY_COL_MASK, keycode);
		TM16XX_SET_KEY(bitmap, 0, col, true);
	}

	return 0;
}

/* I2C controller definitions */
static const struct tm16xx_controller tm1650_controller = {
	.max_grids = 4,
	.max_segments = 8,
	.max_brightness = 8,
	.max_key_rows = 4,
	.max_key_cols = 7,
	.init = tm1650_init,
	.data = tm1650_data,
	.keys = tm1650_keys,
};

static const struct tm16xx_controller fd655_controller = {
	.max_grids = 5,
	.max_segments = 7,
	.max_brightness = 3,
	.max_key_rows = 5,
	.max_key_cols = 7,
	.init = fd655_init,
	.data = fd655_data,
	.keys = tm1650_keys,
};

static const struct tm16xx_controller fd6551_controller = {
	.max_grids = 5,
	.max_segments = 7,
	.max_brightness = 8,
	.max_key_rows = 0,
	.max_key_cols = 0,
	.init = fd6551_init,
	.data = fd655_data,
	.keys = NULL,
};

static const struct tm16xx_controller hbs658_controller = {
	.max_grids = 5,
	.max_segments = 8,
	.max_brightness = 8,
	.max_key_rows = 1,
	.max_key_cols = 8,
	.init = hbs658_init,
	.data = hbs658_data,
	.keys = hbs658_keys,
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tm16xx_i2c_of_match[] = {
	{ .compatible = "titanmec,tm1650", .data = &tm1650_controller },
	{ .compatible = "icore,aip650",    .data = &tm1650_controller },
	{ .compatible = "fdhisi,fd650",    .data = &tm1650_controller },
	{ .compatible = "fdhisi,fd6551",   .data = &fd6551_controller },
	{ .compatible = "fdhisi,fd655",    .data = &fd655_controller },
	{ .compatible = "winrise,hbs658",   .data = &hbs658_controller },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_i2c_of_match);
#endif

static const struct i2c_device_id tm16xx_i2c_id[] = {
	{ "tm1650", (kernel_ulong_t)&tm1650_controller },
	{ "aip650", (kernel_ulong_t)&tm1650_controller },
	{ "fd650",  (kernel_ulong_t)&tm1650_controller },
	{ "fd6551", (kernel_ulong_t)&fd6551_controller },
	{ "fd655",  (kernel_ulong_t)&fd655_controller },
	{ "hbs658",  (kernel_ulong_t)&hbs658_controller },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tm16xx_i2c_id);

static struct i2c_driver tm16xx_i2c_driver = {
	.driver = {
		.name = TM16XX_DRIVER_NAME,
		.of_match_table = of_match_ptr(tm16xx_i2c_of_match),
	},
	.probe = tm16xx_i2c_probe,
	.remove = tm16xx_i2c_remove,
	.shutdown = tm16xx_i2c_remove,
	.id_table = tm16xx_i2c_id,
};

static int tm16xx_i2c_register(void)
{
	return i2c_add_driver(&tm16xx_i2c_driver);
}

static void tm16xx_i2c_unregister(void)
{
	i2c_del_driver(&tm16xx_i2c_driver);
}
#else
static int tm16xx_i2c_register(void)
{
	return 0;
}

static void tm16xx_i2c_unregister(void)
{
}
#endif /* CONFIG_I2C */

static int __init tm16xx_init(void)
{
	int ret;

	ret = tm16xx_spi_register();
	if (ret)
		return ret;

	ret = tm16xx_i2c_register();
	if (ret) {
		tm16xx_spi_unregister();
		return ret;
	}

	return 0;
}

static void __exit tm16xx_exit(void)
{
	tm16xx_i2c_unregister();
	tm16xx_spi_unregister();
}

module_init(tm16xx_init);
module_exit(tm16xx_exit);

MODULE_AUTHOR("Jean-François Lessard");
MODULE_DESCRIPTION("TM16XX Compatible LED Display Controllers");
MODULE_LICENSE("GPL");
