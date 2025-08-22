// SPDX-License-Identifier: GPL-2.0
/*
 * TM16xx and compatible LED display/keypad controller driver
 * Supports TM16xx, FD6xx, PT6964, HBS658, AIP16xx and related chips.
 *
 * Copyright (C) 2024 Jean-François Lessard
 */

#ifndef _TM16XX_H
#define _TM16XX_H

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/version.h> // TODO remove

#define TM16XX_DRIVER_NAME "tm16xx"
#define TM16XX_DEVICE_NAME "display"
#define TM16XX_DIGIT_SEGMENTS	7

/* Common bit field definitions */

/* Command type bits (bits 7-6) */
#define TM16XX_CMD_MASK		GENMASK(7, 6)
#define TM16XX_CMD_MODE		0
#define TM16XX_CMD_DATA		BIT(6)
#define TM16XX_CMD_CTRL		BIT(7)
#define TM16XX_CMD_ADDR		(BIT(7) | BIT(6))
#define TM16XX_CMD_WRITE	(TM16XX_CMD_DATA | TM16XX_DATA_MODE_WRITE)
#define TM16XX_CMD_READ		(TM16XX_CMD_DATA | TM16XX_DATA_MODE_READ)

/* Mode command grid settings (bits 1-0) */
#define TM16XX_MODE_GRID_MASK	GENMASK(1, 0)
#define TM16XX_MODE_4GRIDS	0
#define TM16XX_MODE_5GRIDS	BIT(0)
#define TM16XX_MODE_6GRIDS	BIT(1)
#define TM16XX_MODE_7GRIDS	(BIT(1) | BIT(0))

/* Data command settings */
#define TM16XX_DATA_ADDR_MASK	BIT(2)
#define TM16XX_DATA_ADDR_AUTO	0
#define TM16XX_DATA_ADDR_FIXED	BIT(2)
#define TM16XX_DATA_MODE_MASK	GENMASK(1, 0)
#define TM16XX_DATA_MODE_WRITE	0
#define TM16XX_DATA_MODE_READ	BIT(1)

/* Control command settings */
#define TM16XX_CTRL_ON		BIT(3)
#define TM16XX_CTRL_BR_MASK	GENMASK(2, 0)

/* TM1618 specific constants */
#define TM1618_BYTE1_MASK	GENMASK(4, 0)
#define TM1618_BYTE2_MASK	GENMASK(7, 5)
#define TM1618_BYTE2_SHIFT	3
#define TM1618_KEY_READ_LEN	3
#define TM1618_KEY_MASK		(BIT(4) | BIT(1))

/* TM1628 specific constants */
#define TM1628_BYTE1_MASK	GENMASK(7, 0)
#define TM1628_BYTE2_MASK	GENMASK(13, 8)
#define TM1628_KEY_READ_LEN	5
#define TM1628_KEY_MASK		(GENMASK(4, 3) | GENMASK(1, 0))

/* TM1638 specific constants */
#define TM1638_KEY_READ_LEN	4
#define TM1638_KEY_MASK		(GENMASK(6, 4) | GENMASK(2, 0))

/* FD620 specific constants */
#define FD620_BYTE1_MASK	GENMASK(6, 0)
#define FD620_BYTE2_MASK	BIT(7)
#define FD620_BYTE2_SHIFT	5
#define FD620_KEY_READ_LEN	4
#define FD620_KEY_MASK		(BIT(3) | BIT(0))

/* I2C controller addresses and control settings */
#define TM1650_CMD_CTRL		0x48
#define TM1650_CMD_READ		0x4F
#define TM1650_CMD_ADDR		0x68
#define TM1650_CTRL_BR_MASK	GENMASK(6, 4)
#define TM1650_CTRL_ON		BIT(0)
#define TM1650_CTRL_SLEEP	BIT(2)
#define TM1650_CTRL_SEG_MASK	BIT(3)
#define TM1650_CTRL_SEG8_MODE	0
#define TM1650_CTRL_SEG7_MODE	BIT(3)
#define TM1650_KEY_ROW_MASK	GENMASK(1, 0)
#define TM1650_KEY_COL_MASK	GENMASK(5, 3)
#define TM1650_KEY_DOWN_MASK	BIT(6)
#define TM1650_KEY_COMBINED	GENMASK(5, 3)

#define FD655_CMD_CTRL		0x48
#define FD655_CMD_ADDR		0x66
#define FD655_CTRL_BR_MASK	GENMASK(6, 5)
#define FD655_CTRL_ON		BIT(0)

#define FD6551_CMD_CTRL		0x48
#define FD6551_CTRL_BR_MASK	GENMASK(3, 1)
#define FD6551_CTRL_ON		BIT(0)

#define HBS658_KEY_COL_MASK	GENMASK(7, 5)

#define TM16XX_CTRL_BRIGHTNESS(enabled, value, prefix) \
	((enabled) ? (FIELD_PREP(prefix##_CTRL_BR_MASK, (value)) | \
		      prefix##_CTRL_ON) : 0)

/* Forward declarations */
struct tm16xx_display;
struct tm16xx_keypad;

/**
 * DOC: struct tm16xx_controller - Controller-specific operations and limits
 * @max_grids: Maximum number of grids supported by the controller.
 * @max_segments: Maximum number of segments supported by the controller.
 * @max_brightness: Maximum brightness level supported by the controller.
 * @max_key_rows: Maximum number of key input rows supported by the controller.
 * @max_key_cols: Maximum number of key input columns supported by the controller.
 * @init: Pointer to controller mode/brightness configuration function.
 * @data: Pointer to function writing display data to the controller.
 * @keys: Pointer to function reading controller key state into bitmap.
 *
 * Holds function pointers and limits for controller-specific operations.
 */
struct tm16xx_controller {
	const u8 max_grids;
	const u8 max_segments;
	const u8 max_brightness;
	const u8 max_key_rows;
	const u8 max_key_cols;
	int (*const init)(struct tm16xx_display *display);
	int (*const data)(struct tm16xx_display *display, u8 index, u16 data);
	int (*const keys)(struct tm16xx_keypad *keypad);
};

/**
 * struct tm16xx_led - Individual LED icon mapping
 * @cdev: LED class device for sysfs interface.
 * @grid: Controller grid index of the LED.
 * @segment: Controller segment index of the LED.
 */
struct tm16xx_led {
	struct led_classdev cdev;
	u8 grid;
	u8 segment;
};

/**
 * struct tm16xx_digit_segment - Digit segment mapping to display coordinates
 * @grid: Controller grid index for this segment.
 * @segment: Controller segment index for this segment.
 */
struct tm16xx_digit_segment {
	u8 grid;
	u8 segment;
};

/**
 * struct tm16xx_digit - 7-segment digit mapping and value
 * @segments: Array mapping each 7-segment position to a grid/segment on the controller.
 * @value: Current character value displayed on this digit.
 */
struct tm16xx_digit {
	struct tm16xx_digit_segment segments[TM16XX_DIGIT_SEGMENTS];
	char value;
};

/**
 * struct tm16xx_display - Main driver structure for the display
 * @dev: Pointer to device struct.
 * @controller: Controller-specific function table and limits.
 * @client: Union of I2C and SPI client pointers.
 * @spi_buffer: DMA-safe buffer for SPI transactions, or NULL for I2C.
 * @num_grids: Number of controller grids in use.
 * @num_segments: Number of controller segments in use.
 * @main_led: LED class device for the entire display.
 * @leds: Array of individual LED icon structures.
 * @num_leds: Number of individual LED icons.
 * @digits: Array of 7-segment digit structures.
 * @num_digits: Number of 7-segment digits.
 * @flush_init: Work struct for configuration update.
 * @flush_display: Work struct for display update.
 * @flush_status: Status/result of last flush work.
 * @lock: Mutex protecting concurrent access to work operations.
 * @state: Bitmap holding current raw display state.
 */
struct tm16xx_display {
	struct device *dev;
	const struct tm16xx_controller *controller;
	union {
		struct i2c_client *i2c;
		struct spi_device *spi;
	} client;
	u8 *spi_buffer;
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
	unsigned long *state;
};

/**
 * struct tm16xx_keypad - Keypad matrix state and input device
 * @display: Backpointer to owning display structure.
 * @input: Input device for reporting key events.
 * @state: Current bitmap of key states.
 * @last_state: Previous bitmap of key states for change detection.
 * @changes: Bitmap of key state changes since last poll.
 * @row_shift: Row shift for keymap encoding.
 */
struct tm16xx_keypad {
	struct tm16xx_display *display;
	struct input_dev *input;
	unsigned long *state;
	unsigned long *last_state;
	unsigned long *changes;
	u8 row_shift;
};

extern int tm16xx_probe(struct tm16xx_display *display);
extern void tm16xx_remove(struct tm16xx_display *display);
int tm16xx_keypad_probe(struct tm16xx_display *display);

/* state bitmap helpers */
/**
 * tm16xx_led_nbits() - Number of bits used for the display state bitmap
 * @display: pointer to tm16xx_display
 *
 * Return: total bits in the display state bitmap (grids * segments)
 */
static inline unsigned int tm16xx_led_nbits(const struct tm16xx_display *display)
{
	return display->num_grids * display->num_segments;
}

/**
 * tm16xx_set_seg() - Set the display state for a specific grid/segment
 * @display: pointer to tm16xx_display
 * @grid: grid index
 * @seg: segment index
 * @on: true to turn on, false to turn off
 */
static inline void tm16xx_set_seg(const struct tm16xx_display *display,
				  const u8 grid, const u8 seg, const bool on)
{
	assign_bit(grid * display->num_segments + seg, display->state, on);
}

#if KERNEL_VERSION(6, 10, 0) <= LINUX_VERSION_CODE // TODO remove
/**
 * tm16xx_get_grid() - Get the current segment pattern for a grid
 * @display: pointer to tm16xx_display
 * @grid: grid index
 *
 * Return: bit pattern of all segments for the given grid
 */
static inline u16 tm16xx_get_grid(const struct tm16xx_display *display,
				  const unsigned int grid)
{
	return bitmap_read(display->state, grid * display->num_segments,
			   display->num_segments);
}
#else
static inline u16 tm16xx_get_grid(const struct tm16xx_display *display,
				  const unsigned int grid)
{
	unsigned int start = grid * display->num_segments;
	u16 value = 0;
	int i;

	for (i = 0; i < display->num_segments; i++) {
		if (test_bit(start + i, display->state))
			value |= BIT(i);
	}

	return value;
}
#endif

/**
 * tm16xx_key_nbits() - Number of bits for the keypad state bitmap
 * @keypad: pointer to tm16xx_keypad
 *
 * Return: total bits in keypad state bitmap (max_key_rows * max_key_cols)
 */
static inline unsigned int tm16xx_key_nbits(const struct tm16xx_keypad *keypad)
{
	return keypad->display->controller->max_key_rows *
	       keypad->display->controller->max_key_cols;
}

/**
 * tm16xx_set_key() - Set the keypad state for a key
 * @keypad: pointer to tm16xx_keypad
 * @row: row index
 * @col: column index
 * @pressed: true if pressed, false otherwise
 */
static inline void tm16xx_set_key(const struct tm16xx_keypad *keypad,
				  const u8 row, const u8 col, const bool pressed)
{
	__assign_bit(row * keypad->display->controller->max_key_cols + col,
		     keypad->state, pressed);
}

/**
 * tm16xx_get_key_row() - Get row index from keypad bit index
 * @keypad: pointer to tm16xx_keypad
 * @bit: bit index in state bitmap
 *
 * Return: row index
 */
static inline u8 tm16xx_get_key_row(const struct tm16xx_keypad *keypad,
				    const unsigned int bit)
{
	return bit / keypad->display->controller->max_key_cols;
}

/**
 * tm16xx_get_key_col() - Get column index from keypad bit index
 * @keypad: pointer to tm16xx_keypad
 * @bit: bit index in state bitmap
 *
 * Return: column index
 */
static inline u8 tm16xx_get_key_col(const struct tm16xx_keypad *keypad,
				    const unsigned int bit)
{
	return bit % keypad->display->controller->max_key_cols;
}

#define tm16xx_for_each_key(keypad, _r, _c) \
	for (unsigned int _r = 0; \
	     _r < (keypad)->display->controller->max_key_rows; _r++) \
		for (unsigned int _c = 0; \
		     _c < (keypad)->display->controller->max_key_cols; _c++)

#endif /* _TM16XX_H */
