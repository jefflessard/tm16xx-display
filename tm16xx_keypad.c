// SPDX-License-Identifier: GPL-2.0
/*
 * TM16xx and compatible LED display/keypad controller driver
 * Supports TM16xx, FD6xx, PT6964, HBS658, AIP16xx and related chips.
 *
 * Copyright (C) 2024 Jean-François Lessard
 */

#include <linux/input/matrix_keypad.h>
#include <linux/property.h>

#include "tm16xx.h"

/**
 * tm16xx_keypad_poll() - Polls the keypad, reports events
 * @input: pointer to input_dev
 *
 * Reads the matrix keypad state, compares with previous state, and
 * reports key events to the input subsystem.
 */
static void tm16xx_keypad_poll(struct input_dev *input)
{
	struct tm16xx_keypad *keypad = input_get_drvdata(input);
	const unsigned short *keycodes = keypad->input->keycode;
	unsigned int nbits = tm16xx_key_nbits(keypad);
	unsigned int bit, scancode;
	u8 row, col;
	bool pressed;
	int ret;

	bitmap_zero(keypad->state, nbits);
	bitmap_zero(keypad->changes, nbits);

	mutex_lock(&keypad->display->lock);
	ret = keypad->display->controller->keys(keypad);
	mutex_unlock(&keypad->display->lock);

	if (ret < 0) {
		dev_err(keypad->display->dev, "Reading failed: %d\n", ret);
		return;
	}

	bitmap_xor(keypad->changes, keypad->state, keypad->last_state, nbits);

	for_each_set_bit(bit, keypad->changes, nbits) {
		row = tm16xx_get_key_row(keypad, bit);
		col = tm16xx_get_key_col(keypad, bit);
		pressed = _test_bit(bit, keypad->state);
		scancode = MATRIX_SCAN_CODE(row, col, keypad->row_shift);

		dev_dbg(keypad->display->dev,
			"key changed: %u, row=%u col=%u down=%d\n", bit, row,
			col, pressed);

		input_event(keypad->input, EV_MSC, MSC_SCAN, scancode);
		input_report_key(keypad->input, keycodes[scancode], pressed);
	}
	input_sync(keypad->input);

	bitmap_copy(keypad->last_state, keypad->state, nbits);
}

/**
 * tm16xx_keypad_probe() - Initialize keypad/input device
 * @display: pointer to tm16xx_display
 *
 * Return: 0 on success, negative error code on failure
 */
int tm16xx_keypad_probe(struct tm16xx_display *display)
{
	const u8 rows = display->controller->max_key_rows;
	const u8 cols = display->controller->max_key_cols;
	struct tm16xx_keypad *keypad;
	struct input_dev *input;
	unsigned int poll_interval, nbits;
	int ret;

	if (!display->controller->keys || !rows || !cols) {
		dev_dbg(display->dev, "keypad not supported\n");
		return 0;
	}

	if (!device_property_present(display->dev, "poll-interval") ||
	    !device_property_present(display->dev, "linux,keymap")) {
		dev_dbg(display->dev, "keypad disabled\n");
		return 0;
	}

	dev_dbg(display->dev, "Configuring keypad\n");

	ret = device_property_read_u32(display->dev, "poll-interval",
				       &poll_interval);
	if (ret < 0) {
		dev_err(display->dev, "Failed to read poll-interval: %d\n", ret);
		return ret;
	}

	keypad = devm_kzalloc(display->dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;
	keypad->display = display;

	nbits = tm16xx_key_nbits(keypad);
	keypad->state = devm_bitmap_zalloc(display->dev, nbits, GFP_KERNEL);
	keypad->last_state = devm_bitmap_zalloc(display->dev, nbits, GFP_KERNEL);
	keypad->changes = devm_bitmap_zalloc(display->dev, nbits, GFP_KERNEL);
	if (!keypad->state || !keypad->last_state || !keypad->changes) {
		return -ENOMEM;
	}

	input = devm_input_allocate_device(display->dev);
	if (!input) {
		dev_err(display->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}
	input->name = TM16XX_DRIVER_NAME "-keypad";
	keypad->input = input;
	input_set_drvdata(input, keypad);

	keypad->row_shift = get_count_order(cols);
	ret = matrix_keypad_build_keymap(NULL, "linux,keymap", rows, cols, NULL,
					 input);
	if (ret < 0) {
		dev_err(display->dev, "Failed to build keymap: %d\n", ret);
		return ret;
	}

	if (device_property_read_bool(display->dev, "autorepeat"))
		__set_bit(EV_REP, input->evbit);

	input_setup_polling(input, tm16xx_keypad_poll);
	input_set_poll_interval(input, poll_interval);
	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(display->dev, "Failed to register input device: %d\n",
			ret);
		return ret;
	}

	dev_dbg(display->dev, "keypad rows=%u, cols=%u, poll=%u\n", rows, cols,
		poll_interval);

	return 0;
}
