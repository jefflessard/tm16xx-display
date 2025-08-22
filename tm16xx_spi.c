// SPDX-License-Identifier: GPL-2.0
/*
 * TM16xx and compatible LED display/keypad controller driver
 * Supports TM16xx, FD6xx, PT6964, HBS658, AIP16xx and related chips.
 *
 * Copyright (C) 2024 Jean-François Lessard
 */

#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#include "tm16xx.h"

#define TM16XX_SPI_BUFFER_SIZE	8
#define CH34XX_SPI_TWAIT_US	2

/**
 * tm16xx_spi_probe() - Probe callback for SPI-attached controllers
 * @spi: pointer to spi_device
 *
 * Return: 0 on success, negative error code on failure
 */
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

	/* Allocate DMA-safe buffer */
	display->spi_buffer = devm_kzalloc(&spi->dev, TM16XX_SPI_BUFFER_SIZE,
					   GFP_KERNEL);
	if (!display->spi_buffer)
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

/**
 * tm16xx_spi_remove() - Remove callback for SPI-attached controllers
 * @spi: pointer to spi_device
 */
static void tm16xx_spi_remove(struct spi_device *spi)
{
	struct tm16xx_display *display = spi_get_drvdata(spi);

	tm16xx_remove(display);
}

/**
 * tm16xx_spi_read() - SPI read helper for controller
 * @display: pointer to tm16xx_display
 * @cmd: command to send
 * @cmd_len: length of command
 * @data: buffer for received data
 * @data_len: length of data to read
 *
 * Return: 0 on success, negative error code on failure
 */
static int tm16xx_spi_read(struct tm16xx_display *display, u8 *cmd,
			   size_t cmd_len, u8 *data, size_t data_len)
{
	struct spi_device *spi = display->client.spi;
	struct spi_message msg;
	int ret;

	dev_dbg(display->dev, "spi_write %*ph", (char)cmd_len, cmd);

	/* If STB is high during transmission, command is invalid.
	 * Reading requires a minimum 2 microseconds wait (Twait)
	 * after the 8th CLK rising edge before reading on falling edge.
	 */
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = cmd,
			.len = cmd_len,
			.cs_change = 0, /* NO CS toggle */
			.delay.value = CH34XX_SPI_TWAIT_US,
			.delay.unit = SPI_DELAY_UNIT_USECS,
		}, {
			.rx_buf = data,
			.len = data_len,
		}
	};

	spi_message_init_with_transfers(&msg, xfers, ARRAY_SIZE(xfers));

	ret = spi_sync(spi, &msg);

	dev_dbg(display->dev, "spi_read %*ph", (char)data_len, data);

	return ret;
}

/**
 * tm16xx_spi_write() - SPI write helper for controller
 * @display: pointer to tm16xx_display
 * @data: data to write
 * @len: number of bytes to write
 *
 * Return: 0 on success, negative error code on failure
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
	u8 *cmd = display->spi_buffer;
	int ret;

	/* Set mode command based on grid count */
	cmd[0] = TM16XX_CMD_MODE;
	if (num_grids <= 4)
		cmd[0] |= TM16XX_MODE_4GRIDS;
	else if (num_grids == 5)
		cmd[0] |= TM16XX_MODE_5GRIDS;
	else if (num_grids == 6)
		cmd[0] |= TM16XX_MODE_6GRIDS;
	else
		cmd[0] |= TM16XX_MODE_7GRIDS;

	ret = tm16xx_spi_write(display, cmd, 1);
	if (ret < 0)
		return ret;

	/* Set data command */
	cmd[0] = TM16XX_CMD_WRITE | TM16XX_DATA_ADDR_AUTO;
	ret = tm16xx_spi_write(display, cmd, 1);
	if (ret < 0)
		return ret;

	/* Set control command with brightness */
	cmd[0] = TM16XX_CMD_CTRL |
		 TM16XX_CTRL_BRIGHTNESS(brightness, brightness - 1, TM16XX);
	ret = tm16xx_spi_write(display, cmd, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int tm1618_data(struct tm16xx_display *display, u8 index,
		       unsigned int grid)
{
	u8 *cmd = display->spi_buffer;

	cmd[0] = TM16XX_CMD_ADDR + index * 2;
	cmd[1] = FIELD_GET(TM1618_BYTE1_MASK, grid);
	cmd[2] = FIELD_GET(TM1618_BYTE2_MASK, grid) << TM1618_BYTE2_SHIFT;

	return tm16xx_spi_write(display, cmd, 3);
}

static int tm1628_data(struct tm16xx_display *display, u8 index,
		       unsigned int grid)
{
	u8 *cmd = display->spi_buffer;

	cmd[0] = TM16XX_CMD_ADDR + index * 2;
	cmd[1] = FIELD_GET(TM1628_BYTE1_MASK, grid);
	cmd[2] = FIELD_GET(TM1628_BYTE2_MASK, grid);

	return tm16xx_spi_write(display, cmd, 3);
}

static int tm1628_keys(struct tm16xx_keypad *keypad)
{
	u8 *cmd = keypad->display->spi_buffer;
	u8 *codes = keypad->display->spi_buffer;
	int ret, i;

	cmd[0] = TM16XX_CMD_READ;
	ret = tm16xx_spi_read(keypad->display, cmd, 1, codes,
			      TM1628_KEY_READ_LEN);
	if (ret)
		return ret;

	/* prevent false readings */
	for (i = 0; i < TM1628_KEY_READ_LEN; i++) {
		if (codes[i] & ~TM1628_KEY_MASK)
			return -EINVAL;
	}

	tm16xx_for_each_key(keypad, row, col) {
		int byte = col >> 1;
		int bit = row + ((col & 1) * 3);
		bool value = !!(codes[byte] & BIT(bit));

		tm16xx_set_key(keypad, row, col, value);
	}

	return 0;
}

static int tm1638_keys(struct tm16xx_keypad *keypad)
{
	u8 *cmd = keypad->display->spi_buffer;
	u8 *codes = keypad->display->spi_buffer;
	int ret, i;

	cmd[0] = TM16XX_CMD_READ;
	ret = tm16xx_spi_read(keypad->display, cmd, 1, codes,
			      TM1638_KEY_READ_LEN);
	if (ret)
		return ret;

	/* prevent false readings */
	for (i = 0; i < TM1638_KEY_READ_LEN; i++) {
		if (codes[i] & ~TM1638_KEY_MASK)
			return -EINVAL;
	}

	tm16xx_for_each_key(keypad, row, col) {
		int byte = col >> 1;
		int bit = (2 - row) + ((col & 1) << 2);
		bool value = !!(codes[byte] & BIT(bit));

		tm16xx_set_key(keypad, row, col, value);
	}

	return 0;
}

static int tm1618_keys(struct tm16xx_keypad *keypad)
{
	u8 *cmd = keypad->display->spi_buffer;
	u8 *codes = keypad->display->spi_buffer;
	int ret, i;

	cmd[0] = TM16XX_CMD_READ;
	ret = tm16xx_spi_read(keypad->display, cmd, 1, codes,
			      TM1618_KEY_READ_LEN);
	if (ret)
		return ret;

	/* prevent false readings */
	for (i = 0; i < TM1618_KEY_READ_LEN; i++) {
		if (codes[i] & ~TM1618_KEY_MASK)
			return -EINVAL;
	}

	tm16xx_set_key(keypad, 0, 0, !!(codes[0] & BIT(1)));
	tm16xx_set_key(keypad, 0, 1, !!(codes[0] & BIT(4)));
	tm16xx_set_key(keypad, 0, 2, !!(codes[1] & BIT(1)));
	tm16xx_set_key(keypad, 0, 3, !!(codes[1] & BIT(4)));
	tm16xx_set_key(keypad, 0, 4, !!(codes[2] & BIT(1)));

	return 0;
}

static int fd620_data(struct tm16xx_display *display, u8 index,
		      unsigned int grid)
{
	u8 *cmd = display->spi_buffer;

	cmd[0] = TM16XX_CMD_ADDR + index * 2;
	cmd[1] = FIELD_GET(FD620_BYTE1_MASK, grid);
	cmd[2] = FIELD_GET(FD620_BYTE2_MASK, grid) << FD620_BYTE2_SHIFT;

	return tm16xx_spi_write(display, cmd, 3);
}

static int fd620_keys(struct tm16xx_keypad *keypad)
{
	u8 *cmd = keypad->display->spi_buffer;
	u8 *codes = keypad->display->spi_buffer;
	int ret, i;

	cmd[0] = TM16XX_CMD_READ;
	ret = tm16xx_spi_read(keypad->display, cmd, 1, codes,
			      FD620_KEY_READ_LEN);
	if (ret)
		return ret;

	/* prevent false readings */
	for (i = 0; i < FD620_KEY_READ_LEN; i++) {
		if (codes[i] & ~FD620_KEY_MASK)
			return -EINVAL;
	}

	tm16xx_set_key(keypad, 0, 0, codes[0] & BIT(0));
	tm16xx_set_key(keypad, 0, 1, codes[0] & BIT(3));
	tm16xx_set_key(keypad, 0, 2, codes[1] & BIT(0));
	tm16xx_set_key(keypad, 0, 3, codes[1] & BIT(3));
	tm16xx_set_key(keypad, 0, 4, codes[2] & BIT(0));
	tm16xx_set_key(keypad, 0, 5, codes[2] & BIT(3));
	tm16xx_set_key(keypad, 0, 6, codes[3] & BIT(0));

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

static const struct tm16xx_controller tm1638_controller = {
	.max_grids = 8,
	.max_segments = 10,
	.max_brightness = 8,
	.max_key_rows = 3,
	.max_key_cols = 8,
	.init = tm1628_init,
	.data = tm1628_data,
	.keys = tm1638_keys,
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
	{ .compatible = "titanmec,tm1638",  .data = &tm1638_controller },
	{ .compatible = "fdhisi,fd620",     .data = &fd620_controller  },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm16xx_spi_of_match);
#endif

static const struct spi_device_id tm16xx_spi_id[] = {
	{ "tm1618",  (kernel_ulong_t)&tm1618_controller },
	{ "tm1620",  (kernel_ulong_t)&tm1620_controller },
	{ "tm1628",  (kernel_ulong_t)&tm1628_controller },
	{ "tm1638",  (kernel_ulong_t)&tm1638_controller },
	{ "fd620",   (kernel_ulong_t)&fd620_controller  },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, tm16xx_spi_id);

static struct spi_driver tm16xx_spi_driver = {
	.driver = {
		.name = "tm16xx-spi",
		.of_match_table = of_match_ptr(tm16xx_spi_of_match),
	},
	.probe = tm16xx_spi_probe,
	.remove = tm16xx_spi_remove,
	.shutdown = tm16xx_spi_remove,
	.id_table = tm16xx_spi_id,
};
module_spi_driver(tm16xx_spi_driver);

MODULE_AUTHOR("Jean-François Lessard");
MODULE_DESCRIPTION("TM16xx-spi LED Display Controllers");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(TM16XX);
