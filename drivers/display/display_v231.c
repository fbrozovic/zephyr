/*
 * Copyright (c) 2019 Filip Brozovic <fbrozovic@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_v231.h"
#include <display.h>
#include <gpio.h>
#include <spi.h>
#include <string.h>

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_v231);

K_MEM_POOL_DEFINE(v231_pool,
		  CONFIG_V231_POOL_BLOCK_MIN,
		  CONFIG_V231_POOL_BLOCK_MAX,
		  CONFIG_V231_POOL_BLOCK_NUM,
		  CONFIG_V231_POOL_BLOCK_ALIGN);

struct v231_data {
	int x_res;
	int y_res;
	enum display_orientation orientation;
	enum display_screen_info screen_info;
	struct device *reset_gpio;
	struct device *command_data_gpio;
	struct device *busy_gpio;
	struct device *enable_gpio;
#ifdef DT_PERVASIVE_V231_0_BS_GPIOS_CONTROLLER
	struct device *bs_gpio;
#endif
	struct device *spi_dev;
	struct spi_config spi_config;
#ifdef DT_PERVASIVE_V231_0_CS_GPIO_CONTROLLER
	struct spi_cs_control cs_ctrl;
#endif
};

static int v231_init(struct device *dev)
{
	struct v231_data *data = (struct v231_data *)dev->driver_data;

	LOG_DBG("Initializing display driver");

	data->spi_dev = device_get_binding(DT_PERVASIVE_V231_0_BUS_NAME);
	if (data->spi_dev == NULL) {
		LOG_ERR("Could not get SPI device for V231");
		return -EPERM;
	}

	data->spi_config.frequency = DT_PERVASIVE_V231_0_SPI_MAX_FREQUENCY;
	data->spi_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	data->spi_config.slave = DT_PERVASIVE_V231_0_BASE_ADDRESS;

#ifdef DT_PERVASIVE_V231_0_CS_GPIO_CONTROLLER
	data->cs_ctrl.gpio_dev =
		device_get_binding(DT_PERVASIVE_V231_0_CS_GPIO_CONTROLLER);
	data->cs_ctrl.gpio_pin = DT_PERVASIVE_V231_0_CS_GPIO_PIN;
	data->cs_ctrl.delay = 0;
	data->spi_config.cs = &data->cs_ctrl;
#else
	data->spi_config.cs = NULL;
#endif

	data->reset_gpio =
		device_get_binding(DT_PERVASIVE_V231_0_RESET_GPIOS_CONTROLLER);
	if (data->reset_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for V231 reset");
		return -EPERM;
	}

	gpio_pin_configure(data->reset_gpio,
			   DT_PERVASIVE_V231_0_RESET_GPIOS_PIN, GPIO_DIR_OUT);
	gpio_pin_write(data->reset_gpio,
		       DT_PERVASIVE_V231_0_RESET_GPIOS_PIN, 0);

	data->command_data_gpio =
		device_get_binding(DT_PERVASIVE_V231_0_CMD_DATA_GPIOS_CONTROLLER);
	if (data->command_data_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for V231 command/data");
		return -EPERM;
	}

	gpio_pin_configure(data->command_data_gpio,
			   DT_PERVASIVE_V231_0_CMD_DATA_GPIOS_PIN,
			   GPIO_DIR_OUT);

	data->busy_gpio =
		device_get_binding(DT_PERVASIVE_V231_0_BUSY_GPIOS_CONTROLLER);
	if (data->busy_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for V231 busy");
		return -EPERM;
	}

	gpio_pin_configure(data->busy_gpio, DT_PERVASIVE_V231_0_BUSY_GPIOS_PIN,
			   GPIO_DIR_IN);

	data->enable_gpio =
		device_get_binding(DT_PERVASIVE_V231_0_ENABLE_GPIOS_CONTROLLER);
	if (data->enable_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for V231 enable");
		return -EPERM;
	}

	gpio_pin_configure(data->enable_gpio,
			   DT_PERVASIVE_V231_0_ENABLE_GPIOS_PIN,
			   GPIO_DIR_OUT);
	gpio_pin_write(data->enable_gpio,
		       DT_PERVASIVE_V231_0_ENABLE_GPIOS_PIN, 0);

#ifdef DT_PERVASIVE_V231_0_BS_GPIOS_CONTROLLER
	data->bs_gpio =
		device_get_binding(DT_PERVASIVE_V231_0_BS_GPIOS_CONTROLLER);
	if (data->bs_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for V231 bs");
		return -EPERM;
	}

	gpio_pin_configure(data->bs_gpio, DT_PERVASIVE_V231_0_BS_GPIOS_PIN,
			   GPIO_DIR_OUT);
	gpio_pin_write(data->bs_gpio,
		       DT_PERVASIVE_V231_0_BS_GPIOS_PIN, 0);
#endif

#ifdef DT_PERVASIVE_V231_0_ORIENTATION
	switch (DT_PERVASIVE_V231_0_ORIENTATION) {
	case 0:
		display_set_orientation(dev, DISPLAY_ORIENTATION_NORMAL);
		break;
	case 90:
		display_set_orientation(dev, DISPLAY_ORIENTATION_ROTATED_90);
		break;
	case 180:
		display_set_orientation(dev, DISPLAY_ORIENTATION_ROTATED_180);
		break;
	case 270:
		display_set_orientation(dev, DISPLAY_ORIENTATION_ROTATED_270);
		break;
	default:
		LOG_WRN("Invalid display orientation");
		display_set_orientation(dev, DISPLAY_ORIENTATION_NORMAL);
		break;
	}

#else
	display_set_orientation(dev, DISPLAY_ORIENTATION_NORMAL);
#endif

	return 0;
}

static int v231_send_buf(struct v231_data *data, u8_t addr, const u8_t *buf,
			 u16_t len, bool repeat_first_value)
{
	int err;
	struct spi_buf spi_buf = {
		.buf = &addr,
		.len = 1,
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf,
		.count = 1,
	};

	gpio_pin_write(data->command_data_gpio,
		       DT_PERVASIVE_V231_0_CMD_DATA_GPIOS_PIN,
		       V231_CMD_DATA_PIN_COMMAND);

	err = spi_write(data->spi_dev, &data->spi_config, &tx);

	if (!len || err)
		return err;

	gpio_pin_write(data->command_data_gpio,
		       DT_PERVASIVE_V231_0_CMD_DATA_GPIOS_PIN,
		       V231_CMD_DATA_PIN_DATA);

	spi_buf.buf = (u8_t *)buf;

	while (len--) {
		if (!repeat_first_value)
			spi_buf.buf = buf++;
		err = spi_write(data->spi_dev, &data->spi_config, &tx);
		if (err)
			return err;
	}

	return 0;
}

static inline int v231_send_byte(struct v231_data *data, u8_t addr, u8_t val)
{
	return v231_send_buf(data, addr, &val, 1, false);
}

static inline void v231_wait_for_busy_high(struct v231_data *data)
{
	u32_t val = 0;

	do {
		gpio_pin_read(data->busy_gpio,
			      DT_PERVASIVE_V231_0_BUSY_GPIOS_PIN, &val);
	} while (!val);
}

static int v231_send_frame(struct v231_data *data, const u8_t *buf, int len)
{
	if (data->orientation == DISPLAY_ORIENTATION_NORMAL ||
	    data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		return v231_send_buf(data, V231_CMD_DATA1, buf, len, false);
	} else {
		struct k_mem_block block;
		int ret;

		k_mem_pool_alloc(&v231_pool, &block, len, K_NO_WAIT);

		for (int i = 0; i < len; i++) {
			int x = i % (DT_PERVASIVE_V231_0_WIDTH / 8);
			int y = i / (DT_PERVASIVE_V231_0_WIDTH / 8);

			int offset = ((DT_PERVASIVE_V231_0_WIDTH / 8) - 1 - x)
				     * DT_PERVASIVE_V231_0_HEIGHT + y;

			((u8_t *)block.data)[i] = buf[offset];
		}

		ret = v231_send_buf(data, V231_CMD_DATA1, block.data, len,
				    false);
		k_mem_pool_free(&block);

		return ret;
	}
}

static int v231_write(const struct device *dev, const u16_t x, const u16_t y,
		      const struct display_buffer_descriptor *desc,
		      const void *buf)
{
	if (x != 0 || y != 0) {
		LOG_ERR("Partial screen update not supported");
		return -EINVAL;
	}

	struct v231_data *data = (struct v231_data *)dev->driver_data;

	gpio_pin_write(data->enable_gpio,
		       DT_PERVASIVE_V231_0_ENABLE_GPIOS_PIN, 1);
	k_sleep(5);
	gpio_pin_write(data->reset_gpio,
		       DT_PERVASIVE_V231_0_RESET_GPIOS_PIN, 1);
	k_sleep(5);
	gpio_pin_write(data->reset_gpio,
		       DT_PERVASIVE_V231_0_RESET_GPIOS_PIN, 0);
	k_sleep(10);
	gpio_pin_write(data->reset_gpio,
		       DT_PERVASIVE_V231_0_RESET_GPIOS_PIN, 1);
	k_sleep(5);

	v231_send_byte(data, V231_CMD_PSR, V231_DATA_PSR_SOFT_RESET);
	k_sleep(5);

	/* TODO set temperature dynamically */
	v231_send_byte(data, V231_CMD_TEMP_SET, 25);
	v231_send_byte(data, V231_CMD_TEMP_ACTIVATE, V231_DATA_TEMP_ACTIVATE);

	u8_t psr[] = V231_DATA_PSR_DEFAULT;

	if (data->orientation == DISPLAY_ORIENTATION_ROTATED_90 ||
	    data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		psr[0] &= ~V231_DATA_PSR_ROTATION_MASK;
	}
	v231_send_buf(data, V231_CMD_PSR, psr, 2, false);

	v231_send_frame(data, buf, desc->buf_size);

	u8_t zero = 0;

	v231_send_buf(data, V231_CMD_DATA2, &zero, desc->buf_size, true);
	v231_wait_for_busy_high(data);

	v231_send_buf(data, V231_CMD_POWER_ON, NULL, 0, false);
	v231_wait_for_busy_high(data);

	v231_send_buf(data, V231_CMD_REFRESH, NULL, 0, false);
	v231_wait_for_busy_high(data);

	v231_send_buf(data, V231_CMD_POWER_OFF, NULL, 0, false);
	v231_wait_for_busy_high(data);

	gpio_pin_write(data->enable_gpio, DT_PERVASIVE_V231_0_ENABLE_GPIOS_PIN,
		       0);

	return 0;
}

static int v231_read(const struct device *dev, const u16_t x,
		     const u16_t y,
		     const struct display_buffer_descriptor *desc,
		     void *buf)
{
	LOG_ERR("Reading not supported");
	return -ENOTSUP;
}

static void *v231_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Direct framebuffer access not supported");
	return NULL;
}

static int v231_blanking_off(const struct device *dev)
{
	LOG_ERR("Blanking not supported");
	return -ENOTSUP;
}

static int v231_blanking_on(const struct device *dev)
{
	LOG_ERR("Blanking not supported");
	return -ENOTSUP;
}

static int v231_set_brightness(const struct device *dev,
			       const u8_t brightness)
{
	LOG_ERR("Set brightness not supported");
	return -ENOTSUP;
}

static int v231_set_contrast(const struct device *dev,
			     const u8_t contrast)
{
	LOG_ERR("Set contrast not supported");
	return -ENOTSUP;
}

static void v231_get_capabilities(const struct device *dev,
		struct display_capabilities *capabilities)
{
	struct v231_data *data = (struct v231_data *)dev->driver_data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = data->x_res;
	capabilities->y_resolution = data->y_res;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_MONO01;
	capabilities->current_pixel_format = PIXEL_FORMAT_MONO01;
	capabilities->current_orientation = data->orientation;
	capabilities->screen_info = data->screen_info;
}

static int v231_set_pixel_format(const struct device *dev,
		const enum display_pixel_format pixel_format)
{
	if (pixel_format == PIXEL_FORMAT_MONO01) {
		return 0;
	}

	LOG_ERR("Pixel format change not supported");
	return -ENOTSUP;
}

static int v231_set_orientation(const struct device *dev,
				const enum display_orientation orientation)
{
	struct v231_data *data = (struct v231_data *)dev->driver_data;

	data->orientation = orientation;

	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
	case DISPLAY_ORIENTATION_ROTATED_180:
		data->x_res = DT_PERVASIVE_V231_0_WIDTH;
		data->y_res = DT_PERVASIVE_V231_0_HEIGHT;
		data->screen_info = SCREEN_INFO_MONO_MSB_FIRST;
		break;
	case DISPLAY_ORIENTATION_ROTATED_90:
	case DISPLAY_ORIENTATION_ROTATED_270:
		data->x_res = DT_PERVASIVE_V231_0_HEIGHT;
		data->y_res = DT_PERVASIVE_V231_0_WIDTH;
		data->screen_info = SCREEN_INFO_MONO_VTILED;
		break;
	}

	return 0;
}

static const struct display_driver_api v231_api = {
	.blanking_on = v231_blanking_on,
	.blanking_off = v231_blanking_off,
	.write = v231_write,
	.read = v231_read,
	.get_framebuffer = v231_get_framebuffer,
	.set_brightness = v231_set_brightness,
	.set_contrast = v231_set_contrast,
	.get_capabilities = v231_get_capabilities,
	.set_pixel_format = v231_set_pixel_format,
	.set_orientation = v231_set_orientation,
};

static struct v231_data v231_data;

DEVICE_AND_API_INIT(v231, DT_PERVASIVE_V231_0_LABEL, &v231_init, &v231_data,
		    NULL, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY,
		    &v231_api);

