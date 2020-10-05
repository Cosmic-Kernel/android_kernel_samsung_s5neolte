/*
 * drivers/video/decon_7580/panels/s6d7aa0x62_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2014 Samsung Electronics
 *
 * Jiun Yu, <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <video/mipi_display.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "../dsim.h"

#include "panel_info.h"

struct i2c_client *backlight_client;
int	backlight_on;

static int s6d7aa0x62_read_init_info(struct dsim_device *dsim)
{
	int i = 0;
	int ret;
	struct panel_private *panel = &dsim->priv;

	dsim_info("MDD : %s was called\n", __func__);

	// id
	ret = dsim_read_hl_data(dsim, S6D7AA0X62_ID_REG, S6D7AA0X62_ID_LEN, dsim->priv.id);
	if (ret != S6D7AA0X62_ID_LEN) {
		dsim_err("%s : can't find connected panel. check panel connection\n",__func__);
		panel->lcdConnected = PANEL_DISCONNEDTED;
		goto read_exit;
	}

	dsim_info("READ ID : ");
	for(i = 0; i < S6D7AA0X62_ID_LEN; i++)
		dsim_info("%02x, ", dsim->priv.id[i]);
	dsim_info("\n");

read_exit:
	return 0;
}

static int s6d7aa0x62_displayon(struct dsim_device *dsim)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	ret = dsim_write_hl_data(dsim, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : DISPLAY_ON\n", __func__);
 		goto displayon_err;
	}

	msleep(20);

	ret = dsim_write_hl_data(dsim, SEQ_BRIGHTNESS_1, ARRAY_SIZE(SEQ_BRIGHTNESS_1));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_BRIGHTNESS_1\n", __func__);
		goto displayon_err;
	}

	ret = dsim_write_hl_data(dsim, SEQ_BRIGHTNESS_2, ARRAY_SIZE(SEQ_BRIGHTNESS_2));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_BRIGHTNESS_2\n", __func__);
		goto displayon_err;
	}

	ret = dsim_write_hl_data(dsim, SEQ_BRIGHTNESS_3, ARRAY_SIZE(SEQ_BRIGHTNESS_3));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_BRIGHTNESS_3\n", __func__);
		goto displayon_err;
	}
displayon_err:
	return ret;

}

static int s6d7aa0x62_exit(struct dsim_device *dsim)
{
	int ret = 0;
	dsim_info("MDD : %s was called\n", __func__);
	ret = dsim_write_hl_data(dsim, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : DISPLAY_OFF\n", __func__);
		goto exit_err;
	}

	msleep(10);

	ret = dsim_write_hl_data(dsim, SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SLEEP_IN\n", __func__);
		goto exit_err;
	}

	msleep(150);

	ret = gpio_request_one(backlight_on, GPIOF_OUT_INIT_LOW, "BLIC_ON");
	gpio_free(backlight_on);
	usleep_range(10000, 11000);

exit_err:
	return ret;
}

int lm3632_array_write(const struct LM3632_rom_data * eprom_ptr, int eprom_size) {
	int i = 0;
	int ret = 0;
	for (i = 0; i < eprom_size; i++) {
		ret = i2c_smbus_write_byte_data(backlight_client, eprom_ptr[i].addr, eprom_ptr[i].val);
		if (ret < 0)
			dsim_err(":%s error : BL DEVICE_CTRL setting fail : %d \n", __func__, ret);
	}

	return 0;
}

static int s6d7aa0x62_init(struct dsim_device *dsim)
{
	int ret;

	dsim_info("MDD : %s was called\n", __func__);

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_ON_F0\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_ON_F1, ARRAY_SIZE(SEQ_TEST_KEY_ON_F1));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_ON_F1\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_ON_FC, ARRAY_SIZE(SEQ_TEST_KEY_ON_FC));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_ON_FC\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_OTP_RELOAD, ARRAY_SIZE(SEQ_OTP_RELOAD));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_OTP_RELOAD\n", __func__);
		goto init_exit;
	}

#if 0
	ret = dsim_read_hl_data(dsim, S6D7AA0X62_ID_REG, S6D7AA0X62_ID_LEN, dsim->priv.id);
	if (ret != S6D7AA0X62_ID_LEN) {
		dsim_err("%s : can't find connected panel. check panel connection\n",__func__);
	}


	dsim_info("READ ID : ");
	for(i = 0; i < S6D7AA0X62_ID_LEN; i++)
		dsim_info("%02x, ", dsim->priv.id[i]);
	dsim_info("\n");
#else
	s6d7aa0x62_read_init_info(dsim);
#endif
	ret = dsim_write_hl_data(dsim, SEQ_INIT_1, ARRAY_SIZE(SEQ_INIT_1));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_1\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_2, ARRAY_SIZE(SEQ_INIT_2));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_2\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_3, ARRAY_SIZE(SEQ_INIT_3));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_3\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_4, ARRAY_SIZE(SEQ_INIT_4));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_4\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_5, ARRAY_SIZE(SEQ_INIT_5));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_5\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_6, ARRAY_SIZE(SEQ_INIT_6));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_6\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_7, ARRAY_SIZE(SEQ_INIT_7));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_7\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_8, ARRAY_SIZE(SEQ_INIT_8));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_8\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_9, ARRAY_SIZE(SEQ_INIT_9));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_9\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_10, ARRAY_SIZE(SEQ_INIT_10));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_10\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_11, ARRAY_SIZE(SEQ_INIT_11));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_11\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_12, ARRAY_SIZE(SEQ_INIT_12));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_12\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_13, ARRAY_SIZE(SEQ_INIT_13));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_13\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_14, ARRAY_SIZE(SEQ_INIT_14));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_14\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_15, ARRAY_SIZE(SEQ_INIT_15));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_15\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_16, ARRAY_SIZE(SEQ_INIT_16));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_16\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_INIT_17, ARRAY_SIZE(SEQ_INIT_17));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_INIT_17\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_GAMMA_1, ARRAY_SIZE(SEQ_GAMMA_1));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_GAMMA_1\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_GAMMA_2, ARRAY_SIZE(SEQ_GAMMA_2));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_GAMMA_2\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_SLEEP_OUT\n", __func__);
		goto init_exit;
	}

	msleep(120);

init_exit:
	return ret;
}


static int lm3632_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "led : need I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_i2c;
	}

	backlight_client = client;

	backlight_on = of_get_gpio(client->dev.of_node, 0);

err_i2c:
	return ret;
}

static struct i2c_device_id lm3632_id[] = {
	{"lm3632", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lm3632_id);

static struct of_device_id lm3632_i2c_dt_ids[] = {
	{ .compatible = "lm3632,i2c" },
	{ }
};

MODULE_DEVICE_TABLE(of, lm3632_i2c_dt_ids);

static struct i2c_driver lm3632_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lm3632",
		.of_match_table	= of_match_ptr(lm3632_i2c_dt_ids),
	},
	.id_table = lm3632_id,
	.probe = lm3632_probe,
};

static int s6d7aa0x62_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *panel = &dsim->priv;

	dsim_info("MDD : %s was called\n", __func__);

	panel->dim_data = (void *)NULL;
	panel->lcdConnected = PANEL_CONNECTED;
	panel->panel_type = 0;

	if (panel->lcdConnected == PANEL_DISCONNEDTED) {
		dsim_err("dsim : %s lcd was not connected\n", __func__);
		goto probe_exit;

	}

probe_exit:
	return ret;
}

static int s6d7aa0x62_early_probe(struct dsim_device *dsim)
{
	int ret = 0;

	if (!backlight_client)
		i2c_add_driver(&lm3632_i2c_driver);

	ret = gpio_request_one(backlight_on, GPIOF_OUT_INIT_HIGH, "BLIC_ON");
	gpio_free(backlight_on);
	usleep_range(10000, 11000);

	lm3632_array_write(LM3632_eprom_drv_arr, ARRAY_SIZE(LM3632_eprom_drv_arr));

	return ret;
}

struct dsim_panel_ops s6d7aa0x62_panel_ops = {
	.early_probe = s6d7aa0x62_early_probe,
	.probe		= s6d7aa0x62_probe,
	.displayon	= s6d7aa0x62_displayon,
	.exit		= s6d7aa0x62_exit,
	.init		= s6d7aa0x62_init,
};

struct dsim_panel_ops *dsim_panel_get_priv_ops(struct dsim_device *dsim)
{
	return &s6d7aa0x62_panel_ops;
}
