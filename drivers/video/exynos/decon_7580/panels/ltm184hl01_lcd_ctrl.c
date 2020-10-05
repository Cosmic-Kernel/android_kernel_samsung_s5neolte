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

int	backlight_on;
int board_rev;
unsigned int bl_current = 18000;
int lvds_comp;

struct i2c_client *lvds_client;
struct i2c_client *backlight1_client;
struct i2c_client *backlight2_client;

#define V5D3BX_VEESTRENGHT		0x00001f07
#define V5D3BX_VEEDEFAULTVAL		0
#define V5D3BX_DEFAULT_STRENGHT		5
#define V5D3BX_DEFAULT_LOW_STRENGHT	8
#define V5D3BX_DEFAULT_HIGH_STRENGHT	10
#define V5D3BX_MAX_STRENGHT		15

#define V5D3BX_CABCBRIGHTNESSRATIO	815
#define V5D3BX_10KHZ_DEFAULT_RATIO	187040
#define AUTOBRIGHTNESS_LIMIT_VALUE	207

#define MIN_BRIGHTNESS			0
#define MAX_BRIGHTNESS_LEVEL		255
#define MID_BRIGHTNESS_LEVEL		195
#define LOW_BRIGHTNESS_LEVEL		29
#define DIM_BRIGHTNESS_LEVEL		19
#define BL_DEFAULT_BRIGHTNESS		MID_BRIGHTNESS_LEVEL

#define V5D3BX_MIN_BRIGHTNESS			0
#define V5D3BX_MAX_BRIGHTNESS_LEVEL_SDC		330
#define V5D3BX_MID_BRIGHTNESS_LEVEL_SDC		185

#define V5D3BX_MAX_BRIGHTNESS_LEVEL_BOE		255
#define V5D3BX_MID_BRIGHTNESS_LEVEL_BOE		195

#define V5D3BX_LOW_BRIGHTNESS_LEVEL		11
#define V5D3BX_DIM_BRIGHTNESS_LEVEL		5

static int ltm184hl01_exit(struct dsim_device *dsim)
{
	int ret = 0;

	ret = gpio_request_one(backlight_on, GPIOF_OUT_INIT_LOW, "BLIC_ON");
	gpio_free(backlight_on);
	usleep_range(10000, 11000);

	msleep(100);

	lvds_comp = 0;

	return ret;
}

#define LVDS_I2C_RETRY_COUNT (2)

static int ql_lvds_WriteReg(u16 addr, u32 w_data)
{
	int retries = LVDS_I2C_RETRY_COUNT;
	int ret=0, err = 0;
	u8 tx_data[7];

	struct i2c_msg msg[] = {
		{lvds_client->addr, 0, 7, tx_data }
	};

	/* NOTE: Register address big-endian, data little-endian. */
	tx_data[0] = 0xA;

	tx_data[1] = (u8)(addr >> 8) & 0xff;
	tx_data[2] = (u8)addr & 0xff; ;
	tx_data[3] = w_data & 0xff;
	tx_data[4] = (w_data >> 8) & 0xff;
	tx_data[5] = (w_data >> 16) & 0xff;
	tx_data[6] = (w_data >> 24) & 0xff;

	do {

		ret = i2c_transfer(lvds_client->adapter, msg, ARRAY_SIZE(msg));
		if (likely(ret == 1))
				break;

			usleep_range(10000,11000);
			err = ret;
	} while (--retries > 0);


	/* Retry occured */
	if (unlikely(retries < LVDS_I2C_RETRY_COUNT)) {
			dsim_info("[vx5b3d]i2c_write: error %d, write (%04X, %08X), retry %d",
					err, addr, w_data, LVDS_I2C_RETRY_COUNT - retries);
	}

	if (unlikely(ret != 1)) {
			dsim_info("[vx5b3d]I2C does not work\n");
			return -EIO;
	}

	return 0;
}

static int lp8558_array_write(struct i2c_client * client, const struct LP8558_rom_data * eprom_ptr, int eprom_size) {
	int i = 0;
	int ret = 0;

	if (board_rev == 0)	return 0;

	if (client == backlight1_client)
		dsim_info("lp8558_1_i2c\n");
	else
		dsim_info("lp8558_2_i2c\n");

	for (i = 0; i < eprom_size; i++) {
		ret = i2c_smbus_write_byte_data(client, eprom_ptr[i].addr, eprom_ptr[i].val);
		if (ret < 0)
			dsim_err(":%s error : BL DEVICE setting fail : %d \n", __func__,  ret);
	}
	return 0;
}

static int ltm184hl01_lvds_init(struct dsim_device *dsim)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	ql_lvds_WriteReg(0x700, 0x18900040);
	ql_lvds_WriteReg(0x704, 0x101DA);
	ql_lvds_WriteReg(0x70C, 0x00004604);
	ql_lvds_WriteReg(0x710, 0x0545100B);
	ql_lvds_WriteReg(0x714, 0x20);
	ql_lvds_WriteReg(0x718, 0x00000102);
	ql_lvds_WriteReg(0x71C, 0xA8002F);
	ql_lvds_WriteReg(0x720, 0x1800);
	ql_lvds_WriteReg(0x154, 0x00000000);
	usleep_range(1000,1000);
	ql_lvds_WriteReg(0x154, 0x80000000);
	usleep_range(1000,1000);
	ql_lvds_WriteReg(0x700, 0x18900840);
	ql_lvds_WriteReg(0x70C, 0x5E46);
	ql_lvds_WriteReg(0x718, 0x00000202);

	ql_lvds_WriteReg(0x154, 0x00000000);
	usleep_range(1000,1000);
	ql_lvds_WriteReg(0x154, 0x80000000);
	usleep_range(1000,1000);

	ql_lvds_WriteReg(0x120, 0x5);
	ql_lvds_WriteReg(0x124, 0x521C780);
	ql_lvds_WriteReg(0x128, 0x102008);
	ql_lvds_WriteReg(0x12C, 0x14);
	ql_lvds_WriteReg(0x130, 0x3C10);
	ql_lvds_WriteReg(0x134, 0x15);
	ql_lvds_WriteReg(0x138, 0xFF0000);
	ql_lvds_WriteReg(0x13C, 0x0);

	ql_lvds_WriteReg(0x114, 0xc6302);/*added for bl pwm*/
	ql_lvds_WriteReg(0x140, 0x10000);

	ql_lvds_WriteReg(0x20C, 0x24);
	ql_lvds_WriteReg(0x21C, 0x780);
	ql_lvds_WriteReg(0x224, 0x7);
	ql_lvds_WriteReg(0x228, 0x50000);
	ql_lvds_WriteReg(0x22C, 0xFF08);
	ql_lvds_WriteReg(0x230, 0x1);
	ql_lvds_WriteReg(0x234, 0xCA033E10);
	ql_lvds_WriteReg(0x238, 0x00000060);
	ql_lvds_WriteReg(0x23C, 0x82E86030);
	ql_lvds_WriteReg(0x240, 0x28616088);
	ql_lvds_WriteReg(0x244, 0x00160285);
	ql_lvds_WriteReg(0x250, 0x600882A8);

	ql_lvds_WriteReg(0x258, 0x80014);
	ql_lvds_WriteReg(0x158, 0x0);
	usleep_range(1000,1000);
	ql_lvds_WriteReg(0x158, 0x1);
	usleep_range(1000,1000);
	ql_lvds_WriteReg(0x37C, 0x00001063);
	ql_lvds_WriteReg(0x380, 0x82A86030);
	ql_lvds_WriteReg(0x384, 0x2861408B);
	ql_lvds_WriteReg(0x388, 0x00130285);
	ql_lvds_WriteReg(0x38C, 0x10630009);
	ql_lvds_WriteReg(0x394, 0x400B82A8);
	ql_lvds_WriteReg(0x600, 0x16CC78D);
	if(board_rev) /*TCON RGB 888*/
		ql_lvds_WriteReg(0x608, 0x20F80);/*20 : delay for skew*/
	else /*TCON RGB 666*/
		ql_lvds_WriteReg(0x608, 0x20F0A);/*20 : delay for skew*/
	ql_lvds_WriteReg(0x154, 0x00000000);
	usleep_range(1000,1000);
	ql_lvds_WriteReg(0x154, 0x80000000);
	usleep_range(1000,1000);
	/*vee strenght initialization*/
	ql_lvds_WriteReg(0x400, 0x0);
	usleep_range(80, 80);

	lvds_comp = 1;

	dsim_info("--MDD : %s was called\n", __func__);

	return ret;

}


static int ltm184hl01_displayon(struct dsim_device *dsim)
{
	int ret = 0;

	if (!lvds_comp){
		dsim_info("MDD : %s lvds init needed\n", __func__);
		return 0;
	}

	dsim_info("MDD : %s was called\n", __func__);

	ret = gpio_request_one(backlight_on, GPIOF_OUT_INIT_HIGH, "BLIC_ON");
	gpio_free(backlight_on);
	usleep_range(10000, 11000);

	lp8558_array_write(backlight1_client, LP8558_eprom_drv_arr_init, ARRAY_SIZE(LP8558_eprom_drv_arr_init));
	lp8558_array_write(backlight2_client, LP8558_eprom_drv_arr_init, ARRAY_SIZE(LP8558_eprom_drv_arr_init));

// --------- displayon set
	/*pwm freq.=33.66M /(0x2DAA1+1)= 180hz;  */
	ql_lvds_WriteReg(0x160, 0x2C2DD);
	if(board_rev) /*TCON RGB 888*/
		ql_lvds_WriteReg(0x604, 0x3FFFFC00);
	else /*TCON RGB 666*/
		ql_lvds_WriteReg(0x604, 0x3FFFFD08);
	msleep(200);

	ql_lvds_WriteReg(0x138, 0x3fff0000);
	ql_lvds_WriteReg(0x15c, 0x5);/*selelct pwm*/
	msleep(1);/*after init -3*/

#ifdef __RGB_OUT__
	dsim_info(" <delay for RGB out> !!! ================================ \n");
	msleep(500);
	msleep(500);
	pr_info(" <making RGB out> !!! =================================== \n");
	ql_lvds_WriteReg(0x70C, 0x5E76);
	ql_lvds_WriteReg(0x710, 0x54D004F);
	ql_lvds_WriteReg(0x134, 0x05);
	mdelay(1);
	ql_lvds_WriteReg(0x154, 0x00000000);
	mdelay(1);
	ql_lvds_WriteReg(0x154, 0x80000000);
	mdelay(1);
	dsim_info(" <ending RGB out> !!! =================================== \n");
#endif

	msleep(5);

	return ret;
}

static int vx5b3d_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "lvds : need I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		return ret;
	}

	lvds_client = client;

	return ret;
}

static struct i2c_device_id vx5b3d_id[] = {
	{"lvds", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, vx5b3d_id);

static struct of_device_id vx5b3d_i2c_dt_ids[] = {
	{ .compatible = "lvds" },
	{ }
};

MODULE_DEVICE_TABLE(of, vx5b3d_i2c_dt_ids);

static struct i2c_driver vx5b3d_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lvds_vx5b3d",
		.of_match_table	= of_match_ptr(vx5b3d_i2c_dt_ids),
	},
	.id_table = vx5b3d_id,
	.probe = vx5b3d_probe,
};

static int lp8558_1_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "lp8558_1 : need I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		return ret;
	}

	backlight1_client = client;
	backlight_on = of_get_gpio(client->dev.of_node, 0);

	of_property_read_u32(client->dev.of_node, "rev", &board_rev);
	dsim_info("board rev (%d)\n", board_rev);
	return ret;
}

static struct i2c_device_id lp8558_1_id[] = {
	{"lp8558_1", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lp8558_1_id);

static struct of_device_id lp8558_1_i2c_dt_ids[] = {
	{ .compatible = "lp8558_1" },
	{ }
};

MODULE_DEVICE_TABLE(of, lp8558_1_i2c_dt_ids);

static struct i2c_driver lp8558_1_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "bl_lp8558_1",
		.of_match_table	= of_match_ptr(lp8558_1_i2c_dt_ids),
	},
	.id_table = lp8558_1_id,
	.probe = lp8558_1_probe,
};

static int lp8558_2_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "lp8558_2 : need I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		return ret;
	}

	backlight2_client = client;

	return ret;
}

static struct i2c_device_id lp8558_2_id[] = {
	{"lp8558_2", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lp8558_2_id);

static struct of_device_id lp8558_2_i2c_dt_ids[] = {
	{ .compatible = "lp8558_2" },
	{ }
};

MODULE_DEVICE_TABLE(of, lp8558_2_i2c_dt_ids);

static struct i2c_driver lp8558_2_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "bl_lp8558_2",
		.of_match_table	= of_match_ptr(lp8558_2_i2c_dt_ids),
	},
	.id_table = lp8558_2_id,
	.probe = lp8558_2_probe,
};

static int ltm184hl01_probe(struct dsim_device *dsim)
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

	dsim->priv.id[0]=0xff;
	lvds_comp = 1; /* in the first boot, lvds was initialized on bootloader already.*/

probe_exit:
	return ret;
}

static int ltm184hl01_early_probe(struct dsim_device *dsim)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	if (!lvds_client)
	ret = i2c_add_driver(&vx5b3d_i2c_driver);

	if (ret) {
		pr_err("vx5b3dx_i2c_init registration failed. ret= %d\n",ret);
	}

	if (!backlight1_client)
	ret = i2c_add_driver(&lp8558_1_i2c_driver);

	if (ret) {
		pr_err("lp8558_1_i2c_init registration failed. ret= %d\n",ret);
	}

	if (!backlight2_client)
	ret = i2c_add_driver(&lp8558_2_i2c_driver);

	if (ret) {
		pr_err("lp8558_2_i2c_init registration failed. ret= %d\n",ret);
	}

	return ret;
}

static void ltm184hl01_lvds_pwm_set(struct dsim_device *dsim, int level, int auto_bl)
{
	int vx5b3d_level = 0;
	u32 vee_strenght = 0;
	static u32 prev_vee_strenght=0;

	if (!lvds_comp){
		dsim_info("%s bl level(%d) autobl(%d) lvds init needed\n", __func__, level, auto_bl);
		return;
	}

	dsim_info("%s bl level(%d) autobl(%d)\n", __func__, level, auto_bl);

	if(auto_bl >= 5){
		lp8558_array_write(backlight1_client, LP8558_eprom_drv_arr_outdoor, ARRAY_SIZE(LP8558_eprom_drv_arr_outdoor));
		lp8558_array_write(backlight2_client, LP8558_eprom_drv_arr_outdoor, ARRAY_SIZE(LP8558_eprom_drv_arr_outdoor));
	}else{
		lp8558_array_write(backlight1_client, LP8558_eprom_drv_arr_normal, ARRAY_SIZE(LP8558_eprom_drv_arr_normal));
		lp8558_array_write(backlight2_client, LP8558_eprom_drv_arr_normal, ARRAY_SIZE(LP8558_eprom_drv_arr_normal));
	}

	/* brightness tuning*/
	if (level > MAX_BRIGHTNESS_LEVEL)
		level = MAX_BRIGHTNESS_LEVEL;

	if (level >= MID_BRIGHTNESS_LEVEL) {
		vx5b3d_level  = (level - MID_BRIGHTNESS_LEVEL) *
		(V5D3BX_MAX_BRIGHTNESS_LEVEL_BOE - V5D3BX_MID_BRIGHTNESS_LEVEL_BOE) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + V5D3BX_MID_BRIGHTNESS_LEVEL_BOE;
	} else if (level >= LOW_BRIGHTNESS_LEVEL) {
		vx5b3d_level  = (level - LOW_BRIGHTNESS_LEVEL) *
		(V5D3BX_MID_BRIGHTNESS_LEVEL_BOE - V5D3BX_LOW_BRIGHTNESS_LEVEL) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + V5D3BX_LOW_BRIGHTNESS_LEVEL;
	} else if (level >= DIM_BRIGHTNESS_LEVEL) {
		vx5b3d_level  = (level - DIM_BRIGHTNESS_LEVEL) *
		(V5D3BX_LOW_BRIGHTNESS_LEVEL - V5D3BX_DIM_BRIGHTNESS_LEVEL) / (LOW_BRIGHTNESS_LEVEL-DIM_BRIGHTNESS_LEVEL) + V5D3BX_DIM_BRIGHTNESS_LEVEL;
	} else if (level > 0)
		vx5b3d_level  = V5D3BX_DIM_BRIGHTNESS_LEVEL;
	else {
		vx5b3d_level = 0;
		pr_info("level = [%d]: vx5b3d_level = [%d]\n", level,vx5b3d_level);
	}

	switch (dsim->priv.auto_brightness) {
		case	0 ... 3:
			vee_strenght = V5D3BX_DEFAULT_STRENGHT;
			break;
		case	4 ... 5:
			vee_strenght = V5D3BX_DEFAULT_LOW_STRENGHT;
			break;
		case	6 ... 8:
			vee_strenght = V5D3BX_DEFAULT_HIGH_STRENGHT;
			break;
		default:
			vee_strenght = V5D3BX_DEFAULT_STRENGHT;
	}

	vee_strenght = V5D3BX_VEESTRENGHT | ((vee_strenght) << 27);

	if ((dsim->priv.auto_brightness && dsim->priv.auto_brightness < 5) || dsim->priv.siop_enable)
		vx5b3d_level = (vx5b3d_level * V5D3BX_CABCBRIGHTNESSRATIO) / 1000;

	if((vee_strenght != prev_vee_strenght)&& vx5b3d_level) {
		ql_lvds_WriteReg(0x400,0);/*temp*/
		prev_vee_strenght = vee_strenght;
	}

	if (vx5b3d_level != 0) {
		ql_lvds_WriteReg(0x164,(vx5b3d_level*V5D3BX_10KHZ_DEFAULT_RATIO)/255);

		dsim_info("[MIPI2LVDS-18INCH]:level=%d vx5b3d_level:%d auto_brightness:%d CABC:%d \n",\
			level,vx5b3d_level, dsim->priv.auto_brightness, dsim->priv.siop_enable);
	}

	dsim_info("%s bl_level : %d \n", __func__, level);

	return;
}


struct dsim_panel_ops ltm184hl01_panel_ops = {
	.early_probe = ltm184hl01_early_probe,
	.probe		= ltm184hl01_probe,
	.displayon	= ltm184hl01_displayon,
	.exit		= ltm184hl01_exit,
	.init		= NULL,
	.lvds_init	= ltm184hl01_lvds_init,
	.lvds_pwm_set	= ltm184hl01_lvds_pwm_set,
};

struct dsim_panel_ops *dsim_panel_get_priv_ops(struct dsim_device *dsim)
{
	printk(" %s \n ", __func__  );
	return &ltm184hl01_panel_ops;
}
