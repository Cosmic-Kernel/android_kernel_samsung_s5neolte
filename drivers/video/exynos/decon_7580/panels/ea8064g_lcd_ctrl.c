/*
 * drivers/video/decon_7580/panels/s6e3fa2_lcd_ctrl.c
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
#include "../dsim.h"

#include "panel_info.h"

#ifdef CONFIG_PANEL_AID_DIMMING
#include "aid_dimming.h"
#include "dimming_core.h"
#include "ea8064g_aid_dimming.h"
#include "dsim_backlight.h"
#endif

#ifdef CONFIG_PANEL_AID_DIMMING
static const unsigned char *HBM_TABLE[HBM_STATUS_MAX] = {SEQ_HBM_OFF, SEQ_HBM_ON};
static const unsigned char *ACL_CUTOFF_TABLE[ACL_STATUS_MAX] = {SEQ_ACL_OFF, SEQ_ACL_15};
static const unsigned char *ACL_OPR_TABLE[ACL_OPR_MAX] = {SEQ_ACL_OFF_OPR_AVR, SEQ_ACL_ON_OPR_AVR};

static const unsigned int br_tbl [256] = {
	2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 20, 21, 22,
	24, 25, 27, 29, 30, 32, 34, 37, 39, 41, 41, 44, 44, 47, 47, 50, 50, 53, 53, 56,
	56, 56, 60, 60, 60, 64, 64, 64, 68, 68, 68, 72, 72, 72, 72, 77, 77, 77, 82,
	82, 82, 82, 87, 87, 87, 87, 93, 93, 93, 93, 98, 98, 98, 98, 98, 105, 105,
	105, 105, 111, 111, 111, 111, 111, 111, 119, 119, 119, 119, 119, 126, 126, 126,
	126, 126, 126, 134, 134, 134, 134, 134, 134, 134, 143, 143, 143, 143, 143, 143,
	152, 152, 152, 152, 152, 152, 152, 152, 162, 162, 162, 162, 162, 162, 162, 172,
	172, 172, 172, 172, 172, 172, 172, 183, 183, 183, 183, 183, 183, 183, 183, 183,
	195, 195, 195, 195, 195, 195, 195, 195, 207, 207, 207, 207, 207, 207, 207, 207,
	207, 207, 220, 220, 220, 220, 220, 220, 220, 220, 220, 220, 234, 234, 234, 234,
	234, 234, 234, 234, 234, 234, 234, 249, 249, 249, 249, 249, 249, 249, 249, 249,
	249, 249, 249, 265, 265, 265, 265, 265, 265, 265, 265, 265, 265, 265, 265, 282,
	282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 300, 300, 300, 300,
	300, 300, 300, 300, 300, 300, 300, 300, 316, 316, 316, 316, 316, 316, 316, 316,
	316, 316, 316, 316, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 360,
};

static const short center_gamma[NUM_VREF][CI_MAX] = {
	{0x000, 0x000, 0x000},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x080, 0x080, 0x080},
	{0x100, 0x100, 0x100},
};

struct SmtDimInfo lily_dimming_info[MAX_BR_INFO + 1] = {				// add hbm array
	{.br = 5, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl5nit_B, .cTbl = ctbl5nit_B, .aid = aid5_B, .elvAcl = elv84, .elv = elv84},
	{.br = 6, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl6nit_B, .cTbl = ctbl6nit_B, .aid = aid6_B, .elvAcl = elv84, .elv = elv84},
	{.br = 7, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl7nit_B, .cTbl = ctbl7nit_B, .aid = aid7_B, .elvAcl = elv84, .elv = elv84},
	{.br = 8, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl8nit_B, .cTbl = ctbl8nit_B, .aid = aid8_B, .elvAcl = elv84, .elv = elv84},
	{.br = 9, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl9nit_B, .cTbl = ctbl9nit_B, .aid = aid9_B, .elvAcl = elv84, .elv = elv84},
	{.br = 10, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl10nit_B, .cTbl = ctbl10nit_B, .aid = aid10_B, .elvAcl = elv84, .elv = elv84},
	{.br = 11, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl11nit_B, .cTbl = ctbl11nit_B, .aid = aid11_B, .elvAcl = elv84, .elv = elv84},
	{.br = 12, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl12nit_B, .cTbl = ctbl12nit_B, .aid = aid12_B, .elvAcl = elv84, .elv = elv84},
	{.br = 13, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl13nit_B, .cTbl = ctbl13nit_B, .aid = aid13_B, .elvAcl = elv84, .elv = elv84},
	{.br = 14, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl14nit_B, .cTbl = ctbl14nit_B, .aid = aid14_B, .elvAcl = elv84, .elv = elv84},
	{.br = 15, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl15nit_B, .cTbl = ctbl15nit_B, .aid = aid15_B, .elvAcl = elv84, .elv = elv84},
	{.br = 16, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl16nit_B, .cTbl = ctbl16nit_B, .aid = aid16_B, .elvAcl = elv84, .elv = elv84},
	{.br = 17, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl17nit_B, .cTbl = ctbl17nit_B, .aid = aid17_B, .elvAcl = elv84, .elv = elv84},
	{.br = 19, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl19nit_B, .cTbl = ctbl19nit_B, .aid = aid19_B, .elvAcl = elv84, .elv = elv84},
	{.br = 20, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl20nit_B, .cTbl = ctbl20nit_B, .aid = aid20_B, .elvAcl = elv84, .elv = elv84},
	{.br = 21, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl21nit_B, .cTbl = ctbl21nit_B, .aid = aid21_B, .elvAcl = elv86, .elv = elv86},
	{.br = 22, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl22nit_B, .cTbl = ctbl22nit_B, .aid = aid22_B, .elvAcl = elv88, .elv = elv88},
	{.br = 24, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl24nit_B, .cTbl = ctbl24nit_B, .aid = aid24_B, .elvAcl = elv8a, .elv = elv8a},
	{.br = 25, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl25nit_B, .cTbl = ctbl25nit_B, .aid = aid25_B, .elvAcl = elv8c, .elv = elv8c},
	{.br = 27, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl27nit_B, .cTbl = ctbl27nit_B, .aid = aid27_B, .elvAcl = elv8e, .elv = elv8e},
	{.br = 29, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl29nit_B, .cTbl = ctbl29nit_B, .aid = aid29_B, .elvAcl = elv90, .elv = elv90},
	{.br = 30, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl30nit_B, .cTbl = ctbl30nit_B, .aid = aid30_B, .elvAcl = elv92, .elv = elv92},
	{.br = 32, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl32nit_B, .cTbl = ctbl32nit_B, .aid = aid32_B, .elvAcl = elv92, .elv = elv92},
	{.br = 34, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl34nit_B, .cTbl = ctbl34nit_B, .aid = aid34_B, .elvAcl = elv92, .elv = elv92},
	{.br = 37, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl37nit_B, .cTbl = ctbl37nit_B, .aid = aid37_B, .elvAcl = elv92, .elv = elv92},
	{.br = 39, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl39nit_B, .cTbl = ctbl39nit_B, .aid = aid39_B, .elvAcl = elv92, .elv = elv92},
	{.br = 41, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl41nit_B, .cTbl = ctbl41nit_B, .aid = aid41_B, .elvAcl = elv92, .elv = elv92},
	{.br = 44, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl44nit_B, .cTbl = ctbl44nit_B, .aid = aid44_B, .elvAcl = elv92, .elv = elv92},
	{.br = 47, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl47nit_B, .cTbl = ctbl47nit_B, .aid = aid47_B, .elvAcl = elv92, .elv = elv92},
	{.br = 50, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl50nit_B, .cTbl = ctbl50nit_B, .aid = aid50_B, .elvAcl = elv92, .elv = elv92},
	{.br = 53, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl53nit_B, .cTbl = ctbl53nit_B, .aid = aid53_B, .elvAcl = elv92, .elv = elv92},
	{.br = 56, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl56nit_B, .cTbl = ctbl56nit_B, .aid = aid56_B, .elvAcl = elv92, .elv = elv92},
	{.br = 60, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl60nit_B, .cTbl = ctbl60nit_B, .aid = aid60_B, .elvAcl = elv92, .elv = elv92},
	{.br = 64, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl64nit_B, .cTbl = ctbl64nit_B, .aid = aid64_B, .elvAcl = elv92, .elv = elv92},
	{.br = 68, .refBr = 114, .cGma = gma2p15, .rTbl = rtbl68nit_B, .cTbl = ctbl68nit_B, .aid = aid68_B, .elvAcl = elv92, .elv = elv92},
	{.br = 72, .refBr = 120, .cGma = gma2p15, .rTbl = rtbl72nit_B, .cTbl = ctbl72nit_B, .aid = aid68_B, .elvAcl = elv92, .elv = elv92},
	{.br = 77, .refBr = 127, .cGma = gma2p15, .rTbl = rtbl77nit_B, .cTbl = ctbl77nit_B, .aid = aid68_B, .elvAcl = elv91, .elv = elv91},
	{.br = 82, .refBr = 136, .cGma = gma2p15, .rTbl = rtbl82nit_B, .cTbl = ctbl82nit_B, .aid = aid68_B, .elvAcl = elv91, .elv = elv91},
	{.br = 87, .refBr = 143, .cGma = gma2p15, .rTbl = rtbl87nit_B, .cTbl = ctbl87nit_B, .aid = aid68_B, .elvAcl = elv91, .elv = elv91},
	{.br = 93, .refBr = 153, .cGma = gma2p15, .rTbl = rtbl93nit_B, .cTbl = ctbl93nit_B, .aid = aid68_B, .elvAcl = elv90, .elv = elv90},
	{.br = 98, .refBr = 159, .cGma = gma2p15, .rTbl = rtbl98nit_B, .cTbl = ctbl98nit_B, .aid = aid68_B, .elvAcl = elv90, .elv = elv90},
	{.br = 105, .refBr = 171, .cGma = gma2p15, .rTbl = rtbl105nit_B, .cTbl = ctbl105nit_B, .aid = aid68_B, .elvAcl = elv90, .elv = elv90},
	{.br = 111, .refBr = 179, .cGma = gma2p15, .rTbl = rtbl111nit_B, .cTbl = ctbl111nit_B, .aid = aid68_B, .elvAcl = elv8f, .elv = elv8f},
	{.br = 119, .refBr = 190, .cGma = gma2p15, .rTbl = rtbl119nit_B, .cTbl = ctbl119nit_B, .aid = aid68_B, .elvAcl = elv8f, .elv = elv8f},
	{.br = 126, .refBr = 202, .cGma = gma2p15, .rTbl = rtbl126nit_B, .cTbl = ctbl126nit_B, .aid = aid68_B, .elvAcl = elv8e, .elv = elv8e},
	{.br = 134, .refBr = 216, .cGma = gma2p15, .rTbl = rtbl134nit_B, .cTbl = ctbl134nit_B, .aid = aid68_B, .elvAcl = elv8e, .elv = elv8e},
	{.br = 143, .refBr = 227, .cGma = gma2p15, .rTbl = rtbl143nit_B, .cTbl = ctbl143nit_B, .aid = aid68_B, .elvAcl = elv8d, .elv = elv8d},
	{.br = 152, .refBr = 240, .cGma = gma2p15, .rTbl = rtbl152nit_B, .cTbl = ctbl152nit_B, .aid = aid68_B, .elvAcl = elv8c, .elv = elv8c},
	{.br = 162, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl162nit_B, .cTbl = ctbl162nit_B, .aid = aid68_B, .elvAcl = elv8c, .elv = elv8c},
	{.br = 172, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl172nit_B, .cTbl = ctbl172nit_B, .aid = aid172_B, .elvAcl = elv8c, .elv = elv8c},
	{.br = 183, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl183nit_B, .cTbl = ctbl183nit_B, .aid = aid183_B, .elvAcl = elv8c, .elv = elv8c},
	{.br = 195, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl195nit_B, .cTbl = ctbl195nit_B, .aid = aid195_B, .elvAcl = elv8b, .elv = elv8b},
	{.br = 207, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl207nit_B, .cTbl = ctbl207nit_B, .aid = aid207_B, .elvAcl = elv8a, .elv = elv8a},
	{.br = 220, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl220nit_B, .cTbl = ctbl220nit_B, .aid = aid220_B, .elvAcl = elv8a, .elv = elv8a},
	{.br = 234, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl234nit_B, .cTbl = ctbl234nit_B, .aid = aid234_B, .elvAcl = elv8a, .elv = elv8a},
	{.br = 249, .refBr = 256, .cGma = gma2p15, .rTbl = rtbl249nit_B, .cTbl = ctbl249nit_B, .aid = aid249_B, .elvAcl = elv89, .elv = elv89},
	{.br = 265, .refBr = 268, .cGma = gma2p15, .rTbl = rtbl265nit_B, .cTbl = ctbl265nit_B, .aid = aid249_B, .elvAcl = elv88, .elv = elv88},
	{.br = 282, .refBr = 286, .cGma = gma2p15, .rTbl = rtbl282nit_B, .cTbl = ctbl282nit_B, .aid = aid249_B, .elvAcl = elv87, .elv = elv87},
	{.br = 300, .refBr = 305, .cGma = gma2p15, .rTbl = rtbl300nit_B, .cTbl = ctbl300nit_B, .aid = aid249_B, .elvAcl = elv86, .elv = elv86},
	{.br = 316, .refBr = 319, .cGma = gma2p15, .rTbl = rtbl316nit_B, .cTbl = ctbl316nit_B, .aid = aid249_B, .elvAcl = elv86, .elv = elv86},
	{.br = 333, .refBr = 336, .cGma = gma2p15, .rTbl = rtbl333nit_B, .cTbl = ctbl333nit_B, .aid = aid249_B, .elvAcl = elv85, .elv = elv85},
	{.br = 360, .refBr = 350, .cGma = gma2p20, .rTbl = rtbl360nit_B, .cTbl = ctbl360nit_B, .aid = aid249_B, .elvAcl = elv84, .elv = elv84},
	{.br = 360, .refBr = 350, .cGma = gma2p20, .rTbl = rtbl360nit_B, .cTbl = ctbl360nit_B, .aid = aid249_B, .elvAcl = elv84, .elv = elv84},
};

struct SmtDimInfo daisy_dimming_info[MAX_BR_INFO + 1] = {				// add hbm array
	{.br = 5, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl5nit, .cTbl = ctbl5nit, .aid = aid5, .elvAcl = elv84, .elv = elv84},
	{.br = 6, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl6nit, .cTbl = ctbl6nit, .aid = aid6, .elvAcl = elv84, .elv = elv84},
	{.br = 7, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl7nit, .cTbl = ctbl7nit, .aid = aid7, .elvAcl = elv84, .elv = elv84},
	{.br = 8, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl8nit, .cTbl = ctbl8nit, .aid = aid8, .elvAcl = elv84, .elv = elv84},
	{.br = 9, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl9nit, .cTbl = ctbl9nit, .aid = aid9, .elvAcl = elv84, .elv = elv84},
	{.br = 10, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl10nit, .cTbl = ctbl10nit, .aid = aid10, .elvAcl = elv84, .elv = elv84},
	{.br = 11, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl11nit, .cTbl = ctbl11nit, .aid = aid11, .elvAcl = elv84, .elv = elv84},
	{.br = 12, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl12nit, .cTbl = ctbl12nit, .aid = aid12, .elvAcl = elv84, .elv = elv84},
	{.br = 13, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl13nit, .cTbl = ctbl13nit, .aid = aid13, .elvAcl = elv84, .elv = elv84},
	{.br = 14, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl14nit, .cTbl = ctbl14nit, .aid = aid14, .elvAcl = elv84, .elv = elv84},
	{.br = 15, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl15nit, .cTbl = ctbl15nit, .aid = aid15, .elvAcl = elv84, .elv = elv84},
	{.br = 16, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl16nit, .cTbl = ctbl16nit, .aid = aid16, .elvAcl = elv84, .elv = elv84},
	{.br = 17, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl17nit, .cTbl = ctbl17nit, .aid = aid17, .elvAcl = elv84, .elv = elv84},
	{.br = 19, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl19nit, .cTbl = ctbl19nit, .aid = aid19, .elvAcl = elv84, .elv = elv84},
	{.br = 20, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl20nit, .cTbl = ctbl20nit, .aid = aid20, .elvAcl = elv84, .elv = elv84},
	{.br = 21, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl21nit, .cTbl = ctbl21nit, .aid = aid21, .elvAcl = elv86, .elv = elv86},
	{.br = 22, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl22nit, .cTbl = ctbl22nit, .aid = aid22, .elvAcl = elv88, .elv = elv88},
	{.br = 24, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl24nit, .cTbl = ctbl24nit, .aid = aid24, .elvAcl = elv8a, .elv = elv8a},
	{.br = 25, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl25nit, .cTbl = ctbl25nit, .aid = aid25, .elvAcl = elv8c, .elv = elv8c},
	{.br = 27, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl27nit, .cTbl = ctbl27nit, .aid = aid27, .elvAcl = elv8e, .elv = elv8e},
	{.br = 29, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl29nit, .cTbl = ctbl29nit, .aid = aid29, .elvAcl = elv90, .elv = elv90},
	{.br = 30, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl30nit, .cTbl = ctbl30nit, .aid = aid30, .elvAcl = elv92, .elv = elv92},
	{.br = 32, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl32nit, .cTbl = ctbl32nit, .aid = aid32, .elvAcl = elv92, .elv = elv92},
	{.br = 34, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl34nit, .cTbl = ctbl34nit, .aid = aid34, .elvAcl = elv92, .elv = elv92},
	{.br = 37, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl37nit, .cTbl = ctbl37nit, .aid = aid37, .elvAcl = elv92, .elv = elv92},
	{.br = 39, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl39nit, .cTbl = ctbl39nit, .aid = aid39, .elvAcl = elv92, .elv = elv92},
	{.br = 41, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl41nit, .cTbl = ctbl41nit, .aid = aid41, .elvAcl = elv92, .elv = elv92},
	{.br = 44, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl44nit, .cTbl = ctbl44nit, .aid = aid44, .elvAcl = elv92, .elv = elv92},
	{.br = 47, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl47nit, .cTbl = ctbl47nit, .aid = aid47, .elvAcl = elv92, .elv = elv92},
	{.br = 50, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl50nit, .cTbl = ctbl50nit, .aid = aid50, .elvAcl = elv92, .elv = elv92},
	{.br = 53, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl53nit, .cTbl = ctbl53nit, .aid = aid53, .elvAcl = elv92, .elv = elv92},
	{.br = 56, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl56nit, .cTbl = ctbl56nit, .aid = aid56, .elvAcl = elv92, .elv = elv92},
	{.br = 60, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl60nit, .cTbl = ctbl60nit, .aid = aid60, .elvAcl = elv92, .elv = elv92},
	{.br = 64, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl64nit, .cTbl = ctbl64nit, .aid = aid64, .elvAcl = elv92, .elv = elv92},
	{.br = 68, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl68nit, .cTbl = ctbl68nit, .aid = aid68, .elvAcl = elv92, .elv = elv92},
	{.br = 72, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl72nit, .cTbl = ctbl72nit, .aid = aid72, .elvAcl = elv92, .elv = elv92},
	{.br = 77, .refBr = 119, .cGma = gma2p15, .rTbl = rtbl77nit, .cTbl = ctbl77nit, .aid = aid77, .elvAcl = elv91, .elv = elv91},
	{.br = 82, .refBr = 126, .cGma = gma2p15, .rTbl = rtbl82nit, .cTbl = ctbl82nit, .aid = aid77, .elvAcl = elv91, .elv = elv91},
	{.br = 87, .refBr = 132, .cGma = gma2p15, .rTbl = rtbl87nit, .cTbl = ctbl87nit, .aid = aid77, .elvAcl = elv91, .elv = elv91},
	{.br = 93, .refBr = 142, .cGma = gma2p15, .rTbl = rtbl93nit, .cTbl = ctbl93nit, .aid = aid77, .elvAcl = elv90, .elv = elv90},
	{.br = 98, .refBr = 149, .cGma = gma2p15, .rTbl = rtbl98nit, .cTbl = ctbl98nit, .aid = aid77, .elvAcl = elv90, .elv = elv90},
	{.br = 105, .refBr = 157, .cGma = gma2p15, .rTbl = rtbl105nit, .cTbl = ctbl105nit, .aid = aid77, .elvAcl = elv90, .elv = elv90},
	{.br = 111, .refBr = 167, .cGma = gma2p15, .rTbl = rtbl111nit, .cTbl = ctbl111nit, .aid = aid77, .elvAcl = elv8f, .elv = elv8f},
	{.br = 119, .refBr = 176, .cGma = gma2p15, .rTbl = rtbl119nit, .cTbl = ctbl119nit, .aid = aid77, .elvAcl = elv8f, .elv = elv8f},
	{.br = 126, .refBr = 186, .cGma = gma2p15, .rTbl = rtbl126nit, .cTbl = ctbl126nit, .aid = aid77, .elvAcl = elv8e, .elv = elv8e},
	{.br = 134, .refBr = 198, .cGma = gma2p15, .rTbl = rtbl134nit, .cTbl = ctbl134nit, .aid = aid77, .elvAcl = elv8e, .elv = elv8e},
	{.br = 143, .refBr = 209, .cGma = gma2p15, .rTbl = rtbl143nit, .cTbl = ctbl143nit, .aid = aid77, .elvAcl = elv8d, .elv = elv8d},
	{.br = 152, .refBr = 221, .cGma = gma2p15, .rTbl = rtbl152nit, .cTbl = ctbl152nit, .aid = aid77, .elvAcl = elv8c, .elv = elv8c},
	{.br = 162, .refBr = 235, .cGma = gma2p15, .rTbl = rtbl162nit, .cTbl = ctbl162nit, .aid = aid77, .elvAcl = elv8c, .elv = elv8c},
	{.br = 172, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl172nit, .cTbl = ctbl172nit, .aid = aid77, .elvAcl = elv8c, .elv = elv8c},
	{.br = 183, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl183nit, .cTbl = ctbl183nit, .aid = aid183, .elvAcl = elv8c, .elv = elv8c},
	{.br = 195, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl195nit, .cTbl = ctbl195nit, .aid = aid195, .elvAcl = elv8b, .elv = elv8b},
	{.br = 207, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl207nit, .cTbl = ctbl207nit, .aid = aid207, .elvAcl = elv8a, .elv = elv8a},
	{.br = 220, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl220nit, .cTbl = ctbl220nit, .aid = aid220, .elvAcl = elv8a, .elv = elv8a},
	{.br = 234, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl234nit, .cTbl = ctbl234nit, .aid = aid234, .elvAcl = elv8a, .elv = elv8a},
	{.br = 249, .refBr = 250, .cGma = gma2p15, .rTbl = rtbl249nit, .cTbl = ctbl249nit, .aid = aid249, .elvAcl = elv89, .elv = elv89},
	{.br = 265, .refBr = 269, .cGma = gma2p15, .rTbl = rtbl265nit, .cTbl = ctbl265nit, .aid = aid249, .elvAcl = elv88, .elv = elv88},
	{.br = 282, .refBr = 284, .cGma = gma2p15, .rTbl = rtbl282nit, .cTbl = ctbl282nit, .aid = aid249, .elvAcl = elv87, .elv = elv87},
	{.br = 300, .refBr = 300, .cGma = gma2p15, .rTbl = rtbl300nit, .cTbl = ctbl300nit, .aid = aid249, .elvAcl = elv86, .elv = elv86},
	{.br = 316, .refBr = 319, .cGma = gma2p15, .rTbl = rtbl316nit, .cTbl = ctbl316nit, .aid = aid249, .elvAcl = elv86, .elv = elv86},
	{.br = 333, .refBr = 335, .cGma = gma2p15, .rTbl = rtbl333nit, .cTbl = ctbl333nit, .aid = aid249, .elvAcl = elv85, .elv = elv85},
	{.br = 360, .refBr = 350, .cGma = gma2p20, .rTbl = rtbl350nit, .cTbl = ctbl350nit, .aid = aid249, .elvAcl = elv84, .elv = elv84},
	{.br = 360, .refBr = 350, .cGma = gma2p20, .rTbl = rtbl350nit, .cTbl = ctbl350nit, .aid = aid249, .elvAcl = elv84, .elv = elv84},
};

unsigned char lookup_tbl_350 [351] = { /*for max 350 nit */
	0, 18, 24, 29, 33, 37, 40, 43, 46, 48, 51, 53, 55, 57, 59, 61, 63, 64, 66, 68, 69,
	71, 72, 74, 75, 77, 78, 80, 81, 82, 83, 85, 86, 87, 88, 90, 91, 92, 93, 94, 95, 96,
	97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
	114, 114, 115, 116, 117, 118, 119, 119, 120, 121, 122, 123, 123, 124, 125, 126, 127,
	127,128, 129, 130, 130, 131, 132, 133, 133, 134, 135, 135, 136, 137, 138, 138, 139,
	140, 140, 141, 142, 142, 143, 144, 144, 145, 146, 146, 147, 148, 148, 149, 149, 150,
	151, 151, 152, 153, 153, 154, 154, 155, 156, 156, 157, 157, 158, 159, 159, 160, 160,
	161, 161, 162, 163, 163, 164, 164, 165, 165, 166, 166, 167, 168, 168, 169, 169, 170,
	170, 171, 171, 172, 172, 173, 173, 174, 175, 175, 176, 176, 177, 177, 178, 178, 179,
	179, 180, 180, 181, 181, 182, 182, 183, 183, 184, 184, 185, 185, 186, 186, 187, 187,
	188, 188, 188, 189, 189, 190, 190, 191, 191, 192, 192, 193, 193, 194, 194, 195, 195,
	195, 196, 196, 197, 197, 198, 198, 199, 199, 200, 200, 200, 201, 201, 202, 202, 203,
	203, 203, 204, 204, 205, 205, 206, 206, 206, 207, 207, 208, 208, 209, 209, 209, 210,
	210, 211, 211, 212, 212, 212, 213, 213, 214, 214, 214, 215, 215, 216, 216, 216, 217,
	217, 218, 218, 218, 219, 219, 220, 220, 220, 221, 221, 222, 222, 222, 223, 223, 224,
	224, 224, 225, 225, 225, 226, 226, 227, 227, 227, 228, 228, 229, 229, 229, 230, 230,
	230, 231, 231, 232, 232, 232, 233, 233, 233, 234, 234, 234, 235, 235, 236, 236, 236,
	237, 237, 237, 238, 238, 238, 239, 239, 240, 240, 240, 241, 241, 241, 242, 242, 242,
	243, 243, 243, 244, 244, 244, 245, 245, 246, 246, 246, 247, 247, 247, 248, 248, 248,
	249, 249, 249, 250, 250, 250, 251, 251, 251, 252, 252, 252, 253, 253, 253, 254, 254,
	254, 255, 255,
};

unsigned int gamma_multi_tbl_350 [256] = { /*for max 350 nit */
	0,	2,	9,	20, 	39, 	63, 	93, 	131,
	177,		228,	289,	356,	431,	513,	605,	704,
	811,	927,	1051,	1184,	1325,	1475,	1634,	1803,
	1979,	2165,	2360,	2565,	2778,	3001,	3234,	3475,
	3727,	3988,	4258,	4538,	4829,	5128,	5439,	5758,
	6088,	6428,	6778,	7139,	7508,	7890,	8280,	8682,
	9093,	9515,	9948,	10390,	10844,	11308,	11782,	12268,
	12764,	13271,	13789,	14317,	14857,	15407,	15968,	16539,
	17123,	17717,	18323,	18939,	19566,	20205,	20854,	21515,
	22188,	22872,	23567,	24273,	24990,	25719,	26460,	27212,
	27976,	28751,	29537,	30335,	31145,	31967,	32800,	33645,
	34502,	35370,	36250,	37143,	38046,	38962,	39889,	40829,
	41780,	42744,	43720,	44708,	45707,	46718,	47742,	48777,
	49825,	50886,	51958,	53043,	54139,	55248,	56369,	57503,
	58649,	59807,	60978,	62161,	63356,	64563,	65784,	67016,
	68262,	69520,	70789,	72073,	73368,	74676,	75997,	77330,
	78676,	80034,	81406,	82790,	84187,	85596,	87018,	88454,
	89901,	91362,	92836,	94322,	95821,	97333,	98858,	100397,
	101947, 	103512, 	105088, 	106679, 	108282, 	109898, 	111528, 	113170,
	114824, 	116494, 	118175, 	119870, 	121577, 	123299, 	125034, 	126781,
	128541, 	130316, 	132103, 	133903, 	135717, 	137545, 	139386, 	141240,
	143107, 	144988, 	146881, 	148789, 	150710, 	152645, 	154592, 	156554,
	158529, 	160517, 	162519, 	164534, 	166563, 	168606, 	170662, 	172732,
	174815, 	176912, 	179023, 	181147, 	183285, 	185437, 	187602, 	189781,
	191974, 	194181, 	196401, 	198635, 	200883, 	203145, 	205420, 	207709,
	210013, 	212329, 	214661, 	217006, 	219364, 	221737, 	224123, 	226524,
	228939, 	231368, 	233810, 	236266, 	238736, 	241221, 	243720, 	246232,
	248759, 	251299, 	253854, 	256423, 	259006, 	261603, 	264214, 	266840,
	269480, 	272133, 	274801, 	277483, 	280180, 	282890, 	285615, 	288354,
	291108, 	293875, 	296657, 	299453, 	302264, 	305088, 	307928, 	310782,
	313650, 	316531, 	319428, 	322339, 	325265, 	328205, 	331159, 	334128,
	337111, 	340109, 	343122, 	346148, 	349189, 	352245, 	355315, 	358400,
};

unsigned char lookup_tbl_360 [361] = { /*for max 360 nit */
	0, 	18, 	24, 	29, 	33, 	36, 	40, 	43,
	45, 	48, 	50, 	52, 	54, 	56, 	58, 	60,
	62, 	64, 	65, 	67, 	69, 	70, 	72, 	73,
	74, 	76, 	77, 	79, 	80, 	81, 	82, 	84,
	85, 	86, 	87, 	88, 	90, 	91, 	92, 	93,
	94, 	95, 	96, 	97, 	98, 	99, 	100, 	101,
	102, 	103, 	104, 	105, 	106, 	107, 	108, 	109,
	109, 	110, 	111, 	112, 	113, 	114, 	115, 	115,
	116, 	117, 	118, 	119, 	120, 	120, 	121, 	122,
	123, 	123, 	124, 	125, 	126, 	126, 	127, 	128,
	129, 	129, 	130, 	131, 	132, 	132, 	133, 	134,
	134, 	135, 	136, 	136, 	137, 	138, 	139, 	139,
	140, 	140, 	141, 	142, 	142, 	143, 	144, 	144,
	145, 	146, 	146, 	147, 	148, 	148, 	149, 	149,
	150, 	151, 	151, 	152, 	152, 	153, 	154, 	154,
	155, 	155, 	156, 	157, 	157, 	158, 	158, 	159,
	159, 	160, 	160, 	161, 	162, 	162, 	163, 	163,
	164, 	164, 	165, 	165, 	166, 	167, 	167, 	168,
	168, 	169, 	169, 	170, 	170, 	171, 	171, 	172,
	172, 	173, 	173, 	174, 	174, 	175, 	175, 	176,
	176, 	177, 	177, 	178, 	178, 	179, 	179, 	180,
	180, 	181, 	181, 	182, 	182, 	183, 	183, 	184,
	184, 	185, 	185, 	186, 	186, 	187, 	187, 	187,
	188, 	188, 	189, 	189, 	190, 	190, 	191, 	191,
	192, 	192, 	193, 	193, 	193, 	194, 	194, 	195,
	195, 	196, 	196, 	197, 	197, 	197, 	198, 	198,
	199, 	199, 	200, 	200, 	200, 	201, 	201, 	202,
	202, 	203, 	203, 	203, 	204, 	204, 	205, 	205,
	206, 	206, 	206, 	207, 	207, 	208, 	208, 	208,
	209, 	209, 	210, 	210, 	210, 	211, 	211, 	212,
	212, 	212, 	213, 	213, 	214, 	214, 	214, 	215,
	215, 	216, 	216, 	216, 	217, 	217, 	218, 	218,
	218, 	219, 	219, 	220, 	220, 	220, 	221, 	221,
	221, 	222, 	222, 	223, 	223, 	223, 	224, 	224,
	224, 	225, 	225, 	226, 	226, 	226, 	227, 	227,
	227, 	228, 	228, 	229, 	229, 	229, 	230, 	230,
	230, 	231, 	231, 	231, 	232, 	232, 	233, 	233,
	233, 	234, 	234, 	234, 	235, 	235, 	235, 	236,
	236, 	236, 	237, 	237, 	238, 	238, 	238, 	239,
	239, 	239, 	240, 	240, 	240, 	241, 	241, 	241,
	242, 	242, 	242, 	243, 	243, 	243, 	244, 	244,
	244, 	245, 	245, 	245, 	246, 	246, 	246, 	247,
	247, 	247, 	248, 	248, 	248, 	249, 	249, 	249,
	250, 	250, 	250, 	251, 	251, 	251, 	252, 	252,
	252, 	253, 	253, 	253, 	254, 	254, 	254, 	255,
	255
};

unsigned int gamma_multi_tbl_360 [256] = { /*for max 360 nit */
	0, 			2, 			9, 			21, 		40, 		65, 		96, 		135,
	182, 		235, 		297, 		366, 		443, 		528, 		622, 		724,
	834, 		953, 		1081, 		1218, 		1363, 		1517, 		1681, 		1854,
	2036, 		2227, 		2427, 		2638, 		2857, 		3087, 		3326, 		3574,
	3833, 		4102, 		4380, 		4668, 		4967, 		5275, 		5594, 		5923,
	6262, 		6612, 		6972, 		7343, 		7723, 		8115, 		8517, 		8930,
	9353, 		9787, 		10232, 		10687, 		11154, 		11631, 		12119, 		12619,
	13129, 		13650, 		14183, 		14726, 		15281, 		15847, 		16424, 		17012,
	17612, 		18223, 		18846, 		19480, 		20125, 		20782, 		21450, 		22130,
	22822, 		23525, 		24240, 		24966, 		25704, 		26454, 		27216, 		27989,
	28775, 		29572, 		30381, 		31202, 		32035, 		32880, 		33737, 		34606,
	35488, 		36381, 		37286, 		38204, 		39133, 		40075, 		41029, 		41996,
	42974, 		43965, 		44969, 		45985, 		47013, 		48053, 		49106, 		50171,
	51249, 		52340, 		53443, 		54558, 		55686, 		56827, 		57980, 		59146,
	60325, 		61516, 		62720, 		63937, 		65166, 		66408, 		67664, 		68931,
	70212, 		71506, 		72812, 		74132, 		75464, 		76810, 		78168, 		79539,
	80924, 		82321, 		83732, 		85155, 		86592, 		88042, 		89504, 		90981,
	92470, 		93972, 		95488, 		97017, 		98559, 		100114, 	101683, 	103265,
	104860, 	106469, 	108091, 	109727, 	111376, 	113038, 	114714, 	116403,
	118105, 	119822, 	121551, 	123295, 	125051, 	126822, 	128606, 	130403,
	132214, 	134039, 	135877, 	137729, 	139595, 	141475, 	143368, 	145275,
	147196, 	149130, 	151078, 	153040, 	155016, 	157006, 	159009, 	161027,
	163058, 	165103, 	167162, 	169235, 	171322, 	173423, 	175538, 	177667,
	179810, 	181967, 	184138, 	186323, 	188522, 	190735, 	192962, 	195203,
	197459, 	199729, 	202012, 	204310, 	206622, 	208949, 	211289, 	213644,
	216013, 	218396, 	220794, 	223206, 	225632, 	228072, 	230527, 	232996,
	235480, 	237978, 	240490, 	243016, 	245557, 	248113, 	250683, 	253267,
	255866, 	258479, 	261107, 	263749, 	266406, 	269077, 	271763, 	274464,
	277179, 	279908, 	282652, 	285411, 	288185, 	290973, 	293775, 	296593,
	299425, 	302271, 	305133, 	308009, 	310900, 	313805, 	316726, 	319661,
	322611, 	325575, 	328555, 	331549, 	334558, 	337582, 	340621, 	343675,
	346743, 	349826, 	352925, 	356038, 	359166, 	362309, 	365467, 	368640,
};

unsigned int * gamma_multi_tbl;
unsigned char * lookup_tbl;

static int init_dimming(struct dsim_device *dsim, u8 *mtp)
{
	int i, j;
	int pos = 0;
	int ret = 0;
	short temp;
	struct dim_data *dimming;
	struct panel_private *panel = &dsim->priv;
	struct SmtDimInfo *diminfo = NULL;

	dimming = (struct dim_data *)kmalloc(sizeof(struct dim_data), GFP_KERNEL);
	if (!dimming) {
		dsim_err("failed to allocate memory for dim data\n");
		ret = -ENOMEM;
		goto error;
	}

	if (dsim->priv.id[2] >= 0x02) {
		diminfo = lily_dimming_info;

		gamma_multi_tbl = gamma_multi_tbl_360;
		lookup_tbl = lookup_tbl_360;
	} else {
		diminfo = daisy_dimming_info;

		gamma_multi_tbl = gamma_multi_tbl_350;
		lookup_tbl = lookup_tbl_350;
	}

	panel->dim_data= (void *)dimming;
	panel->dim_info = (void *)diminfo;

	panel->br_tbl = (unsigned int *)br_tbl;
	panel->hbm_tbl = (unsigned char **)HBM_TABLE;
	panel->acl_cutoff_tbl = (unsigned char **)ACL_CUTOFF_TABLE;
	panel->acl_opr_tbl = (unsigned char **)ACL_OPR_TABLE;

	for (j = 0; j < CI_MAX; j++) {
		temp = ((mtp[pos] & 0x01) ? -1 : 1) * mtp[pos+1];
		dimming->t_gamma[V255][j] = (int)center_gamma[V255][j] + temp;
		dimming->mtp[V255][j] = temp;
		pos += 2;
	}

	for (i = V203; i > V0; i--) {
		for (j = 0; j < CI_MAX; j++) {
			temp = ((mtp[pos] & 0x80) ? -1 : 1) * (mtp[pos] & 0x7f);
			dimming->t_gamma[i][j] = (int)center_gamma[i][j] + temp;
			dimming->mtp[i][j] = temp;
			pos++;
		}
	}

	for (j = 0; j < CI_MAX; j++) {
		dimming->t_gamma[V0][j] = (int)center_gamma[V0][j] + temp;
		dimming->mtp[V0][j] = 0;
	}

	for (j = 0; j < CI_MAX; j++) {
		dimming->vt_mtp[j] = mtp[pos];
		pos++;
	}

#ifdef SMART_DIMMING_DEBUG
	dimm_info("Center Gamma Info : \n");
	for(i=0;i<VMAX;i++) {
		dsim_info("Gamma : %3d %3d %3d : %3x %3x %3x\n",
			dimming->t_gamma[i][CI_RED], dimming->t_gamma[i][CI_GREEN], dimming->t_gamma[i][CI_BLUE],
			dimming->t_gamma[i][CI_RED], dimming->t_gamma[i][CI_GREEN], dimming->t_gamma[i][CI_BLUE] );
	}
#endif
	dimm_info("VT MTP : \n");
	dimm_info("Gamma : %3d %3d %3d : %3x %3x %3x\n",
			dimming->vt_mtp[CI_RED], dimming->vt_mtp[CI_GREEN], dimming->vt_mtp[CI_BLUE],
			dimming->vt_mtp[CI_RED], dimming->vt_mtp[CI_GREEN], dimming->vt_mtp[CI_BLUE] );

	dimm_info("MTP Info : \n");
	for(i=0;i<VMAX;i++) {
		dimm_info("Gamma : %3d %3d %3d : %3x %3x %3x\n",
			dimming->mtp[i][CI_RED], dimming->mtp[i][CI_GREEN], dimming->mtp[i][CI_BLUE],
			dimming->mtp[i][CI_RED], dimming->mtp[i][CI_GREEN], dimming->mtp[i][CI_BLUE] );
	}

	ret = generate_volt_table(dimming);
	if (ret) {
		dimm_err("[ERR:%s] failed to generate volt table\n", __func__);
		goto error;
	}

	for (i = 0; i < MAX_BR_INFO - 1; i++) {
		ret = cal_gamma_from_index(dimming, &diminfo[i]);
		if (ret) {
			dsim_err("failed to calculate gamma : index : %d\n", i);
			goto error;
		}
	}
	memcpy(diminfo[i].gamma, SEQ_GAMMA_CONDITION_SET, ARRAY_SIZE(SEQ_GAMMA_CONDITION_SET));
error:
	return ret;

}
#endif

static int ea8064g_read_init_info(struct dsim_device *dsim, unsigned char* mtp)
{
	int i = 0;
	int ret;
	struct panel_private *panel = &dsim->priv;
	unsigned char bufForCoordi[EA8064G_COORDINATE_LEN] = {0,};
	unsigned char buf[EA8064G_MTP_DATE_SIZE] = {0, };

	dsim_info("MDD : %s was called\n", __func__);

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_ON_F0\n", __func__);
	}

	// id
	ret = dsim_read_hl_data(dsim, EA8064G_ID_REG, EA8064G_ID_LEN, dsim->priv.id);
	if (ret != EA8064G_ID_LEN) {
		dsim_err("%s : can't find connected panel. check panel connection\n",__func__);
		panel->lcdConnected = PANEL_DISCONNEDTED;
		goto read_exit;
	}

	dsim_info("READ ID : ");
	for(i = 0; i < EA8064G_ID_LEN; i++)
		dsim_info("%02x, ", dsim->priv.id[i]);
	dsim_info("\n");

	// mtp
	ret = dsim_read_hl_data(dsim, EA8064G_MTP_ADDR, EA8064G_MTP_DATE_SIZE, buf);
	if (ret != EA8064G_MTP_DATE_SIZE) {
		dsim_err("failed to read mtp, check panel connection\n");
		goto read_fail;
	}
	memcpy(mtp, buf, EA8064G_MTP_SIZE);
	dsim_info("READ MTP SIZE : %d\n", EA8064G_MTP_SIZE);
	dsim_info("=========== MTP INFO =========== \n");
	for (i = 0; i < EA8064G_MTP_SIZE; i++)
		dsim_info("MTP[%2d] : %2d : %2x\n", i, mtp[i], mtp[i]);

	//elvss

	ret = dsim_read_hl_data(dsim, ELVSS_REG, ELVSS_LEN, dsim->priv.elvss);
	if (ret != ELVSS_LEN) {
		dsim_err("failed to read mtp, check panel connection\n");
		goto read_fail;
	}

	// coordinate
	ret = dsim_read_hl_data(dsim, EA8064G_COORDINATE_REG, EA8064G_COORDINATE_LEN, bufForCoordi);
	if (ret != EA8064G_COORDINATE_LEN) {
		dsim_err("fail to read coordinate on command.\n");
		goto read_fail;
	}
	dsim->priv.coordinate[0] = bufForCoordi[0] << 8 | bufForCoordi[1];	/* X */
	dsim->priv.coordinate[1] = bufForCoordi[2] << 8 | bufForCoordi[3];	/* Y */
	dsim->priv.date[0] = bufForCoordi[4];  /* year, month */
	dsim->priv.date[1] = bufForCoordi[5];  /* day */


	dsim_info("READ coordi : ");
	for(i = 0; i < 2; i++)
		dsim_info("%d, ", dsim->priv.coordinate[i]);
	dsim_info("\n");

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_OFF_F0\n", __func__);
		goto read_fail;
	}
read_exit:
	return 0;

read_fail:
	return -ENODEV;

}

static int ea8064g_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *panel = &dsim->priv;
	unsigned char mtp[EA8064G_MTP_SIZE] = {0, };

	dsim_info("MDD : %s was called\n", __func__);

	panel->dim_data = (void *)NULL;
	panel->lcdConnected = PANEL_CONNECTED;
	panel->panel_type = 0;

	ret = ea8064g_read_init_info(dsim, mtp);
	if (panel->lcdConnected == PANEL_DISCONNEDTED) {
		dsim_err("dsim : %s lcd was not connected\n", __func__);
		goto probe_exit;
	}

#ifdef CONFIG_PANEL_AID_DIMMING
	ret = init_dimming(dsim, mtp);
	if (ret) {
		dsim_err("%s : failed to generate gamma tablen\n", __func__);
	}
#endif
probe_exit:
	return ret;

}


static int ea8064g_displayon(struct dsim_device *dsim)
{
	int ret = 0;
#ifdef CONFIG_LCD_ALPM
	struct panel_private *panel = &dsim->priv;
#endif

	dsim_info("MDD : %s was called\n", __func__);

	ret = dsim_write_hl_data(dsim, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : DISPLAY_ON\n", __func__);
 		goto displayon_err;
	}

displayon_err:
	return ret;

}

static int ea8064g_exit(struct dsim_device *dsim)
{
	int ret = 0;
	dsim_info("MDD : %s was called\n", __func__);
	ret = dsim_write_hl_data(dsim, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : DISPLAY_OFF\n", __func__);
		goto exit_err;
	}

	msleep(35);

	ret = dsim_write_hl_data(dsim, SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SLEEP_IN\n", __func__);
		goto exit_err;
	}

	msleep(130);

exit_err:
	return ret;
}

static int ea8064g_init(struct dsim_device *dsim)
{
	int ret = 0;

	dsim_info("MDD : %s was called\n", __func__);

	ret = dsim_panel_set_brightness(dsim, 1);
	if (ret) {
		dsim_err("%s : fail to set brightness\n", __func__);
	}

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_ON_F0\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_ON_FC, ARRAY_SIZE(SEQ_TEST_KEY_ON_FC));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_ON_FC\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_SLEEP_OUT\n", __func__);
		goto init_exit;
	}

	msleep(25);

	ret = dsim_write_hl_data(dsim, SEQ_DCDC1_GP, ARRAY_SIZE(SEQ_DCDC1_GP));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_SOURCE_CONTROL\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_DCDC1_SET, ARRAY_SIZE(SEQ_DCDC1_SET));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_SOURCE_CONTROL\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_SOURCE_CONTROL, ARRAY_SIZE(SEQ_SOURCE_CONTROL));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_SOURCE_CONTROL\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_PCD_CONTROL, ARRAY_SIZE(SEQ_PCD_CONTROL));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PCD_CONTROL\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_OFF_FC, ARRAY_SIZE(SEQ_TEST_KEY_OFF_FC));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_TEST_KEY_OFF_FC\n", __func__);
		goto init_exit;
	}

	msleep(120);

	ret = dsim_write_hl_data(dsim, SEQ_TE_OUT, ARRAY_SIZE(SEQ_TE_OUT));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_TE_OUT\n", __func__);
		goto init_exit;
	}

	ret = dsim_write_hl_data(dsim, SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TEST_KEY_OFF_F0\n", __func__);
		goto init_exit;
	}

init_exit:
	return ret;
}


struct dsim_panel_ops ea8064g_panel_ops = {
	.early_probe = NULL,
	.probe		= ea8064g_probe,
	.displayon	= ea8064g_displayon,
	.exit		= ea8064g_exit,
	.init		= ea8064g_init,
};

struct dsim_panel_ops *dsim_panel_get_priv_ops(struct dsim_device *dsim)
{
	return &ea8064g_panel_ops;
}
