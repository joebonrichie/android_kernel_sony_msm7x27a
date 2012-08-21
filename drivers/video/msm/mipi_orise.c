/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"
#include <mach/vreg.h>
#include <linux/gpio.h>

#define AUO_PANEL_ID 0x42
#define CMI_PANEL_ID 0x43

static struct msm_panel_common_pdata *mipi_orise_pdata = NULL;

static struct dsi_buf orise_tx_buf;
static struct dsi_buf orise_rx_buf;
/* FIH-SW-MM-VH-DISPLAY-22+ */
static char gPanelModel = 0;

static int display_initialize = 0;
/* FIH-SW2-MM-KW-BKL_PWM-00+{ */
static char BKL_PWM[11] = {
/*   LEVEL Value,   Duty,    Brightness */
	0x00,  /*  0,   0.0%,   0.00 cd/m^2 */
	0x1F,  /* 31,  12.5%,  50.00 cd/m^2 */
	0x34,  /* 52,  20.7%,  88.00 cd/m^2 */
	0x4C,  /* 76,  30.1%, 127.00 cd/m^2 */
	0x64,  /*100,  39.5%, 166.00 cd/m^2 */
	0x7B,  /*123,  48.4%, 205.00 cd/m^2 */
	0x94,  /*148,  58.2%, 242.00 cd/m^2 */
	0xAD,  /*173,  68.0%, 281.00 cd/m^2 */
	0xC7,  /*199,  78.1%, 319.00 cd/m^2 */
	0xE1,  /*225,  88.3%, 357.00 cd/m^2 */
	0xFF   /*255, 100.0%, 396.00 cd/m^2 */
};
/* FIH-SW2-MM-KW-BKL_PWM-00-} */

/* orise fwvga panel */
/* ----------- [For AUO panel setting Start] ----------- */
static char shift_address00[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char extc[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char shift_address01[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char cmd2[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
/* FIH-SW2-MM-KW-AUO_INIT_20120427-00+{ */
static char shift_address02[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char cmd_c430[2] = {0xC4, 0x30};
static char shift_address03[2] = {0x00, 0x90}; /* DTYPE_GEN_WRITE1 */
static char cmd_c596[2] = {0xC5, 0x96};
static char shift_address04[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char cmd_c0xx[6] = {0xC0, 0x00, 0x58,
                           0x00, 0x14, 0x16}; /* DTYPE_GEN_LWRITE */
static char shift_address05[2] = {0x00, 0xC0}; /* DTYPE_GEN_WRITE1 */
static char cmd_cb14[2] = {0xCB, 0x14};
static char shift_address06[2] = {0x00, 0xC2}; /* DTYPE_GEN_WRITE1 */
static char shift_address07[2] = {0x00, 0xC3}; /* DTYPE_GEN_WRITE1 */
static char shift_address08[2] = {0x00, 0xC4}; /* DTYPE_GEN_WRITE1 */
static char shift_address09[2] = {0x00, 0xC5}; /* DTYPE_GEN_WRITE1 */
static char shift_address10[2] = {0x00, 0xC6}; /* DTYPE_GEN_WRITE1 */
static char shift_address11[2] = {0x00, 0xD5}; /* DTYPE_GEN_WRITE1 */
static char shift_address12[2] = {0x00, 0xD7}; /* DTYPE_GEN_WRITE1 */
static char shift_address13[2] = {0x00, 0xD8}; /* DTYPE_GEN_WRITE1 */
static char shift_address14[2] = {0x00, 0xD9}; /* DTYPE_GEN_WRITE1 */
static char shift_address15[2] = {0x00, 0xDA}; /* DTYPE_GEN_WRITE1 */
static char shift_address16[2] = {0x00, 0xDB}; /* DTYPE_GEN_WRITE1 */
static char shift_address17[2] = {0x00, 0x81}; /* DTYPE_GEN_WRITE1 */
static char cmd_f514[2] = {0xF5, 0x14};
static char shift_address18[2] = {0x00, 0x83}; /* DTYPE_GEN_WRITE1 */
static char shift_address19[2] = {0x00, 0x85}; /* DTYPE_GEN_WRITE1 */
static char shift_address20[2] = {0x00, 0x87}; /* DTYPE_GEN_WRITE1 */
static char shift_address21[2] = {0x00, 0x89}; /* DTYPE_GEN_WRITE1 */
static char shift_address22[2] = {0x00, 0x8B}; /* DTYPE_GEN_WRITE1 */
static char cmd_f520[2] = {0xF5, 0x20};
static char shift_address23[2] = {0x00, 0x91}; /* DTYPE_GEN_WRITE1 */
static char shift_address24[2] = {0x00, 0x93}; /* DTYPE_GEN_WRITE1 */
static char shift_address25[2] = {0x00, 0x95}; /* DTYPE_GEN_WRITE1 */
static char shift_address26[2] = {0x00, 0x97}; /* DTYPE_GEN_WRITE1 */
static char shift_address27[2] = {0x00, 0x99}; /* DTYPE_GEN_WRITE1 */
static char shift_address28[2] = {0x00, 0xA1}; /* DTYPE_GEN_WRITE1 */
static char shift_address29[2] = {0x00, 0xA3}; /* DTYPE_GEN_WRITE1 */
static char shift_address30[2] = {0x00, 0xA5}; /* DTYPE_GEN_WRITE1 */
static char shift_address31[2] = {0x00, 0xA7}; /* DTYPE_GEN_WRITE1 */
static char shift_address32[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char shift_address33[2] = {0x00, 0xB3}; /* DTYPE_GEN_WRITE1 */
static char shift_address34[2] = {0x00, 0xB5}; /* DTYPE_GEN_WRITE1 */
static char shift_address35[2] = {0x00, 0xB7}; /* DTYPE_GEN_WRITE1 */
static char shift_address36[2] = {0x00, 0xB9}; /* DTYPE_GEN_WRITE1 */
/* FIH-SW2-MM-KW-AUO_INIT_20120427-00-} */
/* FIH-SW2-MM-KW-AUO_INIT_Gamma2.5-20120510-00+{ */
static char auo_gamma_cmd1[17] = {0xE1, 0x09, 0x0E, 0x13, 0x0C, 0x06,
                                  0x0C, 0x0A, 0x09, 0x04, 0x07, 0x0F,
                                  0x07, 0x0E, 0x0E, 0x0A, 0x08};
static char auo_gamma_cmd2[17] = {0xE2, 0x09, 0x0E, 0x13, 0x0C, 0x06,
                                  0x0C, 0x0A, 0x09, 0x04, 0x07, 0x0F,
                                  0x07, 0x0E, 0x0E, 0x0A, 0x08};
/* FIH-SW2-MM-KW-AUO_INIT_Gamma2.5-20120510-00-} */
/* ----------- [For AUO panel setting End] ----------- */

/* ----------- [For CMI panel setting Start] ----------- */
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-[+ */
static char shift_addr00[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char cmi_extc[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char shift_addr80[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char cmi_cmd2[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-]- */

/* FIH-SW2-MM-KW-CMI_INIT_Gamma2.5-20120504-00+{ */
/* Set GVDD/NGVDD */
static char gamma_cmd1[3] = {0xD8, 0xAF, 0xAF};
/* Gammm Correction Characteristics Setting (2.2+) */
static char gamma_cmd2[17] = {0xE1, 0x00, 0x14, 0x19, 0x0F, 0x06,
                              0x0C, 0x09, 0x09, 0x04, 0x07, 0x11,
                              0x09, 0x10, 0x11, 0x0A, 0x04};
/* Gammm Correction Characteristics Setting (2.2-) */
static char gamma_cmd3[17] = {0xE2, 0x00, 0x14, 0x19, 0x0F, 0x06,
                              0x0C, 0x09, 0x09, 0x04, 0x07, 0x11,
                              0x09, 0x10, 0x11, 0x0A, 0x04};
/* Gammm Correction Characteristics Setting (2.5+) */
static char gamma_cmd4[17] = {0xE5, 0x00, 0x14, 0x1A, 0x0E, 0x06,
                              0x0C, 0x09, 0x06, 0x08, 0x0B, 0x0F,
                              0x09, 0x0E, 0x0C, 0x06, 0x04};
/* Gammm Correction Characteristics Setting (2.5-) */
static char gamma_cmd5[17] = {0xE6, 0x00, 0x14, 0x1A, 0x0E, 0x06,
                              0x0C, 0x09, 0x06, 0x08, 0x0B, 0x0F,
                              0x09, 0x0E, 0x0C, 0x06, 0x04};
/* Gamma  Set */
static char gamma_cmd6[2] = {0x26, 0x04};
/* FIH-SW2-MM-KW-CMI_INIT_Gamma2.5-20120504-00-} */
/* ----------- [For CMI panel setting End] ----------- */

/* FIH-SW2-MM-NC-BKL_PWM-01-[+ */
/* Program PWM frequency to about 33 KHz */
static char shift_addressB1[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char pwm_para3[2] = {0xC6, 0x01}; /* DTYPE_GEN_WRITE1 */
static char shift_addressB4[2] = {0x00, 0xB4}; /* DTYPE_GEN_WRITE1 */
static char pwm_para5[2] = {0xC6, 0x10}; /* DTYPE_GEN_WRITE1 */
/* FIH-SW2-MM-NC-BKL_PWM-01-]- */

static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
static char write_ctrl_display[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */
/* FIH-SW2-MM-NC-LCM_INIT-00-[+ */
static char write_display_brightness[2] = {0x51, 0x00}; /* DTYPE_DCS_WRITE1 */
/* FIH-SW2-MM-NC-LCM_INIT-00-]- */
static char orise_manufacture_idDA[2] = {0xDA, 0x00}; /* DTYPE_DCS_READ */

/* FIH-SW2-MM-KW-AUO_INIT_20120427-00+{ */
static struct dsi_cmd_desc orise_auo_video_on_cmds[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(extc), extc},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address01), shift_address01},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(cmd2), cmd2},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address02), shift_address02},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_c430), cmd_c430},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address03), shift_address03},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_c596), cmd_c596},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address04), shift_address04},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_c0xx), cmd_c0xx},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address05), shift_address05},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address06), shift_address06},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address07), shift_address07},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address08), shift_address08},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address09), shift_address09},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address10), shift_address10},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address11), shift_address11},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address12), shift_address12},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address13), shift_address13},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address14), shift_address14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address15), shift_address15},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address16), shift_address16},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_cb14), cmd_cb14},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address17), shift_address17},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address18), shift_address18},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address19), shift_address19},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address20), shift_address20},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address21), shift_address21},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address22), shift_address22},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f520), cmd_f520},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address23), shift_address23},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address24), shift_address24},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address25), shift_address25},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address26), shift_address26},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address27), shift_address27},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address28), shift_address28},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address29), shift_address29},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address30), shift_address30},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address31), shift_address31},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address32), shift_address32},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address33), shift_address33},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address34), shift_address34},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address35), shift_address35},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address36), shift_address36},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(cmd_f514), cmd_f514},
/* FIH-SW2-MM-KW-AUO_INIT_Gamma2.5-20120510-00+{ */
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address00), shift_address00},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(auo_gamma_cmd1), auo_gamma_cmd1},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_address00), shift_address00},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(auo_gamma_cmd2), auo_gamma_cmd2},
/* FIH-SW2-MM-KW-AUO_INIT_Gamma2.5-20120510-00-} */
/* FIH-SW2-MM-NC-BKL_PWM-01-[+ */
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addressB1), shift_addressB1},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(pwm_para3), pwm_para3},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addressB4), shift_addressB4},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(pwm_para5), pwm_para5},
/* FIH-SW2-MM-NC-BKL_PWM-01-]- */
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
        sizeof(write_ctrl_display), write_ctrl_display},
    {DTYPE_DCS_WRITE, 1, 0, 0, 120,
        sizeof(exit_sleep), exit_sleep},
    {DTYPE_DCS_WRITE, 1, 0, 0, 1,
        sizeof(display_on), display_on}
};
/* FIH-SW2-MM-KW-AUO_INIT_20120427-00-} */

static struct dsi_cmd_desc orise_cmi_video_on_cmds[] = {
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-[+ */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmi_extc), cmi_extc},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmi_cmd2), cmi_cmd2},
/* FIH-SW2-MM-NC-BKL_PWM-01-[+ */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addressB1), shift_addressB1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para3), pwm_para3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addressB4), shift_addressB4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para5), pwm_para5},
/* FIH-SW2-MM-NC-BKL_PWM-01-]- */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_ctrl_display), write_ctrl_display},
/* FIH-SW2-MM-KW-CMI_INIT_Gamma2.5-20120504-00+{ */
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addr00), shift_addr00},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(gamma_cmd1), gamma_cmd1},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addr00), shift_addr00},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(gamma_cmd2), gamma_cmd2},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addr00), shift_addr00},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(gamma_cmd3), gamma_cmd3},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addr00), shift_addr00},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(gamma_cmd4), gamma_cmd4},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addr00), shift_addr00},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(gamma_cmd5), gamma_cmd5},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0,
        sizeof(shift_addr00), shift_addr00},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0,
        sizeof(gamma_cmd6), gamma_cmd6},
/* FIH-SW2-MM-KW-CMI_INIT_Gamma2.5-20120504-00-} */
	{DTYPE_DCS_WRITE, 1, 0, 0, 20,
		sizeof(display_on), display_on}
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-]- */
};

static struct dsi_cmd_desc orise_video_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 20,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc orise_video_bkl_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_display_brightness), write_display_brightness}
};

static struct dsi_cmd_desc orise_ReadDA = {
	DTYPE_DCS_READ, 1, 0, 1, 20, sizeof(orise_manufacture_idDA), orise_manufacture_idDA};

static int mipi_orise_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	char retDA = 0;

	tp = &orise_tx_buf;
	rp = &orise_rx_buf;

	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);
	mipi_dsi_cmds_rx(mfd, tp, rp, &orise_ReadDA, 1);
	retDA = *((char *) rp->data);

	printk(KERN_ALERT "[DISPLAY] Panel ID = 0x%02x\n", retDA);
	return retDA;
}

/* FIH-SW-MM-VH-DISPLAY-22*[ */
static int mipi_orise_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	int rc = 0;  /* FIH-SW2-MM-NC-LCM_INIT-00 */

	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (display_initialize)
		return 0;

	if (gPanelModel == 0) {
		gPanelModel = mipi_orise_manufacture_id(mfd);
		printk(KERN_ALERT "[DISPLAY] %s gPanelModel = %d\n", __func__, gPanelModel);
	}

	if (gPanelModel == CMI_PANEL_ID) {
		rc = mipi_dsi_cmds_tx(mfd, &orise_tx_buf, orise_cmi_video_on_cmds,
			ARRAY_SIZE(orise_cmi_video_on_cmds));
	} else {
		rc = mipi_dsi_cmds_tx(mfd, &orise_tx_buf, orise_auo_video_on_cmds,
			ARRAY_SIZE(orise_auo_video_on_cmds));
	}
	printk(KERN_ALERT "[DISPLAY] Finish sending dsi commands, rc = %d\n", rc);

	display_initialize = 1;

	if (rc > 0)
		rc = 0;

	return rc;
}
/* FIH-SW-MM-VH-DISPLAY-22*] */

static int mipi_orise_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);
	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (!display_initialize)
		return 0;

	mipi_set_tx_power_mode(0);
	mipi_dsi_cmds_tx(mfd, &orise_tx_buf, orise_video_off_cmds,
			ARRAY_SIZE(orise_video_off_cmds));
	mipi_set_tx_power_mode(1);

	display_initialize = 0;

	return 0;
}

static void mipi_orise_lcd_backlight(struct msm_fb_data_type *mfd)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s, set backlight level to %d\n",
			__func__, mfd->bl_level);

/* FIH-SW2-MM-NC-LCM_INIT-01-[+ */
	if (!display_initialize)
		return;

	write_display_brightness[1] = BKL_PWM[mfd->bl_level];  /* Duty_Cycle */

	down(&mfd->dma->mutex);
	mipi_set_tx_power_mode(0);
	mipi_dsi_cmds_tx(mfd, &orise_tx_buf, orise_video_bkl_cmds,
			ARRAY_SIZE(orise_video_bkl_cmds));
	mipi_set_tx_power_mode(1);
	up(&mfd->dma->mutex);
/* FIH-SW2-MM-NC-LCM_INIT-01-]- */
}

static int __devinit mipi_orise_lcd_probe(struct platform_device *pdev)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);

	if (pdev->id == 0) {
		mipi_orise_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_orise_lcd_probe,
	.driver = {
		.name   = "mipi_orise",
	},
};

static struct msm_fb_panel_data orise_panel_data = {
	.on		= mipi_orise_lcd_on,
	.off	= mipi_orise_lcd_off,
	.set_backlight = mipi_orise_lcd_backlight,
};

static int ch_used[3];

int mipi_orise_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_orise", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	orise_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &orise_panel_data,
		sizeof(orise_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_orise_lcd_init(void)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);
	mipi_dsi_buf_alloc(&orise_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&orise_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_orise_lcd_init);
