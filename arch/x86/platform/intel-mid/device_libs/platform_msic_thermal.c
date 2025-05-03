/*
 * platform_msic_thermal.c: msic_thermal platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_thermal.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include "platform_msic.h"
#include "platform_msic_thermal.h"
#include <linux/HWVersion.h>

/* ctp thermal sensor list */
#ifdef CONFIG_TF103CG
static struct intel_mid_thermal_sensor ctp_sensors[] = {
	{
		.name = MSIC_DIE_NAME,
		.index = 0,
		.slope = 368,
		.intercept = 219560,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = SKIN0_NAME,
		.index = 1,
		.slope = 1000,
		.intercept = 0,
		.adc_channel = 0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 2,
		.slope = 1000,
		.intercept = 0,
		.adc_channel = 0x05 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},	
	{
		.name = BPTHERM_NAME,
		.index = 3,
		.slope = 788,
		.intercept = 5065,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = bptherm_temp_correlation,
		.direct = false,
	},

};
#else
static struct intel_mid_thermal_sensor ctp_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 410,
		.intercept = 16808,
		.adc_channel = 0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 665,
		.intercept = 8375,
		.adc_channel = 0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 2,
		.slope = 368,
		.intercept = 219560,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = BPTHERM_NAME,
		.index = 3,
		.slope = 788,
		.intercept = 5065,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = bptherm_temp_correlation,
		.direct = false,
	},

};
#endif

static struct intel_mid_thermal_sensor ctp_170cg_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x05 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = "SYSTHERM2",
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x06 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = "SYSTHERM3",
		.index = 3,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x07 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 4,
		.slope = 368,
		.intercept = 219560,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = BPTHERM_NAME,
		.index = 5,
		.slope = 788,
		.intercept = 5065,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = bptherm_temp_correlation,
		.direct = false,
	},

};
/* ctp thermal sensor list for FE170C without modem*/
static struct intel_mid_thermal_sensor ctp_170c_sensors[] = {
	{
		.name = SKIN1_NAME,
		.index = 0,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x05 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = "SYSTHERM2",
		.index = 1,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x06 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = "SYSTHERM3",
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x07 | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 3,
		.slope = 368,
		.intercept = 219560,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = BPTHERM_NAME,
		.index = 4,
		.slope = 788,
		.intercept = 5065,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = bptherm_temp_correlation,
		.direct = false,
	},

};
/* mfld thermal sensor list */
static struct intel_mid_thermal_sensor mfld_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 851,
		.intercept = 2800,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 806,
		.intercept = 1800,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_SYS_NAME,
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x0A | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 3,
		.slope = 368,
		.intercept = 219560,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},

};

/* LEX thermal sensor list */
static struct intel_mid_thermal_sensor lex_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 851,
		.intercept = 2800,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 806,
		.intercept = 1800,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_SYS_NAME,
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x0A | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 3,
		.slope = 368,
		.intercept = 219560,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},

};


static struct intel_mid_thermal_platform_data pdata[] = {
	[mfld_thermal] = {
		.num_sensors = 4,
		.sensors = mfld_sensors,
		.gpu_cooling = false,
	},
	[ctp_thermal] = {
		.num_sensors = 4,
		.sensors = ctp_sensors,
		.gpu_cooling = true,
	},
	[lex_thermal] = {
		.num_sensors = 4,
		.sensors = lex_sensors,
		.gpu_cooling = false,
	},
};

static struct intel_mid_thermal_platform_data asus_pdata[] = {
        [fe170cg_thermal] = {
                .num_sensors = 6,
                .sensors = ctp_170cg_sensors,
                .gpu_cooling = true,
        },
        [fe170c_thermal] = {
                .num_sensors = 5,
                .sensors = ctp_170c_sensors,
                .gpu_cooling = true,
        },
};

extern int Read_PROJ_ID(void);

void __init *msic_thermal_platform_data(void *info)
{
	struct platform_device *pdev;
	int PROJ_ID;
	PROJ_ID = Read_PROJ_ID();

	pdev = platform_device_alloc(MSIC_THERM_DEV_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			MSIC_THERM_DEV_NAME);
		return NULL;
	}

	if (platform_device_add(pdev)) {
		pr_err("failed to add thermal platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	if (PROJ_ID == PROJ_ID_FE170CG)
		pdev->dev.platform_data = &asus_pdata[fe170cg_thermal];
	else if (PROJ_ID == PROJ_ID_ME170C || PROJ_ID == PROJ_ID_ME70CX)
		pdev->dev.platform_data = &asus_pdata[fe170c_thermal];
	else if (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO) ||
		INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG))
		pdev->dev.platform_data = &pdata[vb_thermal];
	else if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
			(INTEL_MID_BOARD(1, TABLET, CLVT)))
		pdev->dev.platform_data = &pdata[ctp_thermal];
	else if (INTEL_MID_BOARD(2, PHONE, MFLD, LEX, ENG) ||
			(INTEL_MID_BOARD(2, PHONE, MFLD, LEX, PRO)))
		pdev->dev.platform_data = &pdata[lex_thermal];
	else
		pdev->dev.platform_data = &pdata[mfld_thermal];

	register_rpmsg_service("rpmsg_mid_thermal", RPROC_SCU, RP_MSIC_THERMAL);

	return 0;
}
