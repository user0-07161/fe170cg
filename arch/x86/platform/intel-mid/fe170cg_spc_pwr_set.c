/*
 *   fe170cg_spc_pwr_set.c: ASUS FE170CG hardware power related initialization code
 *
 * (C) Copyright 2014 ASUS Corporation
 * Author: Kunyang_Fan
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#define PIN_3VSUS_SYNC  45
#define PIN_5VSUS_SYNC  159
#define PIN_5VSUS_EN    160

static int __init fe170cg_specific_gpio_setting_init()
{

#ifdef CONFIG_FE170CG

       gpio_request(PIN_3VSUS_SYNC, "P_+3VSO_SYNC_5");
       gpio_request(PIN_5VSUS_SYNC, "P_+5VSO_SYNC_EN");
       gpio_direction_output(PIN_3VSUS_SYNC, 0);
       gpio_direction_output(PIN_5VSUS_SYNC, 0);
#endif
       return 0;

}

module_init(fe170cg_specific_gpio_setting_init);
