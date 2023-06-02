/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-10     RT-Thread    first version
 */

#include "../inc/includes.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>





int main(void)
{
    rt_pin_mode(21,PIN_MODE_OUTPUT);
    rt_pin_mode(20,PIN_MODE_OUTPUT);

    rt_pin_write(20, PIN_HIGH);
    rt_pin_mode(21,PIN_LOW);
    ahrs_init();
    control_init();
    bluetooth_init();
    //while(1){};
    return RT_EOK;
}
