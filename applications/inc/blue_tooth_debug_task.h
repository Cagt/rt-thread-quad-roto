/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-19     cagt       the first version
 */

#ifndef __BLUETOOTH_DEBUG_TASK_H_
#define __BLUETOOTH_DEBUG_TASK_H_

#include "includes.h"
#include <string.h>

int8_t setv(char *name,char * value);     //设置变量名


typedef struct
{
    uint16_t motor_no;      /*电机编号*/
    float kp;               /*设置的PID参数*/
    float kd;
    float ki;
    uint64_t duty;
    uint8_t select;
    volatile uint8_t startup_send_wave;

}sendPackage_t;     /*Debug设置参数调试*/

typedef struct
{

    ahrs_t purpose;
    rt_device_t send_device;

}BlueTooth_ctr_t;     /*Debug设置参数调试*/


rt_err_t bluetooth_init(void);
sendPackage_t * get_package_point(void);
rt_err_t bluetooth_input_notice(rt_device_t dev, rt_size_t size);
void serial_thread_entry(void *parameter);
void command_anys_task(void *command);
void wave_send_thread(void *parameter);
static void get_pid(void);

#endif
