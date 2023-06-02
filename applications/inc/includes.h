/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-11     ChinaGhost       the first version
 */
#ifndef APPLICATIONS_INC_INCLUDES_H_

#define APPLICATIONS_INC_INCLUDES_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <pid_algorithm.h>
#include <stdlib.h>
#include <ahrs_task.h>
#include <control_task.h>
#include <blue_tooth_debug_task.h>


#define THREAD_STACK 1024
#define THREAD_PRIO 5
#define THREAD_TICK 5

#define DEBUG_PID 1



#endif /* APPLICATIONS_INC_INCLUDES_H_ */
