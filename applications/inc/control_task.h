/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-11     ChinaGhost       the first version
 */
#ifndef APPLICATIONS_INC_CONTROL_TASK_H_
#define APPLICATIONS_INC_CONTROL_TASK_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <includes.h>
/*
   * 无人机电机编号
 *
 *   3\   /0
 *     \ /
 *     / \
 *   2/    \1
 * */

#define MOTOR_PWM_DEV "pwm3"
#define LEFT_FRONT_MOTOR_NUM 3
#define LEFT_BACK_MOTOR_NUM 2
#define RIGHT_BACK_MOTOR_NUM 1
#define RIGHT_FRONT_MOTOR_NUM 0

#define LEFT_FRONT_MOTOR_PWM_MIN 90
#define LEFT_BACK_MOTOR_PWM_MIN 90
#define RIGHT_BACK_MOTOR_PWM_MIN 91
#define RIGHT_FRONT_MOTOR_PWM_MIN 91
#define STARTUP_PWM 20



#define LEFT_FRONT_MOTOR_PWM_UNLOCK 97
#define LEFT_BACK_MOTOR_PWM_UNLOCK 97
#define RIGHT_BACK_MOTOR_PWM_UNLOCK 97
#define RIGHT_FRONT_MOTOR_PWM_UNLOCK 97

#define LEFT_FRONT_MOTOR_PWM_LOCK 80
#define LEFT_BACK_MOTOR_PWM_LOCK 80
#define RIGHT_BACK_MOTOR_PWM_LOCK 80
#define RIGHT_FRONT_MOTOR_PWM_LOCK 80


#define PWM_PERIOD 20000000     //20ms
#define PWM_K      10000       //控制单位为0.1ms

typedef struct
{
    struct rt_device_pwm *outputPwm;        //PWM输出器
    uint64_t output[4];                   //PWM计算输出
}Motor_t;


enum FLYING_MODE        /*飞行模式*/
{
    BLUETOOTH_MOVING,    /*蓝牙控制模式*/
    BLUETOOTH_DEBUG,    /*蓝牙调试模式*/
    RC_MOVING,            /*遥控器控制模式*/
    GYRO_LOOP_MODE,     /*角速度闭环控制模式*/
    ANGLE_LOOP_MODE    /*角度闭环控制模式*/
};

enum LOCK_STATUS        /*无人机锁的状态*/
{
    LOCK = 0x00,
    UNLOCK
};

typedef struct
{
    PID_TypDef pitch_pid;
    PID_TypDef roll_pid;
    PID_TypDef yaw_pid;
    PID_TypDef gyro_x_pid;
    PID_TypDef gyro_y_pid;
    PID_TypDef height_pid;
}Pid_t;

typedef struct{
    ahrs_t quadrotor_status;        //无人机目标姿态
    Pid_t ctl_pid;

    Motor_t motor;               //无人机的电机控制
    uint8_t flying_mode;            //无人机飞行
    uint8_t lock_status;            //无人机锁的状态
    void (*pModeFun)(const ahrs_t* ,const ahrs_t *);    //无人机飞行模式状态
}Quadrotor_t;

rt_err_t control_init(void);      //初始化
Quadrotor_t* get_quadrotor_handler_point(void);

//static void no_moving_mode(const ahrs_t* purpose_status,const ahrs_t * current_status);

#endif /* APPLICATIONS_INC_KINETIC_CONTROL_H_ */
