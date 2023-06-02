/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-19     cagt       the first version
 */
#ifndef APPLICATIONS_INC_AHRS_TASK_H_
#define APPLICATIONS_INC_AHRS_TASK_H_

#include "sensor_inven_mpu6xxx.h"
#include <includes.h>
#include "sensor_bs_bmp280.h"

#define C_FITTER 0.7   //一节互补系数
#define PI 3.14159f
#define DT 0.001f
#define GROY_CONVER 2000.0f / 32768.0f
#define ACC_CONVER 2.0*9.8f / 32768.0f
#define AHRS_KP 25.00f                       // proportional gain governs rate of convergence to accelerometer/magnetometer
#define AHRS_KI 5.00f                         // integral gain governs rate of convergence of gyroscope biases
#define  halfT 0.0005;                 // half the sample period


#define P0 101325.00f

typedef struct
{
    float pitch;
    float yaw;
    float roll;

}angle_t;

typedef struct
{
    float x;
    float y;
    float z;
    float zero_data;

}mpu_3axes;

typedef struct
{
    mpu_3axes gyro;  /*三轴角加速度*/
    mpu_3axes acc;   /*三轴加速度*/
    angle_t angle;              /*三轴角度*/
    angle_t raw_angle;
    float height;             /*高度*/

}ahrs_t;

typedef struct
{
    struct rt_sensor_config cfg;
    struct rt_sensor_data t_dat;
    struct rt_sensor_data b_dat;
    rt_device_t temp_handler;
    rt_device_t baro_handler;
    int32_t zero_value;
}Bmp;

typedef struct
{
    struct mpu6xxx_device *mpu;     //MPUXXX句柄
    Bmp bmp;    //BMP气压计
    ahrs_t ahrs_data;
}ahrs_handler_t;


ahrs_handler_t * get_ahrs_handler_point(void);
ahrs_handler_t * ahrs_init(void);


extern struct mpu6xxx_device* i2c_bus;

#endif /* APPLICATIONS_INC_AHRS_TASK_H_ */
