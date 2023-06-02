/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *  滤波库
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-04     cagt       the first version
 */
#ifndef APPLICATIONS_INC_FILTER_LIB_H_
#define APPLICATIONS_INC_FILTER_LIB_H_


typedef struct Kalman_argument
{

    float Q_in;                //陀螺仪角度噪声协方差
    float Q_d_in;                 //陀螺仪漂移噪声协方差
    float R_in;                  //加速度计测量噪声协方差
    float dt;                     //积分时间（滤波器采样时间）
    char  C_0;
    float Q_bias,out_err;
    float PCt_0,PCt_1,E;
    float K_0,K_1,t_0,t_1;
    float Pdot[4];
    float PP[2][2];
}Kalman_argument_t;

typedef struct Mahony_argument
{
    float q0,q1,q2,q3;
    float ExInt,EyInt,EzInt;
    float kp,ki;
    float haftT;        //采样时间的一半
}Mahony_argument_t;


/* @function 卡尔曼参数初始化
 * @param argument 卡尔曼的滤波参数
 * */
void Kalman_argument_init(struct Kalman_argument * argument);


/* @function 卡尔曼滤波
 * @param out 滤波后的输出
 * @param d_in 输入量的微分
 * @param in 输入量
 * @param argument 卡尔曼的滤波参数
 *
 * 如果是角度,则传入参数需要全部为弧度制
 * */
void Kalman_Filter(float *out,float d_in,float in,struct Kalman_argument * argument);

/* @function Mahony滤波参数初始化
 * @param kp ki,pi控制器参数
 * @param T 采样时间
 * @param argument Mahony参数
 * 如果是角度,则传入参数需要全部为弧度制
 * */
void MahonyInit(float kp,float ki,float T,struct Mahony_argument* argument);

/* @function Mahony滤波
 * @param *pitch,*roll,*yaw 作为融合后的结果,单位为弧度制
 * @param g_ 输入角速度 弧度制
 * @param a_ 输入加速度 m^2
 * @param argument Mahony参数
 *
 *
 * 如果是角度,则传入参数需要全部为弧度制
 * */
void MahonyFilter(float *pitch,float * roll,float * yaw,float gx, float gy, float gz, float ax, float ay, float az,struct Mahony_argument* argument);


#endif /* APPLICATIONS_INC_FILTER_LIB_H_ */
