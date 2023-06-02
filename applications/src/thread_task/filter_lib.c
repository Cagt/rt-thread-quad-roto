/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-04     cagt       the first version
 */

#include "../inc/filter_lib.h"
#include <string.h>
#include <math.h>

/* @function 卡尔曼滤波
 * @param out 滤波后的输出
 * @param d_in 输入量的微分
 * @param in 输入量
 * @param argument 卡尔曼的滤波参数
 *
 * 如果是角度,则传入参数需要为弧度制
 * */
void Kalman_argument_init(struct Kalman_argument * argument)
{

    /*
     * char  C_0_pitch=1;                        //H矩阵的一个数
        float Q_bias_pitch=0,Angle_err_pitch=0;         //Q_bias为陀螺仪漂移
        float PCt_0_pitch=0,PCt_1_pitch=0,E_pitch=0;          //中间变量
        float K_0_pitch=0,K_1_pitch=0,t_0_pitch=0,t_1_pitch=0;      //K是卡尔曼增益，t是中间变量
        float Pdot_pitch[4] = {0,0,0,0};          //计算P矩阵的中间变量
        float PP_pitch[2][2]={    {1,0},          //公式中P矩阵即X的协方差
                            {0,1}   };
     *
     *
     * */
    memset(argument,0,sizeof(struct Kalman_argument));

    argument->dt = 0.005;
    argument->Q_in = 0.001;
    argument->Q_d_in = 0.003;
    argument->R_in = 0.5;
    argument->C_0 = 1;
    argument->PP[0][0] = 1;
    argument->PP[0][1] = 0;
    argument->PP[1][0] = 0;
    argument->PP[1][1] = 1;
}

/* @function 卡尔曼参数初始化
 * @param argument 卡尔曼的滤波参数
 * */
void Kalman_Filter(float *out,float d_in,float in,struct Kalman_argument * argument)
{
      (*out) += (d_in - argument->Q_bias) * argument->dt;

      argument->Pdot[0] =  argument->Q_in - argument->PP[0][1] - argument->PP[1][0];
      argument->Pdot[1] = -argument->PP[1][1];
      argument->Pdot[2] = -argument->PP[1][1];
      argument->Pdot[3] =  argument->Q_d_in;

      argument->PP[0][0] += argument->Pdot[0] * argument->dt;
      argument->PP[0][1] += argument->Pdot[1] * argument->dt;
      argument->PP[1][0] += argument->Pdot[2] * argument->dt;
      argument->PP[1][1] += argument->Pdot[3] * argument->dt;

      argument->PCt_0 = argument->C_0 * argument->PP[0][0];     //矩阵乘法的中间变量
      argument->PCt_1 = argument->C_0 * argument->PP[1][0];     //C_0=1
      argument->E = argument->R_in + argument->C_0 * argument->PCt_0;   //分母
      argument->K_0 = argument->PCt_0 / argument->E;            //Angle卡尔曼增益
      argument->K_1 = argument->PCt_1 / argument->E;            //Q_bis卡尔曼增益
  //


      /*
          X(k|k)= X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1))
      */
      argument->out_err = in - (*out);
      (*out) += argument->K_0 * argument->out_err;   //计算最优角度
      argument->Q_bias += argument->K_1 * argument->out_err;  //计算最优零漂




      argument->t_0 = argument->PCt_0;                //矩阵计算中间变量，相当于a
      argument->t_1 = argument->C_0 * argument->PP[0][1];       //矩阵计算中间变量，相当于b



      argument->PP[0][0] -= argument->K_0 * argument->t_0;
      argument->PP[0][1] -= argument->K_0 * argument->t_1;
      argument->PP[1][0] -= argument->K_1 * argument->t_0;
      argument->PP[1][1] -= argument->K_1 * argument->t_1;


}


/* @function Mahony滤波参数初始化
 * @param kp ki,pi控制器参数
 * @param T 采样时间
 * @param argument Mahony参数
 * 如果是角度,则传入参数需要全部为弧度制
 * */
void MahonyInit(float kp,float ki,float T,struct Mahony_argument* argument)
{
    memset(argument,0,sizeof(struct Mahony_argument));
    argument->kp = kp;
    argument->ki = ki;
    argument->haftT = (T / 2.0f);
    argument->q0 = 1;
}


/* @function Mahony滤波
 * @param *pitch,*roll,*yaw 作为融合后的结果,单位为弧度制
 * @param g_ 输入角速度 弧度制
 * @param a_ 输入加速度 m^2
 * @param argument Mahony参数
 *
 *
 * 如果是角度,则传入参数需要全部为弧度制
 * */
void MahonyFilter(float *pitch,float * roll,float * yaw,float gx, float gy, float gz, float ax, float ay, float az,struct Mahony_argument* argument)
{
    float norm = 0;
    float vx = 0, vy = 0, vz = 0;
    float ex = 0, ey = 0, ez = 0;

    // normalise the measurements
   norm = sqrt(ax * ax + ay * ay + az * az);

    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;


    vx = 2.0 * (argument->q1 * argument->q3 - argument->q0 * argument->q2);
    vy = 2.0 * (argument->q0 * argument->q1 + argument->q2 * argument->q3);
    vz = argument->q0 * argument->q0 - argument->q1 * argument->q1 - argument->q2 * argument->q2 + argument->q3 * argument->q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    argument->ExInt = argument->ExInt + ex * argument->ki;
    argument->EyInt = argument->EyInt + ey * argument->ki;
    argument->EzInt = argument->EzInt + ez * argument->ki;
    gx = gx + argument->kp * ex + argument->ExInt;
    gy = gy + argument->kp * ey + argument->EyInt;
    gz = gz + argument->kp * ez + argument->EzInt;


    argument->q0 = argument->q0 + (-argument->q1 * gx - argument->q2 * gy - argument->q3 * gz) * argument->haftT;
    argument->q1 = argument->q1 + (argument->q0 * gx + argument->q2 * gz - argument->q3 * gy) * argument->haftT;
    argument->q2 = argument->q2 + (argument->q0 * gy - argument->q1 * gz + argument->q3 * gx) * argument->haftT;
    argument->q3 = argument->q3 + (argument->q0 * gz + argument->q1 * gy - argument->q2 * gx) * argument->haftT;
    norm = sqrt(argument->q0 * argument->q0 + argument->q1 * argument->q1 + argument->q2 * argument->q2 + argument->q3 * argument->q3);

    argument->q0 = argument->q0 / norm;
    argument->q1 = argument->q1 / norm;
    argument->q2 = argument->q2 / norm;
    argument->q3 = argument->q3 / norm;

    if(roll)
        (*roll) = atan2f(2*argument->q2*argument->q3+2*argument->q0*argument->q1,-2*argument->q1*argument->q1-2*argument->q2*argument->q2+1);    //弧度制
    if(pitch)
        (*pitch) = asinf(-2*argument->q1*argument->q3+2*argument->q0*argument->q2);
    if(yaw)
        (*yaw)= atan2(2*argument->q1*argument->q2+2*argument->q0*argument->q3,-2*argument->q2*argument->q2-2*argument->q3*argument->q3+1);


}

