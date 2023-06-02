/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-19     cagt       the first version
 */


#ifndef __PID_ALG_H_
#define __PID_ALG_H_

#include "includes.h"



typedef struct
{
		 float   Kp;
		 float   Ki;
		 float   Kd;
		 float			Integral;
		 float      Bias;
		 float      Last_Bias;
		 float      Pre_Bias;
}PID_TypDef;


void PID_init(PID_TypDef* sptr);
void PID_Set(PID_TypDef *PID,float Kp,float Ki,float Kd);
float IncPID(int Encoder,int Target,PID_TypDef* sptr);
float PstPID(float Angle, float Target,PID_TypDef* sptr,int8_t ClearZeroOffset);




#endif
