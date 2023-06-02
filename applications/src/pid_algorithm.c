#include <includes.h>


void PID_init(PID_TypDef* sptr)
{
			sptr->Kp = 0.0;
			sptr->Ki = 0.0;
	    sptr->Kd = 0.0;
		  sptr->Bias      = 0.0;
			sptr->Integral  = 0.0;
		  sptr->Last_Bias = 0.0;
      sptr->Pre_Bias  = 0.0;

}

void PID_Set(PID_TypDef *PID,float Kp,float Ki,float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
}

float IncPID(int Encoder,int Target,PID_TypDef* sptr)
{
  float	Pwm;
  sptr->Bias = Target - Encoder;
  Pwm = sptr->Kp * (sptr->Bias-sptr->Last_Bias)
         +sptr->Ki * sptr->Bias
         +sptr->Kd * (sptr->Bias-2*sptr->Last_Bias+sptr->Pre_Bias);

  sptr->Pre_Bias=sptr->Last_Bias;
  sptr->Last_Bias=sptr->Bias;

  return(Pwm);

}


float PstPID(float Angle, float Target,PID_TypDef* sptr,int8_t ClearZeroOffset)
{
	float Pwm;

	sptr->Bias = Target -Angle ;
	sptr->Integral += sptr->Bias;

	if(sptr->Integral > 2)
	    sptr->Integral = 2;
	else if(sptr->Integral < -2)
        sptr->Integral = -2;

	if(ClearZeroOffset && sptr->Bias < 0.03 && sptr->Bias > -0.03)     //如果误差值小于一定值,认为是抖动产生的
	    sptr->Bias = 0;

	Pwm = sptr->Kp   * sptr->Bias
         +sptr->Ki * sptr->Integral
         +sptr->Kd * (sptr->Bias-sptr->Last_Bias);

	sptr->Last_Bias=sptr->Bias;

	return(Pwm);

}

