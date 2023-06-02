#include <control_task.h>
#include <math.h>

#define PROETECT 1

#define BASIC_DYN  50  //基础动力
#define PITCH_ZERO -3

static void control_task(void *parm);
static void switch_mode(void);
static void unlock_motor(void);
static void deadband(uint64_t *value,uint64_t min,uint64_t max);

/*
 * @breif 角速度闭环函数
 * */
static void gyro_loopback(ahrs_t* purpose_status,const ahrs_t * current_status);
/*
 * @breif 角速度闭环函数
 * */
static void angle_loopback(ahrs_t* purpose_status,const ahrs_t * current_status);

static void bluetooth_control_mode( ahrs_t* purpose_status,const ahrs_t * current_status);
static void bluetooth_debug_mode(const ahrs_t* purpose_status,const ahrs_t * current_status);
static void bluetooth_control_mode_select_pid();
static void give_pwm(void);


rt_thread_t control_task_handler = RT_NULL;

Quadrotor_t quadrotor_handler;

#define LEFT_FRONT_MOTOR quadrotor_handler.motor.output[LEFT_FRONT_MOTOR_NUM]
#define LEFT_BACK_MOTOR  quadrotor_handler.motor.output[LEFT_BACK_MOTOR_NUM]
#define RIGHT_FRONT_MOTOR quadrotor_handler.motor.output[RIGHT_FRONT_MOTOR_NUM]
#define RIGHT_BACK_MOTOR quadrotor_handler.motor.output[RIGHT_BACK_MOTOR_NUM]



Quadrotor_t* get_quadrotor_handler_point(void)
{
    return (&quadrotor_handler);
}


static void deadband(uint64_t *value,uint64_t min,uint64_t max)
{
   if(max < (*value))
       (*value) = max;
   else if(min > (*value))
       (*value) = min;
}
int32_t jiaozhun = 0;
uint8_t step = 1;
uint8_t times = 0;
static void control_task(void *parm)
{
    rt_thread_mdelay(1000);
    for(int i = 0;i < 2;++i){
        for(int i = 0;i < 3;++i){
            rt_pwm_set(quadrotor_handler.motor.outputPwm, LEFT_FRONT_MOTOR_NUM+1, PWM_PERIOD, (LEFT_FRONT_MOTOR_PWM_UNLOCK+i)*PWM_K);
            rt_pwm_set(quadrotor_handler.motor.outputPwm, LEFT_BACK_MOTOR_NUM+1, PWM_PERIOD, (LEFT_BACK_MOTOR_PWM_UNLOCK+i)*PWM_K);
            rt_pwm_set(quadrotor_handler.motor.outputPwm, RIGHT_FRONT_MOTOR_NUM+1, PWM_PERIOD, (RIGHT_FRONT_MOTOR_PWM_UNLOCK+i)*PWM_K);
            rt_pwm_set(quadrotor_handler.motor.outputPwm, RIGHT_BACK_MOTOR_NUM+1, PWM_PERIOD, (RIGHT_BACK_MOTOR_PWM_UNLOCK+i)*PWM_K);

            rt_thread_mdelay(150);

        }
    }


    while(1)
    {

        if(quadrotor_handler.lock_status == LOCK)
        {

            rt_pwm_set(quadrotor_handler.motor.outputPwm, LEFT_FRONT_MOTOR_NUM+1, PWM_PERIOD, LEFT_FRONT_MOTOR_PWM_LOCK*PWM_K);
            rt_pwm_set(quadrotor_handler.motor.outputPwm, LEFT_BACK_MOTOR_NUM+1, PWM_PERIOD, LEFT_FRONT_MOTOR_PWM_LOCK*PWM_K);
            rt_pwm_set(quadrotor_handler.motor.outputPwm, RIGHT_FRONT_MOTOR_NUM+1, PWM_PERIOD, RIGHT_FRONT_MOTOR_PWM_LOCK*PWM_K);
            rt_pwm_set(quadrotor_handler.motor.outputPwm, RIGHT_BACK_MOTOR_NUM+1, PWM_PERIOD, RIGHT_BACK_MOTOR_PWM_LOCK*PWM_K);

        }
        else {
            switch_mode();
            bluetooth_control_mode_select_pid();
#if PROETECT

            if(get_ahrs_handler_point()->ahrs_data.angle.pitch <  -0.70 ||get_ahrs_handler_point()->ahrs_data.angle.pitch > 0.70 || \
                    get_ahrs_handler_point()->ahrs_data.angle.roll > 0.90 ||  get_ahrs_handler_point()->ahrs_data.angle.roll < -0.80){
                quadrotor_handler.lock_status = LOCK;
                continue;
            }
#endif

            quadrotor_handler.pModeFun(&quadrotor_handler.quadrotor_status,&(get_ahrs_handler_point()->ahrs_data));
            give_pwm();
        }

        rt_thread_delay(5);
    }
}


rt_err_t control_init(void)
{
    quadrotor_handler.motor.outputPwm = (struct rt_device_pwm *)rt_device_find(MOTOR_PWM_DEV);
    quadrotor_handler.lock_status = LOCK;
    quadrotor_handler.flying_mode = GYRO_LOOP_MODE;

    //unlock_motor();
    PID_init(&quadrotor_handler.ctl_pid.pitch_pid);
    PID_init(&quadrotor_handler.ctl_pid.roll_pid);
    PID_init(&quadrotor_handler.ctl_pid.yaw_pid);
    PID_init(&quadrotor_handler.ctl_pid.gyro_x_pid);
    PID_init(&quadrotor_handler.ctl_pid.gyro_y_pid);
    PID_init(&quadrotor_handler.ctl_pid.height_pid);

    PID_Set(&quadrotor_handler.ctl_pid.gyro_x_pid, 50, 0, 0);
    PID_Set(&quadrotor_handler.ctl_pid.gyro_y_pid, 50, 0, 0);
    PID_Set(&quadrotor_handler.ctl_pid.pitch_pid, 0, 0, 0);
    PID_Set(&quadrotor_handler.ctl_pid.yaw_pid, 0, 0, 0);
    PID_Set(&quadrotor_handler.ctl_pid.roll_pid, 1.9, 0, 3);
    PID_Set(&quadrotor_handler.ctl_pid.height_pid, 0, 0, 0);

    for(int i = 1;i <= 4;++i)
    {
        rt_pwm_set(quadrotor_handler.motor.outputPwm, i, PWM_PERIOD, 15*PWM_K);
        rt_pwm_enable(quadrotor_handler.motor.outputPwm, i);
    }


    control_task_handler = rt_thread_create("control task", control_task, RT_NULL, 1024, 7, 10);
    rt_thread_startup(control_task_handler);

    rt_kprintf("Startup control task\r\n");


    return RT_EOK;
}
MSH_CMD_EXPORT(control_init,startup control task thread);

static void switch_mode(void)
{
    switch(quadrotor_handler.flying_mode)
    {
    case BLUETOOTH_MOVING:
        quadrotor_handler.pModeFun = bluetooth_control_mode;
        break;
    case BLUETOOTH_DEBUG:
        quadrotor_handler.pModeFun = bluetooth_debug_mode;
        break;
    case GYRO_LOOP_MODE:
        quadrotor_handler.pModeFun = gyro_loopback;
        break;
    case RC_MOVING:
        break;
    case ANGLE_LOOP_MODE:
        quadrotor_handler.pModeFun = angle_loopback;
        break;

    }
}



static void bluetooth_debug_mode(const ahrs_t* purpose_status,const ahrs_t * current_status)
{
    sendPackage_t * tempPackage = get_package_point();

    rt_pwm_set(quadrotor_handler.motor.outputPwm, tempPackage->motor_no, PWM_PERIOD,tempPackage->duty*PWM_K);

}

static void bluetooth_control_mode( ahrs_t* purpose_status,const ahrs_t * current_status)
{

//    bluetooth_control_mode_select_pid();        //更新pid
//    purpose_status->angle.pitch = PITCH_ZERO;
//
//    quadrotor_handler.motor.output[LEFT_FRONT_MOTOR_NUM] = BASIC_DYN +  LEFT_FRONT_MOTOR_PWM_MIN - PstPID(current_status->angle.pitch, purpose_status->angle.pitch ,&quadrotor_handler.ctl_pid.pitch_pid)
//            + PstPID(current_status->angle.roll,  purpose_status->angle.roll, &quadrotor_handler.ctl_pid.roll_pid);
//    quadrotor_handler.motor.output[LEFT_BACK_MOTOR_NUM] = BASIC_DYN +  LEFT_BACK_MOTOR_PWM_MIN + PstPID(current_status->angle.pitch, purpose_status->angle.pitch ,&quadrotor_handler.ctl_pid.pitch_pid)
//         + PstPID(current_status->angle.roll,  purpose_status->angle.roll, &quadrotor_handler.ctl_pid.roll_pid);
//    quadrotor_handler.motor.output[RIGHT_FRONT_MOTOR_NUM] = BASIC_DYN +  RIGHT_FRONT_MOTOR_PWM_MIN - PstPID(current_status->angle.pitch, purpose_status->angle.pitch ,&quadrotor_handler.ctl_pid.pitch_pid)
//            - PstPID(current_status->angle.roll,  purpose_status->angle.roll, &quadrotor_handler.ctl_pid.roll_pid);
//    quadrotor_handler.motor.output[RIGHT_BACK_MOTOR_NUM] = BASIC_DYN + RIGHT_BACK_MOTOR_PWM_MIN + PstPID(current_status->angle.pitch, purpose_status->angle.pitch ,&quadrotor_handler.ctl_pid.pitch_pid)
//          - PstPID(current_status->angle.roll,  purpose_status->angle.roll, &quadrotor_handler.ctl_pid.roll_pid);


}
void bluetooth_control_mode_select_pid()
{
    sendPackage_t * tempPackage = get_package_point();

    switch(tempPackage->select)
    {
//    case 1:
//        PID_Set(&quadrotor_handler.ctl_pid.gyro_y_pid, tempPackage->kp, tempPackage->ki, tempPackage->kd);
//        break;
//    case 2:
//        PID_Set(&quadrotor_handler.ctl_pid.gyro_x_pid, tempPackage->kp, tempPackage->ki, tempPackage->kd);
//        break;
    case 1:
        PID_Set(&quadrotor_handler.ctl_pid.roll_pid, tempPackage->kp, tempPackage->ki, tempPackage->kd);
        break;
    case 2:
        PID_Set(&quadrotor_handler.ctl_pid.pitch_pid, tempPackage->kp, tempPackage->ki, tempPackage->kd);
        break;
    default:
        break;


    }
    //tempPackage->select = 0;
}

/*
 * 角速度控制模式
 *
 * */
static void gyro_loopback(ahrs_t* purpose_status,const ahrs_t * current_status)
{
    purpose_status->gyro.x = 0.0f;

    LEFT_FRONT_MOTOR = LEFT_FRONT_MOTOR_PWM_MIN +  (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,0) + STARTUP_PWM;
    LEFT_BACK_MOTOR = LEFT_BACK_MOTOR_PWM_MIN + (int)PstPID(current_status->gyro.x , purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,0) + STARTUP_PWM;
    RIGHT_FRONT_MOTOR = RIGHT_FRONT_MOTOR_PWM_MIN - (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,0) + STARTUP_PWM;
    RIGHT_BACK_MOTOR = RIGHT_BACK_MOTOR_PWM_MIN - (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,0) + STARTUP_PWM;

}

/*
 * 角度控制模式
 *
 * */
static void angle_loopback(ahrs_t* purpose_status,const ahrs_t * current_status)
{
    purpose_status->angle.roll = 0.03f;
    purpose_status->gyro.x = PstPID(current_status->angle.roll, purpose_status->angle.roll, &quadrotor_handler.ctl_pid.roll_pid,1);


    LEFT_FRONT_MOTOR = LEFT_FRONT_MOTOR_PWM_MIN +  (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,1) + STARTUP_PWM;
    LEFT_BACK_MOTOR = LEFT_BACK_MOTOR_PWM_MIN + (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,1) + STARTUP_PWM;
    RIGHT_FRONT_MOTOR = RIGHT_FRONT_MOTOR_PWM_MIN - (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,1) + STARTUP_PWM;
    RIGHT_BACK_MOTOR = RIGHT_BACK_MOTOR_PWM_MIN - (int)PstPID(current_status->gyro.x, purpose_status->gyro.x, &quadrotor_handler.ctl_pid.gyro_x_pid,1) + STARTUP_PWM;

}




static void give_pwm(void)
{
    deadband(&LEFT_FRONT_MOTOR,LEFT_FRONT_MOTOR_PWM_MIN,180);
    deadband(&LEFT_BACK_MOTOR,LEFT_BACK_MOTOR_PWM_MIN,180);
    deadband(&RIGHT_FRONT_MOTOR,RIGHT_FRONT_MOTOR_PWM_MIN,180);
    deadband(&RIGHT_BACK_MOTOR,RIGHT_BACK_MOTOR_PWM_MIN,180);

    rt_pwm_set(quadrotor_handler.motor.outputPwm, LEFT_FRONT_MOTOR_NUM + 1, PWM_PERIOD, LEFT_FRONT_MOTOR*PWM_K);
    rt_pwm_set(quadrotor_handler.motor.outputPwm, LEFT_BACK_MOTOR_NUM + 1, PWM_PERIOD, LEFT_BACK_MOTOR*PWM_K);
    rt_pwm_set(quadrotor_handler.motor.outputPwm, RIGHT_FRONT_MOTOR_NUM + 1, PWM_PERIOD,RIGHT_FRONT_MOTOR*PWM_K);
    rt_pwm_set(quadrotor_handler.motor.outputPwm, RIGHT_BACK_MOTOR_NUM + 1, PWM_PERIOD, RIGHT_BACK_MOTOR*PWM_K);

}
