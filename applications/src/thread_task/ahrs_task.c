#include "includes.h"
#include "../inc/filter_lib.h"

#include "inv_mpu_dmp_motion_driver.h"

#define D_TO_RAD(D)  ((D) / 180 * PI)    \ //度数变弧度



ahrs_t zero_data = {0};   /*零漂移数据*/
ahrs_handler_t *ahrs_handler = NULL;
struct mpu6xxx_device* i2c_bus = RT_NULL;

static rt_thread_t barometer_calc_task_handler = RT_NULL;
static rt_thread_t mpu_calc_task_handler = RT_NULL;

static void mpu_calc_task(void *parm); /*MPU解算任务*/
static void barometer_calc_task(void *parm); /*气压计解算任务*/

static float accel_fliter_x[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_y[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_z[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


ahrs_handler_t * get_ahrs_handler_point(void)
{
    return ahrs_handler;
}



ahrs_handler_t * ahrs_init(void)
{

    ahrs_handler = (ahrs_handler_t *)rt_malloc(sizeof(ahrs_handler_t));

    if(ahrs_handler == RT_NULL){
        rt_kprintf("ahrs handler malloc error!\r\n");
        return RT_NULL;
    }

    ahrs_handler->mpu = mpu6xxx_init("i2c1", RT_NULL);
    i2c_bus = ahrs_handler->mpu ;
    memset(&zero_data,0,sizeof(zero_data));

    if(i2c_bus == RT_NULL || ahrs_handler->mpu == RT_NULL)
    {
        rt_pin_write(21, PIN_LOW);
        rt_free(ahrs_handler);
        ahrs_handler = RT_NULL;
        rt_kprintf("MPU FIND ERROR!");
        return RT_NULL;
    }

    rt_pin_mode(21,PIN_LOW);

    mpu6xxx_set_param(ahrs_handler->mpu, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_2000DPS);  //陀螺仪范围配置
    mpu6xxx_set_param(ahrs_handler->mpu, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);     //加速度计，一般设置为±2G
    mpu6xxx_set_param(ahrs_handler->mpu, MPU6XXX_SAMPLE_RATE, 50);                       //采样频率
    mpu6xxx_set_param(ahrs_handler->mpu, MPU6XXX_DLPF_CONFIG, 25);                       //数字低通滤波器设置，一般为1/2采样率


//   ahrs_handler->bmp.cfg.intf.dev_name = "i2c1";
//   ahrs_handler->bmp.cfg.intf.user_data = (void *)BMP280_ADDR_DEFAULT;
//
//   rt_hw_bmp280_init("bmp280", &ahrs_handler->bmp.cfg);

    mpu_calc_task_handler = rt_thread_create("mpu_calc_task",mpu_calc_task,NULL,1024,5,10);
    if(mpu_calc_task_handler == RT_NULL)
    {
        rt_free(ahrs_handler);
        ahrs_handler = RT_NULL;
        rt_kprintf("MPU CALC TASK CREATE ERROR!");
        return RT_NULL;
    }
    rt_thread_startup(mpu_calc_task_handler);



    return ahrs_handler;
}




/*
 * @brief 陀螺仪计算任务
 *
 * */
void mpu_calc_task(void *parm)
{

    float  origin_pitch = 0.0f,origin_roll = 0.0f,origin_yaw = 0.0f; //根据加速度结算出来的角度
    struct mpu6xxx_3axes last_gyro = {0};
    struct mpu6xxx_3axes last_acc = {0};

    struct mpu6xxx_3axes temp_gyro;
    struct mpu6xxx_3axes temp_acc;
    float acc_filter[3][5] = {0}; //平滑滤波参数  x y z

    Kalman_argument_t roll_kalman_argument,pitch_kalman_argument;
    Kalman_argument_init(&roll_kalman_argument);
    Kalman_argument_init(&pitch_kalman_argument);


    while(1)
    {
          mpu6xxx_get_gyro(ahrs_handler->mpu,&temp_gyro);
          mpu6xxx_get_accel(ahrs_handler->mpu, &temp_acc);

          ahrs_handler->ahrs_data.acc.x = (float)(temp_acc.x) * ACC_CONVER;
          ahrs_handler->ahrs_data.acc.y = (float)(temp_acc.y)* ACC_CONVER;
          ahrs_handler->ahrs_data.acc.z = (float)(temp_acc.z)* ACC_CONVER;

          ahrs_handler->ahrs_data.gyro.x = (float)(temp_gyro.x) * GROY_CONVER;
          ahrs_handler->ahrs_data.gyro.y = (float)(temp_gyro.y) * GROY_CONVER;
          ahrs_handler->ahrs_data.gyro.z = (float)(temp_gyro.z ) * GROY_CONVER;

          //对x,y轴进行一阶低通,不必要使用二阶滞后滤波,效果比二阶低通滤波,否则飞行时会产生正弦波
          ahrs_handler->ahrs_data.acc.x = ahrs_handler->ahrs_data.acc.x * (1 - C_FITTER) + last_acc.x * C_FITTER ;
          last_acc.x = ahrs_handler->ahrs_data.acc.x;

          ahrs_handler->ahrs_data.acc.y = ahrs_handler->ahrs_data.acc.y * (1 - C_FITTER) + last_acc.y * C_FITTER ;
          last_acc.y = ahrs_handler->ahrs_data.acc.y;

          accel_fliter_z[2] = accel_fliter_z[1];
          accel_fliter_z[1] = accel_fliter_z[0];
          accel_fliter_z[0] = ahrs_handler->ahrs_data.acc.z = accel_fliter_z[1] * fliter_num[0] \
                  + accel_fliter_z[2] * fliter_num[1] + ahrs_handler->ahrs_data.acc.z * fliter_num[2];      //对z轴进行二阶低通滤波,效果好一点


          //对三轴加速度进行滑动窗口滤波
          acc_filter[0][4] = acc_filter[0][3];
          acc_filter[0][3] = acc_filter[0][2];
          acc_filter[0][2] = acc_filter[0][1];
          acc_filter[0][1] = acc_filter[0][0];
          acc_filter[0][0] = ahrs_handler->ahrs_data.acc.x;

          acc_filter[1][4] = acc_filter[1][3];
          acc_filter[1][3] = acc_filter[1][2];
          acc_filter[1][2] = acc_filter[1][1];
          acc_filter[1][1] = acc_filter[1][0];
          acc_filter[1][0] = ahrs_handler->ahrs_data.acc.y;

          acc_filter[2][4] = acc_filter[2][3];
          acc_filter[2][3] = acc_filter[2][2];
          acc_filter[2][2] = acc_filter[2][1];
          acc_filter[2][1] = acc_filter[2][0];
          acc_filter[2][0] = ahrs_handler->ahrs_data.acc.z;

          ahrs_handler->ahrs_data.acc.x = (acc_filter[0][0] + acc_filter[0][1] + acc_filter[0][2] + acc_filter[0][3] + acc_filter[0][4]) / 5.0f;
          ahrs_handler->ahrs_data.acc.y = (acc_filter[1][0] + acc_filter[1][1] + acc_filter[1][2] + acc_filter[1][3] + acc_filter[1][4]) / 5.0f;
          ahrs_handler->ahrs_data.acc.z = (acc_filter[2][0] + acc_filter[2][1] + acc_filter[2][2] + acc_filter[2][3] + acc_filter[2][4]) / 5.0f;


          //对角速度进行一阶低通滤波,波形很漂亮
          ahrs_handler->ahrs_data.gyro.x = (C_FITTER * last_gyro.x + (1 - C_FITTER) * ahrs_handler->ahrs_data.gyro.x) * PI / 180.0f;
          ahrs_handler->ahrs_data.gyro.y = (C_FITTER * last_gyro.y + (1 - C_FITTER) * ahrs_handler->ahrs_data.gyro.y) * PI / 180.0f;

          last_gyro.x = ahrs_handler->ahrs_data.gyro.x;
          last_gyro.y = ahrs_handler->ahrs_data.gyro.y;



          origin_pitch = -atan2f(ahrs_handler->ahrs_data.acc.x,ahrs_handler->ahrs_data.acc.z);
          origin_roll = atan2f(ahrs_handler->ahrs_data.acc.y,ahrs_handler->ahrs_data.acc.z);
          ahrs_handler->ahrs_data.raw_angle.pitch = origin_pitch;
          ahrs_handler->ahrs_data.raw_angle.roll = origin_roll;
//          origin_yaw = math_yaw + ahrs_handler->ahrs_data.gyro.z * DT;    //存在积分误差


          Kalman_Filter(&ahrs_handler->ahrs_data.angle.roll, ahrs_handler->ahrs_data.gyro.x, origin_roll, &roll_kalman_argument);
          Kalman_Filter(&ahrs_handler->ahrs_data.angle.pitch, ahrs_handler->ahrs_data.gyro.y, origin_pitch, &pitch_kalman_argument);

          rt_thread_mdelay(1);  /*给1000采样*/
          //rt_thread_sleep(rt_tick_from_millisecond(1));

      }

}

void barometer_calc_task(void *parm)/*气压计解算任务*/
{
    ahrs_handler->bmp.baro_handler  = rt_device_find("baro_bmp");
    ahrs_handler->bmp.temp_handler  = rt_device_find("temp_bmp");
    ahrs_handler->bmp.zero_value = 0;
    float temp_acc = 0,temp_v = 0;
    uint32_t temp_data  = 0;



    if(ahrs_handler->bmp.baro_handler == RT_NULL || ahrs_handler->bmp.temp_handler == RT_NULL)
    {
        rt_kprintf("cannot find bmp...");
        return;
    }


    rt_device_open(ahrs_handler->bmp.baro_handler , RT_DEVICE_OFLAG_RDONLY);
    rt_device_open(ahrs_handler->bmp.temp_handler , RT_DEVICE_OFLAG_RDONLY);

    //rt_device_control(ahrs_handler->bmp.baro_handler, RT_SENSOR_CTRL_SET_ODR, (void *)1);
    //rt_device_control(ahrs_handler->bmp.temp_handler, RT_SENSOR_CTRL_SET_ODR, (void *)1);
    for(int i = 0;i < 50;++i){
        rt_device_read(ahrs_handler->bmp.temp_handler, 0, &ahrs_handler->bmp.t_dat, 1);

        rt_device_read(ahrs_handler->bmp.baro_handler, 0, &ahrs_handler->bmp.b_dat, 1);
        ahrs_handler->bmp.zero_value = ahrs_handler->bmp.b_dat.data.baro;
    }

    while(1)
    {


        rt_device_read(ahrs_handler->bmp.temp_handler, 0, &ahrs_handler->bmp.t_dat, 1);
        rt_device_read(ahrs_handler->bmp.baro_handler, 0, &ahrs_handler->bmp.b_dat, 1);
        //temp_data += ahrs_handler->bmp.b_dat.data.baro;

        //ahrs_handler->ahrs_data.height = 44330.00 * (1.00 - pow(((float)ahrs_handler->bmp.b_dat.data.baro)/(float)ahrs_handler->bmp.zero_value,(1.0/5.256)));
        ahrs_handler->ahrs_data.acc.zero_data = temp_acc = ahrs_handler->ahrs_data.acc.z - zero_data.acc.z;
        temp_v += temp_acc;

        ahrs_handler->ahrs_data.height = temp_v;

        //temp_data = 0;

        rt_thread_delay(5);

    }
}
