

#include <blue_tooth_debug_task.h>
#include <stdio.h>

#define UART_COMRECV "uart2"
#define COM_THREAD_PRIO 10
#define COM_THREAD_TICK 10
#define THREAD_STACK_SIZE 1024
#define COM_BUFFER_SIZE 64

#define ARG_NUM 9

static struct rt_semaphore rx_sem,command_sem;
char buffer[COM_BUFFER_SIZE];
rt_size_t buffer_len = 0;
sendPackage_t package = {0};

BlueTooth_ctr_t bluetooth_handler;


rt_thread_t com_thread = RT_NULL;
rt_thread_t wave_thread = RT_NULL;
static rt_thread_t thread = RT_NULL;

sendPackage_t * get_package_point(void)
{

    return (&package);
}

rt_err_t bluetooth_input_notice(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

void serial_thread_entry(void *parameter)
{
    char recv_ch;
    char *get_msg = "getted instruction\r\n";

    memset((void*)buffer,0,sizeof(buffer));

    while (1)
    {
        while (rt_device_read(bluetooth_handler.send_device, -1, &recv_ch, 1) != 1)
        {
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        if(recv_ch == '\r')
        {
            buffer[buffer_len] = '\0';
            rt_device_write(bluetooth_handler.send_device, 0, get_msg, strlen(get_msg));
            rt_device_write(bluetooth_handler.send_device, 0, buffer, strlen(buffer));
            rt_sem_release(&command_sem);
        }
        else
        {
            rt_enter_critical();
            buffer[buffer_len++] = recv_ch;
            rt_exit_critical();
        }

    }
}

/*
 * 函数作用:用于解析指令
     * 指令格式   name:value
 *
 * */

void command_anys_task(void *command)
{
    /*
         * 指令格式   name:value
     *
     * */
    char command_name[8];
    char command_value[8];
    rt_size_t cpy_point = 0;
    rt_size_t cpy_v_point = 0;

    char * ok_msg = "set value successfully!";
    char * no_msg = "set value failed!";


    memset((void*)command_name,'\0',sizeof(buffer));
    memset((void*)command_value,'\0',sizeof(buffer));

    while(1)
    {
        rt_sem_take(&command_sem,RT_WAITING_FOREVER);

        for(cpy_point = 0;cpy_point < buffer_len;++cpy_point)           //提取变量名
        {
            if(buffer[cpy_point] == ':'){
                cpy_point += 1;
                break;
            }
            command_name[cpy_point] = buffer[cpy_point];
        }

        rt_device_write(bluetooth_handler.send_device, 0, command_name,rt_strlen(command_name));

        for(cpy_v_point = 0;cpy_point < buffer_len;cpy_point++,cpy_v_point++)   // 提取变量值
        {
            command_value[cpy_v_point] = buffer[cpy_point];
        }
        //rt_device_write(bluetooth_handler.send_device, 0, command_value, rt_strlen(command_value));

        if(setv(command_name,command_value) != -1){
            rt_device_write(bluetooth_handler.send_device, 0, ok_msg, rt_strlen(ok_msg));
        }
        else{
            rt_device_write(bluetooth_handler.send_device, 0, no_msg, rt_strlen(no_msg));
        }

        buffer_len = 0;
        cpy_point = 0;
        cpy_v_point = 0;
        memset((void*)buffer,'\0',sizeof(buffer));
        memset((void*)command_name,'\0',sizeof(command_name));
        memset((void*)command_value,'\0',sizeof(command_value));
    }
}

int8_t setv(char *name,char * value)
{
    char *name_list[] = {"kp","ki","kd","mtr","duty","lock","wave","gpid","sprw","mode"};
    rt_size_t name_num = sizeof(name_list)/sizeof(char*);
    int8_t point = -1;
    Quadrotor_t *temp_quadrotor = get_quadrotor_handler_point();

    //char uart2_sendmsg[64] = {0};

    if(package.motor_no > 3)
        package.motor_no = 0;

    for(int i = 0;i < name_num;++i)
    {
        if(!strcmp(name_list[i],name))
        {
            point = i;
            break;
        }
    }

    switch(point)
    {
    case -1:
        break;

    case 0:
        package.kp = atof(value);
        break;

    case 1:
        package.ki = atof(value);
        break;

    case 2:
        package.kd = atof(value);
        break;
    case 8:     //设置pitch yaw roll的pid
        package.select = atoi(value);
        break;

    case 3:
        package.motor_no = atoi(value);
        break;

    case 4:
        package.duty = atoi(value);
        break;
    case 5:
        temp_quadrotor->lock_status = atoi(value);
        break;
    case 6:
        package.startup_send_wave = atoi(value);
        break;
    case 7:
        get_pid();
        break;
    case 9:
        break;
    }

    //package.v_x = 0.5;
    //rt_sprintf(uart2_sendmsg,"Having recv msg:%d\);
    //rt_device_write(bluetooth_handler.send_device, 0, uart2_sendmsg, strlen(uart2_sendmsg));

    return point;
}
/*
 * 函数功能: 波形发送线程，用于发送波形
 *利用mailbox
 *
 * */
void wave_send_thread(void *parameter)
{
    char send_wave[32];
    ahrs_handler_t *ahrs_data = get_ahrs_handler_point();
    Quadrotor_t  * temp_qua = get_quadrotor_handler_point();
    while(1)
    {

        if(package.startup_send_wave == 1)
        {
            rt_sprintf(send_wave,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f\r\n",ahrs_data->ahrs_data.angle.pitch,\
                                                      ahrs_data->ahrs_data.angle.roll,\
                                                      ahrs_data->ahrs_data.gyro.x,\
                                                      ahrs_data->ahrs_data.gyro.y,\
                                                      ahrs_data->ahrs_data.raw_angle.pitch, \
                                                      ahrs_data->ahrs_data.raw_angle.roll, \
                                                      (float)temp_qua->motor.output[0],(float)temp_qua->motor.output[1],(float)temp_qua->motor.output[2],(float)temp_qua->motor.output[3]);
//            rt_sprintf(send_wave,"e:%d,%d,%d,%d \r\n",(int)temp_qua->motor.output[RIGHT_FRONT_MOTOR_NUM],(int)temp_qua->motor.output[RIGHT_BACK_MOTOR_NUM],(int)(temp_qua->motor.output[LEFT_FRONT_MOTOR_NUM])
//                    ,(int)temp_qua->motor.output[LEFT_BACK_MOTOR_NUM]);
            rt_device_write(bluetooth_handler.send_device, 0, send_wave, strlen(send_wave));
            //rt_thread_delay(1);
        }
    }
}

void wave_send_5times(void)
{
    ahrs_handler_t *ahrs_data = get_ahrs_handler_point();

    char send_wave[32];

    for(int i = 0;i < 5;++i){
        rt_sprintf(send_wave,"e:%d,%d,%d,%d \r\n",(int)ahrs_data->ahrs_data.angle.pitch,
                (int)ahrs_data->ahrs_data.angle.roll,(int)ahrs_data->ahrs_data.gyro.z,(int)(ahrs_data->ahrs_data.acc.zero_data*10));
        rt_device_write(bluetooth_handler.send_device, 0, send_wave, strlen(send_wave));
    }
}
MSH_CMD_EXPORT(wave_send_5times,SEND 5 TIMES WAVE)

void broc_send_func(void*parm)
{
    ahrs_handler_t *ahrs_data = get_ahrs_handler_point();
    char sendStr[32];
    int32_t temp;
    while(1){
        temp = (int)(ahrs_data->ahrs_data.height*100);
        rt_sprintf(sendStr,"h:%d,%d,%d.%d \r\n",(int)(ahrs_data->ahrs_data.acc.z*10),(int)(ahrs_data->ahrs_data.acc.x*10),temp / 100,temp % 100);
        rt_device_write(bluetooth_handler.send_device, 0, sendStr, strlen(sendStr));

        rt_thread_mdelay(50);
    }

}
MSH_CMD_EXPORT(broc_send_func,send bluetooth broc)

static void get_pid(void)
{
    Quadrotor_t *temp_qua = get_quadrotor_handler_point();
    char sendStr[32];

//    sprintf(sendStr,"\r\n pitch:Kp:%d,Kd:%d,Ki:%d\r\n roll:%d,%d,%d \r\n yaw:%d,%d,%d\r\n",
//            (int)temp_qua->ctl_pid.pitch_pid.Kp,(int)temp_qua->ctl_pid.pitch_pid.Kd,(int)temp_qua->ctl_pid.pitch_pid.Ki
//          ,(int)temp_qua->ctl_pid.roll_pid.Kp,(int)temp_qua->ctl_pid.roll_pid.Kd,(int)temp_qua->ctl_pid.roll_pid.Ki,
//          (int)temp_qua->ctl_pid.yaw_pid.Kp,(int)temp_qua->ctl_pid.yaw_pid.Kd,(int)temp_qua->ctl_pid.yaw_pid.Ki);
    rt_device_write(bluetooth_handler.send_device, 0, sendStr, strlen(sendStr));
}

/*
 * 串口初始化
 * 启动串口指令处理线程
 *
 * */
rt_err_t bluetooth_init(void)
{
    bluetooth_handler.send_device = rt_device_find(UART_COMRECV);
    char *send_msg = "Welcoming to use uart-debug";
    if (!bluetooth_handler.send_device)
    {
        rt_kprintf("find %s failed!\n", UART_COMRECV);
        return RT_ERROR;
    }

    memset(&package,0,sizeof(package));

    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&command_sem,"command_sem",0,RT_IPC_FLAG_FIFO);     //指令处理信号量,处理线程等待指令接收完成

    rt_device_open(bluetooth_handler.send_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(bluetooth_handler.send_device, bluetooth_input_notice);
    rt_device_write(bluetooth_handler.send_device, 0, send_msg, strlen(send_msg));

    thread = rt_thread_create("bluetooth debug tooth", serial_thread_entry, RT_NULL, 1024, COM_THREAD_PRIO, COM_THREAD_TICK);
    if (thread != RT_NULL)
    {
        rt_kprintf("Create uart-recv thread sucessfully!\r\n");
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("Create uart-recv thread failed!\r\n");
        return RT_ERROR;
    }


    /*命令解析函数优先级比接收优先级低一点*/
    com_thread = rt_thread_create("bluetooth command analyze", command_anys_task, RT_NULL, 1024, COM_THREAD_PRIO+1, COM_THREAD_TICK);
    if (com_thread != RT_NULL)
    {
        rt_kprintf("Create com_thread thread sucessfully!\r\n");
        rt_thread_startup(com_thread);
    }
    else
    {
        rt_kprintf("Create com_thread  failed!\r\n");
        return RT_ERROR;
    }
//
    wave_thread = rt_thread_create("wave send thread", wave_send_thread, RT_NULL, 1024, COM_THREAD_PRIO+1, COM_THREAD_TICK);
    if (wave_thread != RT_NULL)
    {
        rt_kprintf("Create wave_thread thread sucessfully!\r\n");
        rt_thread_startup(wave_thread);
    }
    else
    {
        rt_kprintf("Create wave_thread  failed!\r\n");
        return RT_ERROR;
    }

    rt_kprintf("UART Handle thread startup\r\n");
    return RT_EOK;

   // broc_send_func();
}
MSH_CMD_EXPORT(bluetooth_init,command recv uart init);
