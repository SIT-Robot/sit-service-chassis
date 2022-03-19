/**
 * @file ServiceChassis_task.c
 * @author 罗志阳 (3181804071@qq.com)
 * @brief 服务底盘
 * @version 0.1
 * @date 2021-03-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ServiceChassis_task.h"

int16_t set_spd[3] = {0};                         //设置速度
pid_type_def pid_spd[3];                          //PID结构体数组
const fp32 pid_parameter[3] = {1.5f, 0.1f, 2.0f}; //PID系数 {1.5f, 0.1f, 0.0f}
extern motor_measure_t motor_chassis[7];          //大疆底盘电机数据接收存储结构体
uint8_t hasTask = 0;                             //是否存在可执行的任务标志
uint8_t receive[RECEIVE_BUFFER_MAX];              //数据接收数组

HandleFunctionPtr handler_ptrs[0xff] = {
    NULL,                  //0x00
    request_addr_handler,  //0x01
    request_speed_handler, //0x02
    set_speed_handler,     //0x03
    request_imu_handler,   //0x04
};

/**
 * @brief 服务底盘FreeRTOS子任务
 * 
 * @param pvParameters 
 */
void ServiceChassis_task(void const *pvParameters)
{
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    for (uint8_t i = 0; i < 3; i++)
    {
        PID_init(&pid_spd[i], PID_POSITION, pid_parameter, 16384, 2000); //PID初始化
    }
    while (1)
    {
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); //接收电机回传数据
        receive_task_loop();                       //接收数据判断
        service_chassis_move();
    }
}

/**
 * @brief 数据发送函数
 * 
 * @param cmd 指令
 * @param data_len 数据长度 
 * @param data 数据
 */
void transmit_dataframe(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t buf_ptr = 0;
    uint8_t transmit_buffer[100];
    //包头
    transmit_buffer[buf_ptr++] = 0x55;
    transmit_buffer[buf_ptr++] = 0xaa;

    //地址
    transmit_buffer[buf_ptr++] = HOST_ADDR;

    //命令
    transmit_buffer[buf_ptr++] = cmd;

    //数据长度
    transmit_buffer[buf_ptr++] = data_len;

    //数据
    for (int i = 0; i < data_len; i++)
    {
        transmit_buffer[buf_ptr++] = data[i];
    }

    //校验和(从地址加到数据尾)
    uint8_t check_sum = sum_str(
        transmit_buffer + 2,
        transmit_buffer + 2 + 3 + data_len);

    transmit_buffer[buf_ptr++] = check_sum;

    transmit_buffer[buf_ptr++] = 0xbb;
    transmit_buffer[buf_ptr++] = 0xcc;

    HAL_UART_Transmit(&huart6, transmit_buffer, buf_ptr, 20);
}
/**
 * 传送一个简单的响应，将会自动封装一个简单的响应数据包
 * @param cmd 请求指令
 */
void transmit_simple_ack(uint8_t cmd)
{
    transmit_dataframe(reverse_byte(cmd), 0, NULL);
}
/**
 * @brief 上位机请求地址处理函数
 * 
 * @param cmd 
 * @param data_len 
 * @param data 
 */
void request_addr_handler(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t my_address = MY_ADDR;
    transmit_dataframe(reverse_byte(cmd), 1, &my_address);
}
/**
 * @brief 上位机请求速度处理函数
 * 
 * @param cmd 
 * @param data_len 
 * @param data 
 */
void request_speed_handler(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t tx_data[10];
    for (uint8_t i = 0; i < 3; i++)
    {
        tx_data[2 * i] = motor_chassis[i].speed_rpm & 0xff;
        tx_data[2 * i + 1] = motor_chassis[i].speed_rpm >> 8 & 0xff;
    }
    transmit_dataframe(reverse_byte(cmd), 6, tx_data);
}
/**
 * @brief Set the speed handler object
 * 
 * @param cmd 
 * @param data_len 
 * @param data 
 */
void set_speed_handler(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    for (uint8_t i = 0; i < 3; i++) //将速度放入即将对电机设置的速度数组
    {
        set_spd[i] = (int16_t)(data[2 * i] << 8 | data[2 * i + 1]);
    }
    transmit_simple_ack(cmd);
}
/**
 * @brief 回应上位机请求imu数据
 * 
 * @param cmd 
 * @param data_len 
 * @param data 
 */
void request_imu_handler(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t imu_data[60];
    memcpy(imu_data + 4, get_INS_angle_point(), 12);   //欧拉角数据y,p,r
    memcpy(imu_data + 16, get_INS_quat_point(), 16);   //四元数数据x,y,z,w
    memcpy(imu_data + 32, get_gyro_data_point(), 12);  //角速度数据x,y,z
    memcpy(imu_data + 44, get_accel_data_point(), 12); //线加速度数据x,y,z
    transmit_dataframe(reverse_byte(cmd), 52, imu_data);
}

/**
 * @brief 服务底盘运动函数
 * 
 */
void service_chassis_move()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        PID_calc(&pid_spd[i], motor_chassis[i].speed_rpm, set_spd[i]); //PID调节
    }
    CAN_cmd_chassis(pid_spd[0].out, pid_spd[1].out, pid_spd[2].out, 0); //向电机发送速度
    osDelay(2);
}

/**
 * @brief 串口接收数据判断函数
 * 
 */

void receive_task_loop()
{
    if (hasTask == 1)
    {
        uint8_t cmd = receive[3];
        uint8_t data_len = receive[4];
        uint8_t *data = &receive[5];
        //查找任务指针
        if (cmd < sizeof(handler_ptrs) / sizeof(HandleFunctionPtr))
        { //防止越界
            HandleFunctionPtr handle_ptr = handler_ptrs[cmd];
            if (handle_ptr != NULL) //非空检查
            {
                handle_ptr(cmd, data_len, data);
            }
        }
        hasTask = 0; //完成任务
    }
}

/**
 * @brief 串口中断接收函数
 * 
 * @param b 
 */
void serial_interrupt(uint8_t b)
{
    static uint8_t rxcount = 0;

    if (hasTask == 1)
    {
        // 如果任务未处理完毕，屏蔽这个中断
        return;
    }
    receive[rxcount++] = b;
    if (receive[0] == 0x55)
    {
        if (rxcount >= 8)
        {
            if (receive[1] == 0xaa &&                            //二次校验包头
                (receive[2] == MY_ADDR || receive[2] == 0xFF) && //检查地址，只有本机地址与0xFF广播地址允许接收
                receive[rxcount - 1] == 0xcc &&                  //校验包尾
                receive[rxcount - 2] == 0xbb)
            {
                //包头包尾校验成功，检查校验和
                if (sum_str(&receive[2],
                            &receive[rxcount - 3]) == receive[rxcount - 3])
                {
                    rxcount = 0;
                    hasTask = 1;
                }
                else
                {
                    //发生错误
                    rxcount = 0;
                }
            }
            else
            {
                //发生错误
                rxcount = 0;
            }
        }
    }
    else
    {
        //发生错误
        rxcount = 0;
    }
    rxcount = rxcount > RECEIVE_BUFFER_MAX ? 0 : rxcount;
}

void USART6_IRQHandler(void)
{
    if (huart6.Instance->SR & UART_FLAG_RXNE)
    {
        uint8_t d = huart6.Instance->DR;
        serial_interrupt(d);
    }
}
/**
 * @brief 反转字节的16进制
 * 
 * @param byte 
 * @return uint8_t 
 */
uint8_t reverse_byte(uint8_t byte)
{
    return ((byte & 0x0f) << 4 | (byte >> 4));
}

/**
 * 求和函数
 */
uint8_t sum_str(uint8_t *begin, uint8_t *end)
{
    uint8_t sum = 0;
    for (uint8_t *it = begin; it != end; it++)
    {
        sum += *it;
    }
    return sum;
}

/**
 * @brief 清空数组函数
 * 
 * @param str 目标数组
 * @param size 数组大小
 */
void clear_str(uint8_t *str, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        str[i] = 0;
    }
}
