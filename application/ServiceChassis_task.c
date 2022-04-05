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

static const uint8_t crc_table[] =
{
    0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e,
    0x43,0x72,0x21,0x10,0x87,0xb6,0xe5,0xd4,0xfa,0xcb,0x98,0xa9,0x3e,0x0f,0x5c,0x6d,
    0x86,0xb7,0xe4,0xd5,0x42,0x73,0x20,0x11,0x3f,0x0e,0x5d,0x6c,0xfb,0xca,0x99,0xa8,
    0xc5,0xf4,0xa7,0x96,0x01,0x30,0x63,0x52,0x7c,0x4d,0x1e,0x2f,0xb8,0x89,0xda,0xeb,
    0x3d,0x0c,0x5f,0x6e,0xf9,0xc8,0x9b,0xaa,0x84,0xb5,0xe6,0xd7,0x40,0x71,0x22,0x13,
    0x7e,0x4f,0x1c,0x2d,0xba,0x8b,0xd8,0xe9,0xc7,0xf6,0xa5,0x94,0x03,0x32,0x61,0x50,
    0xbb,0x8a,0xd9,0xe8,0x7f,0x4e,0x1d,0x2c,0x02,0x33,0x60,0x51,0xc6,0xf7,0xa4,0x95,
    0xf8,0xc9,0x9a,0xab,0x3c,0x0d,0x5e,0x6f,0x41,0x70,0x23,0x12,0x85,0xb4,0xe7,0xd6,
    0x7a,0x4b,0x18,0x29,0xbe,0x8f,0xdc,0xed,0xc3,0xf2,0xa1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5b,0x6a,0xfd,0xcc,0x9f,0xae,0x80,0xb1,0xe2,0xd3,0x44,0x75,0x26,0x17,
    0xfc,0xcd,0x9e,0xaf,0x38,0x09,0x5a,0x6b,0x45,0x74,0x27,0x16,0x81,0xb0,0xe3,0xd2,
    0xbf,0x8e,0xdd,0xec,0x7b,0x4a,0x19,0x28,0x06,0x37,0x64,0x55,0xc2,0xf3,0xa0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xb2,0xe1,0xd0,0xfe,0xcf,0x9c,0xad,0x3a,0x0b,0x58,0x69,
    0x04,0x35,0x66,0x57,0xc0,0xf1,0xa2,0x93,0xbd,0x8c,0xdf,0xee,0x79,0x48,0x1b,0x2a,
    0xc1,0xf0,0xa3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1a,0x2b,0xbc,0x8d,0xde,0xef,
    0x82,0xb3,0xe0,0xd1,0x46,0x77,0x24,0x15,0x3b,0x0a,0x59,0x68,0xff,0xce,0x9d,0xac
};

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
    uint8_t *ptr = begin;
    uint8_t len = end-begin;
    uint8_t  crc = 0x00;
 
    while (len--)
    {
        crc = crc_table[crc ^ *ptr++];
    }
    return crc;
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
