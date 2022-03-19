#ifndef __SERVICECHASSIS_TASK_H
#define __SERVICECHASSIS_TASK_H

#include "pid.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "can.h"
#include "usart.h"
#include <stdbool.h>
#include "stdio.h"
#include "string.h"
#include "OLED.h"
#include "stdlib.h"

#define MY_ADDR 0x01			//下位机地址
#define HOST_ADDR 0x00			//主机地址
#define RECEIVE_BUFFER_MAX 100 //数据长度最多255字节

//定义函数指针类型
typedef void(*HandleFunctionPtr)(uint8_t cmd,uint8_t data_len,uint8_t *data);


void ServiceChassis_task(void const *pvParameters);

void set_speed_handler(uint8_t cmd,uint8_t data_len,uint8_t *data);
void request_speed_handler(uint8_t cmd,uint8_t data_len,uint8_t *data);
void request_addr_handler(uint8_t cmd,uint8_t data_len,uint8_t *data);
void request_imu_handler(uint8_t cmd,uint8_t data_len,uint8_t *data);

void USART6_IRQHandler(void);

/**
 * @brief 数据发送函数
 * 
 * @param cmd 指令
 * @param data_len 数据长度 
 * @param data 数据
 */
void transmit_dataframe(uint8_t cmd, uint8_t data_len, uint8_t *data);
/**
 * @brief 串口中断接收函数
 * 
 * @param b 
 */
void serial_interrupt(uint8_t b);

/**
 * @brief 串口接收数据判断函数
 * 
 */

void receive_task_loop(void);
/**
 * 传送一个简单的响应，将会自动封装一个简单的响应数据包
 * @param cmd 请求指令
 */
void transmit_simple_ack(uint8_t cmd);

void service_chassis_move(void);
/**
 * 求和函数
 */
uint8_t sum_str(uint8_t *begin, uint8_t *end);
/**
 * @brief 清空数组函数
 * 
 * @param str 目标数组
 * @param size 数组大小
 */
void clear_str(uint8_t *str,uint8_t size);
/**
 * @brief 反转字节的16进制
 * 
 * @param byte 
 * @return uint8_t 
 */
uint8_t reverse_byte(uint8_t byte);
#endif

