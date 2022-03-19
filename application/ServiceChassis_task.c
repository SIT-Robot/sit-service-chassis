/**
 * @file ServiceChassis_task.c
 * @author ��־�� (3181804071@qq.com)
 * @brief �������
 * @version 0.1
 * @date 2021-03-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ServiceChassis_task.h"

int16_t set_spd[3] = {0};                         //�����ٶ�
pid_type_def pid_spd[3];                          //PID�ṹ������
const fp32 pid_parameter[3] = {1.5f, 0.1f, 2.0f}; //PIDϵ�� {1.5f, 0.1f, 0.0f}
extern motor_measure_t motor_chassis[7];          //�󽮵��̵�����ݽ��մ洢�ṹ��
uint8_t hasTask = 0;                             //�Ƿ���ڿ�ִ�е������־
uint8_t receive[RECEIVE_BUFFER_MAX];              //���ݽ�������

HandleFunctionPtr handler_ptrs[0xff] = {
    NULL,                  //0x00
    request_addr_handler,  //0x01
    request_speed_handler, //0x02
    set_speed_handler,     //0x03
    request_imu_handler,   //0x04
};

/**
 * @brief �������FreeRTOS������
 * 
 * @param pvParameters 
 */
void ServiceChassis_task(void const *pvParameters)
{
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    for (uint8_t i = 0; i < 3; i++)
    {
        PID_init(&pid_spd[i], PID_POSITION, pid_parameter, 16384, 2000); //PID��ʼ��
    }
    while (1)
    {
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); //���յ���ش�����
        receive_task_loop();                       //���������ж�
        service_chassis_move();
    }
}

/**
 * @brief ���ݷ��ͺ���
 * 
 * @param cmd ָ��
 * @param data_len ���ݳ��� 
 * @param data ����
 */
void transmit_dataframe(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t buf_ptr = 0;
    uint8_t transmit_buffer[100];
    //��ͷ
    transmit_buffer[buf_ptr++] = 0x55;
    transmit_buffer[buf_ptr++] = 0xaa;

    //��ַ
    transmit_buffer[buf_ptr++] = HOST_ADDR;

    //����
    transmit_buffer[buf_ptr++] = cmd;

    //���ݳ���
    transmit_buffer[buf_ptr++] = data_len;

    //����
    for (int i = 0; i < data_len; i++)
    {
        transmit_buffer[buf_ptr++] = data[i];
    }

    //У���(�ӵ�ַ�ӵ�����β)
    uint8_t check_sum = sum_str(
        transmit_buffer + 2,
        transmit_buffer + 2 + 3 + data_len);

    transmit_buffer[buf_ptr++] = check_sum;

    transmit_buffer[buf_ptr++] = 0xbb;
    transmit_buffer[buf_ptr++] = 0xcc;

    HAL_UART_Transmit(&huart6, transmit_buffer, buf_ptr, 20);
}
/**
 * ����һ���򵥵���Ӧ�������Զ���װһ���򵥵���Ӧ���ݰ�
 * @param cmd ����ָ��
 */
void transmit_simple_ack(uint8_t cmd)
{
    transmit_dataframe(reverse_byte(cmd), 0, NULL);
}
/**
 * @brief ��λ�������ַ������
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
 * @brief ��λ�������ٶȴ�����
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
    for (uint8_t i = 0; i < 3; i++) //���ٶȷ��뼴���Ե�����õ��ٶ�����
    {
        set_spd[i] = (int16_t)(data[2 * i] << 8 | data[2 * i + 1]);
    }
    transmit_simple_ack(cmd);
}
/**
 * @brief ��Ӧ��λ������imu����
 * 
 * @param cmd 
 * @param data_len 
 * @param data 
 */
void request_imu_handler(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t imu_data[60];
    memcpy(imu_data + 4, get_INS_angle_point(), 12);   //ŷ��������y,p,r
    memcpy(imu_data + 16, get_INS_quat_point(), 16);   //��Ԫ������x,y,z,w
    memcpy(imu_data + 32, get_gyro_data_point(), 12);  //���ٶ�����x,y,z
    memcpy(imu_data + 44, get_accel_data_point(), 12); //�߼��ٶ�����x,y,z
    transmit_dataframe(reverse_byte(cmd), 52, imu_data);
}

/**
 * @brief ��������˶�����
 * 
 */
void service_chassis_move()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        PID_calc(&pid_spd[i], motor_chassis[i].speed_rpm, set_spd[i]); //PID����
    }
    CAN_cmd_chassis(pid_spd[0].out, pid_spd[1].out, pid_spd[2].out, 0); //���������ٶ�
    osDelay(2);
}

/**
 * @brief ���ڽ��������жϺ���
 * 
 */

void receive_task_loop()
{
    if (hasTask == 1)
    {
        uint8_t cmd = receive[3];
        uint8_t data_len = receive[4];
        uint8_t *data = &receive[5];
        //��������ָ��
        if (cmd < sizeof(handler_ptrs) / sizeof(HandleFunctionPtr))
        { //��ֹԽ��
            HandleFunctionPtr handle_ptr = handler_ptrs[cmd];
            if (handle_ptr != NULL) //�ǿռ��
            {
                handle_ptr(cmd, data_len, data);
            }
        }
        hasTask = 0; //�������
    }
}

/**
 * @brief �����жϽ��պ���
 * 
 * @param b 
 */
void serial_interrupt(uint8_t b)
{
    static uint8_t rxcount = 0;

    if (hasTask == 1)
    {
        // �������δ������ϣ���������ж�
        return;
    }
    receive[rxcount++] = b;
    if (receive[0] == 0x55)
    {
        if (rxcount >= 8)
        {
            if (receive[1] == 0xaa &&                            //����У���ͷ
                (receive[2] == MY_ADDR || receive[2] == 0xFF) && //����ַ��ֻ�б�����ַ��0xFF�㲥��ַ�������
                receive[rxcount - 1] == 0xcc &&                  //У���β
                receive[rxcount - 2] == 0xbb)
            {
                //��ͷ��βУ��ɹ������У���
                if (sum_str(&receive[2],
                            &receive[rxcount - 3]) == receive[rxcount - 3])
                {
                    rxcount = 0;
                    hasTask = 1;
                }
                else
                {
                    //��������
                    rxcount = 0;
                }
            }
            else
            {
                //��������
                rxcount = 0;
            }
        }
    }
    else
    {
        //��������
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
 * @brief ��ת�ֽڵ�16����
 * 
 * @param byte 
 * @return uint8_t 
 */
uint8_t reverse_byte(uint8_t byte)
{
    return ((byte & 0x0f) << 4 | (byte >> 4));
}

/**
 * ��ͺ���
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
 * @brief ������麯��
 * 
 * @param str Ŀ������
 * @param size �����С
 */
void clear_str(uint8_t *str, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        str[i] = 0;
    }
}
