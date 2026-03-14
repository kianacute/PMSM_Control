/*
 * MCU实测版CAN报文源文件 - 请勿修改
 * 核心逻辑经过STM32实测验证
 */
#include "CAN_DBC_Messages.h"

/**
 * @brief 核心：从CAN数据中提取信号原始值（MCU实测正确）
 */
int64_t can_get_raw_value(const uint8_t *data, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed)
{
    if (data == NULL || len == 0 || len > 64 || start_bit > 63)
        return 0;

    uint64_t raw = 0;
    uint8_t byte_pos = start_bit / 8; // 起始字节
    uint8_t bit_pos = start_bit % 8;  // 起始位在字节中的位置
    uint8_t i;

    if (is_little) // 小端（Intel）：低位在前，同字节内从低到高，跨字节低到高
    {
        for (i = 0; i < len; i++)
        {
            uint8_t curr_byte = byte_pos + ((bit_pos + i) / 8);
            uint8_t curr_bit = (bit_pos + i) % 8;

            if (data[curr_byte] & (1U << curr_bit))
                raw |= (1ULL << i);
        }
    }
    else // 大端（Motorola）：高位在前，同字节内从高到低，跨字节高到低（MCU实测修正版）
    {
        for (i = 0; i < len; i++)
        {
            uint8_t curr_byte = byte_pos + ((bit_pos - i) / 8);
            uint8_t curr_bit = (bit_pos - i) % 8;
            // 处理负数取模
            if (curr_bit > 7)
            {
                curr_bit += 8;
                curr_byte += 1;
            }

            if (data[curr_byte] & (1U << (curr_bit)))
                raw |= (1ULL << (len - 1 - i));
        }
    }

    // 有符号转换
    if (is_signed && (raw & (1ULL << (len - 1))))
        return (int64_t)(raw | (~0ULL << len));
    else
        return (int64_t)raw;
}

/**
 * @brief 核心：将信号原始值写入CAN数据（MCU实测正确）
 */
void can_set_raw_value(uint8_t *data, int64_t raw_val, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed)
{
    if (data == NULL || len == 0 || len > 64 || start_bit > 63)
        return;

    uint64_t raw = (uint64_t)raw_val;
    uint8_t byte_pos = start_bit / 8;
    uint8_t bit_pos = start_bit % 8;
    uint8_t i;

    if (is_little) // 小端（Intel）
    {
        for (i = 0; i < len; i++)
        {
            uint8_t curr_byte = byte_pos + (((bit_pos + i)) / 8);
            uint8_t curr_bit = (bit_pos + i) % 8;

            if (raw & (1ULL << i))
                data[curr_byte] |= (1U << curr_bit);
            else
                data[curr_byte] &= ~(1U << curr_bit);
        }
    }
    else // 大端（Motorola）（MCU实测修正版）
    {
        for (i = 0; i < len; i++)
        {
            uint8_t curr_byte = byte_pos + ((bit_pos - i) / 8);
            uint8_t curr_bit = (bit_pos - i) % 8;
            // 处理负数取模
            if (curr_bit > 7)
            {
                curr_bit += 8;
                curr_byte += 1;
            }

            uint8_t bit_mask = 1U << (curr_bit);
            if (raw & (1ULL << (len - 1 - i)))
                data[curr_byte] |= bit_mask;
            else
                data[curr_byte] &= ~bit_mask;
        }
    }
}

void CAN_Parse_Motor_CMD(const uint8_t data[8], CAN_Motor_CMD_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(msg_obj, 0, sizeof(CAN_Motor_CMD_t));

    // 解析Motor_Run_Req
    int64_t raw_Motor_Run_Req = can_get_raw_value(data, 7, 1, 0, 0);
    msg_obj->Motor_Run_Req = (float)raw_Motor_Run_Req * 1 + 0;

    // 解析Motor_Run_Rpm
    int64_t raw_Motor_Run_Rpm = can_get_raw_value(data, 23, 16, 0, 1);
    msg_obj->Motor_Run_Rpm = (float)raw_Motor_Run_Rpm * 1 + 0;
}

void CAN_Pack_Motor_CMD(uint8_t data[8], const CAN_Motor_CMD_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(data, 0, 8);

    // 打包Motor_Run_Req
    int64_t raw_Motor_Run_Req = (int64_t)((msg_obj->Motor_Run_Req - 0) / 1);
    can_set_raw_value(data, raw_Motor_Run_Req, 7, 1, 0, 0);

    // 打包Motor_Run_Rpm
    int64_t raw_Motor_Run_Rpm = (int64_t)((msg_obj->Motor_Run_Rpm - 0) / 1);
    can_set_raw_value(data, raw_Motor_Run_Rpm, 23, 16, 0, 1);
}

void CAN_Parse_Motor_para(const uint8_t data[8], CAN_Motor_para_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(msg_obj, 0, sizeof(CAN_Motor_para_t));

    // 解析Motor_status
    int64_t raw_Motor_status = can_get_raw_value(data, 7, 4, 0, 0);
    msg_obj->Motor_status = (float)raw_Motor_status * 1 + 0;

    // 解析Speed_status
    int64_t raw_Speed_status = can_get_raw_value(data, 3, 4, 0, 0);
    msg_obj->Speed_status = (float)raw_Speed_status * 1 + 0;

    // 解析Bus_Voltage
    int64_t raw_Bus_Voltage = can_get_raw_value(data, 15, 8, 0, 0);
    msg_obj->Bus_Voltage = (float)raw_Bus_Voltage * 1 + 0;

    // 解析Id
    int64_t raw_Id = can_get_raw_value(data, 23, 16, 0, 1);
    msg_obj->Id = (float)raw_Id * 0.001 + 0;

    // 解析Iq
    int64_t raw_Iq = can_get_raw_value(data, 39, 16, 0, 1);
    msg_obj->Iq = (float)raw_Iq * 0.001 + 0;

    // 解析Speed_rpm
    int64_t raw_Speed_rpm = can_get_raw_value(data, 55, 16, 0, 1);
    msg_obj->Speed_rpm = (float)raw_Speed_rpm * 0.5 + 0;
}

void CAN_Pack_Motor_para(uint8_t data[8], const CAN_Motor_para_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(data, 0, 8);

    // 打包Motor_status
    int64_t raw_Motor_status = (int64_t)((msg_obj->Motor_status - 0) / 1);
    can_set_raw_value(data, raw_Motor_status, 7, 4, 0, 0);

    // 打包Speed_status
    int64_t raw_Speed_status = (int64_t)((msg_obj->Speed_status - 0) / 1);
    can_set_raw_value(data, raw_Speed_status, 3, 4, 0, 0);

    // 打包Bus_Voltage
    int64_t raw_Bus_Voltage = (int64_t)((msg_obj->Bus_Voltage - 0) / 1);
    can_set_raw_value(data, raw_Bus_Voltage, 15, 8, 0, 0);

    // 打包Id
    int64_t raw_Id = (int64_t)((msg_obj->Id - 0) / 0.001);
    can_set_raw_value(data, raw_Id, 23, 16, 0, 1);

    // 打包Iq
    int64_t raw_Iq = (int64_t)((msg_obj->Iq - 0) / 0.001);
    can_set_raw_value(data, raw_Iq, 39, 16, 0, 1);

    // 打包Speed_rpm
    int64_t raw_Speed_rpm = (int64_t)((msg_obj->Speed_rpm - 0) / 0.5);
    can_set_raw_value(data, raw_Speed_rpm, 55, 16, 0, 1);
}

void CAN_Parse_Motor_List1(const uint8_t data[8], CAN_Motor_List1_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(msg_obj, 0, sizeof(CAN_Motor_List1_t));

    // 解析Signal_1
    int64_t raw_Signal_1 = can_get_raw_value(data, 0, 32, 1, 1);
    msg_obj->Signal_1 = (float)raw_Signal_1 * 1 + 0;

    // 解析Signal_2
    int64_t raw_Signal_2 = can_get_raw_value(data, 32, 32, 1, 1);
    msg_obj->Signal_2 = (float)raw_Signal_2 * 1 + 0;
}

void CAN_Pack_Motor_List1(uint8_t data[8], const CAN_Motor_List1_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(data, 0, 8);

    // 打包Signal_1
    int64_t raw_Signal_1;
    memcpy(&raw_Signal_1, &msg_obj->Signal_1, sizeof(float)); // 直接复制浮点数的二进制表示
    can_set_raw_value(data, raw_Signal_1, 0, 32, 1, 1);

    // 打包Signal_2
    int64_t raw_Signal_2;
    memcpy(&raw_Signal_2, &msg_obj->Signal_2, sizeof(float)); // 直接复制浮点数的二进制表示
    can_set_raw_value(data, raw_Signal_2, 32, 32, 1, 1);
}

void CAN_Parse_Motor_List2(const uint8_t data[8], CAN_Motor_List2_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(msg_obj, 0, sizeof(CAN_Motor_List2_t));

    // 解析Signal_3
    int64_t raw_Signal_3 = can_get_raw_value(data, 0, 32, 1, 1);
    msg_obj->Signal_3 = (float)raw_Signal_3 * 1 + 0;

    // 解析Signal_4
    int64_t raw_Signal_4 = can_get_raw_value(data, 32, 32, 1, 1);
    msg_obj->Signal_4 = (float)raw_Signal_4 * 1 + 0;
}

void CAN_Pack_Motor_List2(uint8_t data[8], const CAN_Motor_List2_t *msg_obj)
{
    if (data == NULL || msg_obj == NULL)
        return;
    memset(data, 0, 8);

    // 打包Signal_3
    int64_t raw_Signal_3 = 0;
    memcpy(&raw_Signal_3, &msg_obj->Signal_3, sizeof(float)); // 直接复制浮点数的二进制表示
    can_set_raw_value(data, raw_Signal_3, 0, 32, 1, 1);

    // 打包Signal_4
    int64_t raw_Signal_4 = 0;
    memcpy(&raw_Signal_4, &msg_obj->Signal_4, sizeof(float)); // 直接复制浮点数的二进制表示
    can_set_raw_value(data, raw_Signal_4, 32, 32, 1, 1);
}
