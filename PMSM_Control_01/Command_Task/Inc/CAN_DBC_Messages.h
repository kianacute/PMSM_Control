/*
 * MCU实测版CAN报文头文件 - 请勿修改
 * 适配STM32等MCU的CAN控制器位序规则
 */
#ifndef CAN_DBC_MESSAGES_H
#define CAN_DBC_MESSAGES_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MCU实测版CAN信号解析函数
 * @param data CAN接收寄存器数据（8字节）
 * @param start_bit DBC起始位（bit）
 * @param len 信号长度（bit）
 * @param is_little 1=小端(Intel)，0=大端(Motorola)
 * @param is_signed 1=有符号，0=无符号
 * @return 原始值
 */
int64_t can_get_raw_value(const uint8_t *data, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed);

/**
 * @brief MCU实测版CAN信号编码函数
 * @param data CAN发送寄存器数据（8字节）
 * @param raw_val 原始值
 * @param start_bit DBC起始位（bit）
 * @param len 信号长度（bit）
 * @param is_little 1=小端(Intel)，0=大端(Motorola)
 * @param is_signed 1=有符号，0=无符号
 */
void can_set_raw_value(uint8_t *data, int64_t raw_val, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed);

// 报文：Motor_CMD (ID:0x122, 长度:8字节)
typedef struct {
    float Motor_Run_Req;  // 起始位:7, 长度:1, 缩放:1, 偏移:0
    float Motor_Run_Rpm;  // 起始位:23, 长度:16, 缩放:1, 偏移:0
} CAN_Motor_CMD_t;

void CAN_Parse_Motor_CMD(const uint8_t data[8], CAN_Motor_CMD_t *msg_obj);
void CAN_Pack_Motor_CMD(uint8_t data[8], const CAN_Motor_CMD_t *msg_obj);

// 报文：Motor_para (ID:0x123, 长度:8字节)
typedef struct {
    float Motor_status;  // 起始位:7, 长度:4, 缩放:1, 偏移:0
    float Speed_status;  // 起始位:3, 长度:4, 缩放:1, 偏移:0
    float Bus_Voltage;  // 起始位:15, 长度:8, 缩放:1, 偏移:0
    float Id;  // 起始位:23, 长度:16, 缩放:0.001, 偏移:0
    float Iq;  // 起始位:39, 长度:16, 缩放:0.001, 偏移:0
    float Speed_rpm;  // 起始位:55, 长度:16, 缩放:0.5, 偏移:0
} CAN_Motor_para_t;

void CAN_Parse_Motor_para(const uint8_t data[8], CAN_Motor_para_t *msg_obj);
void CAN_Pack_Motor_para(uint8_t data[8], const CAN_Motor_para_t *msg_obj);

// 报文：Motor_List1 (ID:0x145, 长度:8字节)
typedef struct {
    float Signal_1;  // 起始位:0, 长度:32, 缩放:1, 偏移:0
    float Signal_2;  // 起始位:32, 长度:32, 缩放:1, 偏移:0
} CAN_Motor_List1_t;

void CAN_Parse_Motor_List1(const uint8_t data[8], CAN_Motor_List1_t *msg_obj);
void CAN_Pack_Motor_List1(uint8_t data[8], const CAN_Motor_List1_t *msg_obj);

// 报文：Motor_List2 (ID:0x150, 长度:8字节)
typedef struct {
    float Signal_3;  // 起始位:0, 长度:32, 缩放:1, 偏移:0
    float Signal_4;  // 起始位:32, 长度:32, 缩放:1, 偏移:0
} CAN_Motor_List2_t;

void CAN_Parse_Motor_List2(const uint8_t data[8], CAN_Motor_List2_t *msg_obj);
void CAN_Pack_Motor_List2(uint8_t data[8], const CAN_Motor_List2_t *msg_obj);

#ifdef __cplusplus
}
#endif
#endif
