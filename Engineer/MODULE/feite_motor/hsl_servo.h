/*
 * HLS_SCS.h
 * 整合版：飞特HLS系列串行舵机协议、应用层及STM32底层接口
 */

#ifndef _HSL_SERVO_H
#define _HSL_SERVO_H

#include <stdint.h>
#include "main.h" // 必须包含以识别 huart2 和 HAL 库函数

// ------- 指令定义 -------
#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REG_WRITE  0x04
#define INST_REG_ACTION 0x05
#define INST_RESET      0x06
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_READ  0x82

// ------- 错误代码定义 -------
enum SCS_ERR_LIST {
    SCS_ERR_NO_REPLY = 1,
    SCS_ERR_CRC_CMP  = 2,
    SCS_ERR_SLAVE_ID = 3,
    SCS_ERR_BUFF_LEN = 4,
};

// ------- HLS 内存表常用地址 -------
#define HLS_ID                  5
#define HLS_ACC                 41
#define HLS_GOAL_POSITION_L     42
#define HLS_PRESENT_POSITION_L  56
#define HLS_PRESENT_SPEED_L     58

// ------- 协议层与应用层 API -------
void setEnd(uint8_t _End);
void setLevel(uint8_t _Level);
int  getLastError(void);

// 写操作
int  genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
int  writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);
int  WritePosEx2(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque);
void SyncWritePosEx2(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[], uint16_t Torque[]);

// 读操作
int  Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);
int  readByte(uint8_t ID, uint8_t MemAddr);
int  readWord(uint8_t ID, uint8_t MemAddr);
int  Ping(uint8_t ID);

// 硬件底层函数声明 (内部使用)
void rFlushSCS(void);
void wFlushSCS(void);

#endif