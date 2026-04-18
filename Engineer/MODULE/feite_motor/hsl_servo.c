/*
 * HLS_SCS.c
 * 整合版实现：包含 STM32 HAL 底层驱动、协议逻辑与应用层指令
 */

#include "hsl_servo.h"
#include <stdlib.h>

// 引用外部串口句柄
extern UART_HandleTypeDef huart1;

// ------- 底层缓冲区与变量 -------
static uint8_t wBuf[128];
static uint8_t wLen = 0;
static uint8_t Level = 1;
static uint8_t End = 0;
static uint8_t u8Status;
static uint8_t u8Error;

// =================================================================
// 1. STM32 硬件底层实现
// =================================================================

// 串口发送封装
static void ftUart_Send(uint8_t *nDat, int nLen) {
    HAL_UART_Transmit(&huart1, nDat, nLen, 100);
}

// 串口接收封装
static int ftUart_Read(uint8_t *nDat, int nLen) {
    if(HAL_OK != HAL_UART_Receive(&huart1, nDat, nLen, 100)) {
        return 0;
    }
    return nLen;
}

// 总线切换延迟 (10us 建议改用微秒级延迟函数)
void ftBus_Delay(void) {
    HAL_Delay(1); // 目前为 1ms，如需精准 10us 请使用定时器或循环
}

// 供协议层调用的接口
int readSCS(unsigned char *nDat, int nLen) { return ftUart_Read(nDat, nLen); }

int writeSCS(unsigned char *nDat, int nLen) {
    while(nLen--) {
        if(wLen < sizeof(wBuf)) wBuf[wLen++] = *nDat++;
    }
    return wLen;
}

void rFlushSCS() { ftBus_Delay(); }

void wFlushSCS() {
    if(wLen) {
        ftUart_Send(wBuf, wLen);
        wLen = 0;
    }
}

// =================================================================
// 2. 协议核心逻辑
// =================================================================

void setEnd(uint8_t _End) { End = _End; }
void setLevel(uint8_t _Level) { Level = _Level; }
int  getLastError(void) { return u8Error; }

// 16位数据转换
static void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data) {
    if(End) { *DataL = (Data>>8); *DataH = (Data&0xff); }
    else    { *DataH = (Data>>8); *DataL = (Data&0xff); }
}

static int SCS2Host(uint8_t DataL, uint8_t DataH) {
    if(End) return (DataL<<8) | DataH;
    else    return (DataH<<8) | DataL;
}

// 指令打包发送
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun) {
    uint8_t msgLen = 2, CheckSum = 0;
    uint8_t bBuf[6];
    bBuf[0] = 0xff; bBuf[1] = 0xff; bBuf[2] = ID; bBuf[4] = Fun;
    if(nDat) {
        msgLen += nLen + 1;
        bBuf[3] = msgLen; bBuf[5] = MemAddr;
        writeSCS(bBuf, 6);
    } else {
        bBuf[3] = msgLen; writeSCS(bBuf, 5);
    }
    CheckSum = ID + msgLen + Fun + MemAddr;
    if(nDat) {
        for(uint8_t i=0; i<nLen; i++) CheckSum += nDat[i];
        writeSCS(nDat, nLen);
    }
    CheckSum = ~CheckSum; writeSCS(&CheckSum, 1);
}

// 帧头检测
int checkHead(void) {
    uint8_t bDat, bBuf[2] = {0, 0}, Cnt = 0;
    while(1) {
        if(!readSCS(&bDat, 1)) return 0;
        bBuf[1] = bBuf[0]; bBuf[0] = bDat;
        if(bBuf[0]==0xff && bBuf[1]==0xff) break;
        if(++Cnt > 10) return 0;
    }
    return 1;
}

// 应答检测
int Ack(uint8_t ID) {
    uint8_t bBuf[4];
    if(ID != 0xfe && Level) {
        if(!checkHead()) { u8Error = SCS_ERR_NO_REPLY; return 0; }
        if(readSCS(bBuf, 4) != 4) { u8Error = SCS_ERR_NO_REPLY; return 0; }
        u8Status = bBuf[2];
    }
    return 1;
}

// =================================================================
// 3. 常用指令与 HLS 应用层实现
// =================================================================

int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen) {
    rFlushSCS(); writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE); wFlushSCS();
    return Ack(ID);
}

int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat) {
    uint8_t buf[2]; Host2SCS(buf+0, buf+1, wDat);
    return genWrite(ID, MemAddr, buf, 2);
}

int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen) {
    uint8_t bBuf[4], calSum = 0;
    rFlushSCS(); writeBuf(ID, MemAddr, &nLen, 1, INST_READ); wFlushSCS();
    if(!checkHead() || readSCS(bBuf, 3) != 3) return 0;
    if(readSCS(nData, nLen) != nLen || readSCS(bBuf+3, 1) != 1) return 0;
    u8Status = bBuf[2];
    return nLen;
}

int readByte(uint8_t ID, uint8_t MemAddr) {
    uint8_t bDat;
    return (Read(ID, MemAddr, &bDat, 1) == 1) ? bDat : -1;
}

int readWord(uint8_t ID, uint8_t MemAddr) {
    uint8_t nDat[2];
    if(Read(ID, MemAddr, nDat, 2) != 2) return -1;
    return SCS2Host(nDat[0], nDat[1]);
}

// 写位置指令 (HLS系列专用)
int WritePosEx2(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque) {
    uint8_t bBuf[7];
    if(Position < 0) { Position = -Position; Position |= (1<<15); }
    bBuf[0] = ACC;
    Host2SCS(bBuf+1, bBuf+2, Position);
    Host2SCS(bBuf+3, bBuf+4, Torque);
    Host2SCS(bBuf+5, bBuf+6, Speed);
    return genWrite(ID, HLS_ACC, bBuf, 7);
}

// 同步写位置
void SyncWritePosEx2(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[], uint16_t Torque[]) {
    uint8_t mesLen = (8 * IDN + 4);
    uint8_t bBuf[7], Sum = 0;
    bBuf[0] = 0xff; bBuf[1] = 0xff; bBuf[2] = 0xfe; bBuf[3] = mesLen;
    bBuf[4] = INST_SYNC_WRITE; bBuf[5] = HLS_ACC; bBuf[6] = 7;
    
    rFlushSCS(); writeSCS(bBuf, 7);
    Sum = 0xfe + mesLen + INST_SYNC_WRITE + HLS_ACC + 7;
    for(uint8_t i=0; i<IDN; i++) {
        int16_t pos = Position[i];
        if(pos < 0) { pos = -pos; pos |= (1<<15); }
        uint8_t pData[7];
        pData[0] = ACC ? ACC[i] : 0;
        Host2SCS(pData+1, pData+2, pos);
        Host2SCS(pData+3, pData+4, Torque[i]);
        Host2SCS(pData+5, pData+6, Speed[i]);
        writeSCS(&ID[i], 1); writeSCS(pData, 7);
        Sum += ID[i];
        for(int j=0; j<7; j++) Sum += pData[j];
    }
    Sum = ~Sum; writeSCS(&Sum, 1); wFlushSCS();
}