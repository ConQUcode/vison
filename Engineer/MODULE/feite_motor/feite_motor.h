#ifndef __FEITE_MOTOR_H_
#define __FEITE_MOTOR_H_

#include "bsp_usart.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"

#define FTMOTOR_MAX_BUFFSIZE 32  // 最大发送/接收字节数,如果不够可以增加此数值
#define FTMOTOR_OFFSET_BYTES 4


#define FT_MOTOR_CNT 255 // 飞特舵机数量

//MemAddr
#define SMS_STS_ACC 41

//Fun
#define INST_WRITE 0x03


/* 电机启停标志 */
#define FEITE_STOP 0
#define FEITE_ENALBED 1

typedef struct{
	
	uint8_t ID;
	uint8_t MemAddr;
	uint8_t Fun;
	
}Motor_Set_s;

typedef struct{
	
	uint16_t Position;
	uint16_t Speed;
	uint8_t ACC;
	uint8_t stop_flag;
	
}Motor_Ref_s;


#pragma pack(1)
typedef struct{
	USART_Instance *ft_usart_instance;
	Daemon_Instance *ft_daemon;
	
/* 发送部分 */
	uint8_t send_data_len;                                                // 发送数据长度
	uint8_t send_buff_len;                                                // 发送缓冲区长度
	
	uint8_t wLen;                                                         //wait 待处理数据长度
	
	uint8_t raw_send_buff[FTMOTOR_MAX_BUFFSIZE + FTMOTOR_OFFSET_BYTES]; // 发送缓冲区，额外预留4字节
	uint8_t data_send_buff[FTMOTOR_MAX_BUFFSIZE + FTMOTOR_OFFSET_BYTES];
	
	Motor_Set_s motor_set;
	Motor_Ref_s motor_ref;
}FTMotor_instance;
#pragma pack()


typedef struct{
	USART_Init_Config_s usart_init_config;
	
	Motor_Set_s motor_set; 
	Motor_Ref_s motor_ref;
	
}FTMotor_Init_Config_s;

FTMotor_instance *FTMotorInit(FTMotor_Init_Config_s *config);
void FTMotorControl(void);
void WritePosEx(FTMotor_instance *motor);
#endif

