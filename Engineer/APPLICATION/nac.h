#ifndef NAC_H
#define NAC_H

#include "stdint.h"
#include "bsp_usart.h"

#define NAC_RECV_HEADER1 0xAA // ????????

#define NAC_RECV_TAIL    0x55 // ????????

#define NAC_SEND_HEADER 0x98 // ????????
#define NAC_SEND_TAIL   0x34 // ????????

#define VISION_RECV_SIZE   24u // 
#define VISION_SEND_SIZE   12

// #pragma pack(1) // 1????

/* ???????????? */
typedef struct
{
    uint8_t header1;          // ?? 0xAA

	  uint8_t tail;            // ?? 0x55
} Nac_Recv_Init_Config_s;

/* ???????????? */
typedef struct
{
    uint8_t header;        // ?????
    uint8_t tail;          // ?????
} Nac_Send_Init_Config_s;


/* ???????????? */
typedef struct
{
    Nac_Recv_Init_Config_s recv_config; // ???????
    Nac_Send_Init_Config_s send_config; // ???????
    USART_Init_Config_s usart_config;      // ???????
} Nac_Init_Config_s;


typedef struct {
    uint8_t header1;          // ?? 0xAA

    int16_t x;    // 2??
    int16_t y; // 2??
    int16_t z;      // 2??
    int16_t zero;   // 2??
    uint8_t tail;            // ?? 0x55
} Nac_Recv_s;

typedef struct {
    uint8_t header;          // ?? 0xAA
    int16_t motor_lf_rpm;    // 2??
    int16_t motor_rf_rpm;    // 2??
    int16_t motor_lb_rpm;    // 2??
    int16_t motor_rb_rpm;    // 2??
	  float   chassis_yaw;
    uint8_t tail;            // ?? 0x55
} Nac_Send_s;


typedef struct
{
    Nac_Recv_s *recv_data; // 
    Nac_Send_s *send_data; // 
    USART_Instance *usart;    // 
} Nac_Instance;




/**
 * @brief 
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config);


/**
 * @brief 
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config);


/**
 * @brief 
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacInit(UART_HandleTypeDef *Nac_usart_handle);


/**
 * @brief ????
 *
 *
 */
void NacSend();


/**
 * @brief ??????????
 */
void NacSetAltitude(int16_t motor_lf_rpm,int16_t motor_rf_rpm,int16_t motor_lb_rpm,int16_t motor_rb_rpm,float chassis_yaw);



#endif