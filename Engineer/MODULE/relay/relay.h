#ifndef RELAY_H
#define RELAY_H
#include "stdint.h"
#include "stdlib.h"
#include "gpio.h"//目前没必要封装bsp_gpio
#define RELAY_MAX_NUM 8 //最大允许注册数

#pragma pack(1)
typedef struct {
	
    GPIO_TypeDef *gpio1;
		GPIO_TypeDef *gpio2;
		uint16_t gpio_pin1;
		uint16_t gpio_pin2;
		uint8_t state;
		
} Relay_Instance;

typedef struct
{
    GPIO_TypeDef *gpio1;
		GPIO_TypeDef *gpio2;
		uint16_t gpio_pin1;
		uint16_t gpio_pin2;
		uint8_t state;
} Relay_Init_Config_s;

#pragma pack()

Relay_Instance *RelayInit(Relay_Init_Config_s *relay);

void RelayUp(Relay_Instance *config);
void RelayOff(Relay_Instance *config);
void RelayDown(Relay_Instance *config);

#endif // DAEMON_H
