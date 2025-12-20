#include "relay.h"
#include "string.h"
/* 用于保存所有的daemon instance */
static Relay_Instance *relay_instances[RELAY_MAX_NUM];
static uint8_t idx; // 用于记录当前注册的daemon数量

Relay_Instance *RelayInit(Relay_Init_Config_s *relay){
	Relay_Instance *config = (Relay_Instance *)malloc(sizeof(Relay_Instance));
	memset(config,0,sizeof(Relay_Instance));
	
	
	config->gpio1 = relay->gpio1;
	config->gpio2 = relay->gpio2;
	config->gpio_pin1 = relay->gpio_pin1;
	config->gpio_pin2 = relay->gpio_pin2;
	config->state = relay->state;
	return config;
}

void RelayUp(Relay_Instance *config){
	config->state=2;
	
	HAL_GPIO_WritePin(config->gpio1,config->gpio_pin1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(config->gpio2,config->gpio_pin2,GPIO_PIN_RESET);
}

void RelayOff(Relay_Instance *config){
	config->state=0;
	HAL_GPIO_WritePin(config->gpio1,config->gpio_pin1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config->gpio2,config->gpio_pin2,GPIO_PIN_RESET);
}

void RelayDown(Relay_Instance *config){
	config->state=1;
	HAL_GPIO_WritePin(config->gpio1,config->gpio_pin1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config->gpio2,config->gpio_pin2,GPIO_PIN_SET);
}