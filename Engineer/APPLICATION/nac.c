#include "nac.h"
#include "string.h"
#include "robot_def.h"
#include "daemon.h"


static Nac_Instance *nac_instance; // ????????????
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *nac_daemon_instance;





void NacSetAltitude(int16_t motor_lf_rpm,int16_t motor_rf_rpm,int16_t motor_lb_rpm,int16_t motor_rb_rpm,float chassis_yaw)
{
    nac_instance->send_data->motor_lf_rpm       = motor_lf_rpm;
		nac_instance->send_data->motor_rf_rpm       = motor_rf_rpm;
		nac_instance->send_data->motor_lb_rpm       = motor_lb_rpm;
		nac_instance->send_data->motor_rb_rpm       = motor_rb_rpm;
		nac_instance->send_data->chassis_yaw       = chassis_yaw;
}



// ?????
extern uint16_t CRC_INIT;
/**
 * @brief ?????????
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Nac_Recv_s *recv, uint8_t *rx_buff)
{
	recv->x     = (rx_buff[2]<<8)|rx_buff[1];
	recv->y  = (rx_buff[4]<<8)|rx_buff[3];
	recv->z       = (rx_buff[6]<<8)|rx_buff[5];
	recv->zero    = (rx_buff[8]<<8)|rx_buff[7];

}

/**
 * @brief ????,?????????????
 *
 */
static void DecodeNac()
{
    DaemonReload(nac_daemon_instance); // ??


    if (nac_instance->usart->recv_buff[0] == nac_instance->recv_data->header1  &&nac_instance->usart->recv_buff[9] == nac_instance->recv_data->tail) {
        // ??????
        RecvProcess(nac_instance->recv_data, nac_instance->usart->recv_buff);
    }

}


/**
 * @brief ??????,??daemon.c??daemon task??
 * @attention ??HAL??????,????DMA?????????????__HAL_LOCK()?????,????
 *            ??????.??daemon??????,????????????????.
 *
 * @param id vision_usart_instance???,????.
 */
static void NacOfflineCallback(void *id)
{

    USARTServiceInit(nac_instance->usart);

}



/**
 * @brief ????????
 *
 * @param send ?????
 * @param tx_buff ?????
 *
 */
static void SendProcess(Nac_Send_s *send, uint8_t *tx_buff)
{
		tx_buff[0]      = send->header;
	
		tx_buff[1]      = send->motor_lf_rpm>>8;
		tx_buff[2]      = send->motor_lf_rpm&0xFF;
	
		tx_buff[3]      = send->motor_rf_rpm>>8;
		tx_buff[4]      = send->motor_rf_rpm&0xFF;
	
		tx_buff[5]      = send->motor_lb_rpm>>8;
		tx_buff[6]      = send->motor_lb_rpm&0xFF;
	
		tx_buff[7]      = send->motor_rb_rpm>>8;
		tx_buff[8]      = send->motor_rb_rpm&0xFF;
	
		int16_t scaled  = (int16_t)(send->chassis_yaw * 100); // ??????
		
		tx_buff[9]      = scaled >>8;
		tx_buff[10]     = scaled&0xFF;
	
		tx_buff[11]     = send->tail;
}

/**
 * @brief ????
 *
 * @param send ?????
 *
 */
void NacSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(nac_instance->send_data, send_buff);
    USARTSend(nac_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}

/**
 * @brief ???????????????,???????????????
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config)
{
    Nac_Recv_s *recv_data = (Nac_Recv_s *)malloc(sizeof(Nac_Recv_s));
    memset(recv_data, 0, sizeof(Nac_Recv_s));

    recv_data->header1 = recv_config->header1;
	
		recv_data->tail = recv_config->tail;

    return recv_data;
}

/**
 * @brief ???????????????,???????????????
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config)
{
    Nac_Send_s *send_data = (Nac_Send_s *)malloc(sizeof(Nac_Send_s));
    memset(send_data, 0, sizeof(Nac_Send_s));

    send_data->header   = send_config->header;
    send_data->tail     = send_config->tail;
    return send_data;
}

Nac_Recv_s *NacInit(UART_HandleTypeDef *nac_usart_handle)
{
    nac_instance = (Nac_Instance *)malloc(sizeof(Nac_Instance));
    memset(nac_instance, 0, sizeof(Nac_Instance));
	
		USART_Init_Config_s conf;
    conf.module_callback = DecodeNac;
    conf.recv_buff_size  = VISION_RECV_SIZE;
    conf.usart_handle    = nac_usart_handle;

    nac_instance->usart                = USARTRegister(&conf);
    Nac_Recv_Init_Config_s recv_config = {
        .header1 = NAC_RECV_HEADER1,

			  .tail    = NAC_RECV_TAIL,
    };

    nac_instance->recv_data            = NacRecvRegister(&recv_config);
    Nac_Send_Init_Config_s send_config = {
        .header        = NAC_SEND_HEADER,
         .tail          = NAC_SEND_TAIL,
    };
    nac_instance->send_data = NacSendRegister(&send_config);
    // ?master process??daemon,????????????
    Daemon_Init_Config_s daemon_conf = {
        .callback     = NacOfflineCallback, // ??????????,???????
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    nac_daemon_instance = DaemonRegister(&daemon_conf);

    return nac_instance->recv_data;
}
