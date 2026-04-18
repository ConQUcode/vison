//#include "feite_motor.h"


//static uint8_t idx;
//static FTMotor_instance *ft_motor_instance[FT_MOTOR_CNT] = {NULL};
//uint8_t wBuf[128];




///**
// * @brief  ?????????????????,????????????,?????????????
// *
// * @param motor_ptr
// */
//static void FTMotorLostCallback(void *motor_ptr)
//{
//    FTMotor_instance *motor                 = (FTMotor_instance *)motor_ptr;
//    motor->motor_ref.stop_flag                         = FEITE_STOP;
//   
//}





////??????????????
//FTMotor_instance *FTMotorInit(FTMotor_Init_Config_s *config){
//	
//	if(idx > FT_MOTOR_CNT){
//		return NULL;
//	}

//	FTMotor_instance *motor = (FTMotor_instance *)malloc(sizeof(FTMotor_instance));
//	memset(motor,0,sizeof(FTMotor_instance));
//	
//	// ????????????
//	motor->motor_set  = config->motor_set;
//	motor->motor_ref = config->motor_ref;
//	// ?????UART?????
//  motor->ft_usart_instance        = USARTRegister(&config->usart_init_config);
//	
//	// ???????????
//  Daemon_Init_Config_s daemon_config = {
//      .callback     = FTMotorLostCallback,
//      .owner_id     = motor,
//      .reload_count = 2, //20ms¦Ä??????????
//  };
//  motor->ft_daemon = DaemonRegister(&daemon_config);
//	
//	ft_motor_instance[idx++] = motor;
//	return motor;
//	
//}	


////------------------------------------------------------------------------------------------------

////UART ??????????
//int writeSCS(FTMotor_instance *motor,unsigned char *Dat, int Len)
//{
//	while(Len--){
//		if(motor->wLen<sizeof(motor->data_send_buff)){
//			motor->data_send_buff[motor->wLen++] = *Dat;
//			Dat++;
//		}
//	}
//	return motor->wLen;
//}

//void writeBuf(FTMotor_instance *motor)
////void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
//{
//	uint8_t *nDat = motor->raw_send_buff;
//	uint8_t nLen = motor->send_buff_len;
//	
//	
//	uint8_t i;
//	uint8_t msgLen = 2;
//	uint8_t bBuf[6];
//	uint8_t CheckSum = 0;
//	bBuf[0] = 0xff;
//	bBuf[1] = 0xff;
//	bBuf[2] = motor->motor_set.ID;
//	bBuf[4] = motor->motor_set.Fun;
//	if(nDat){
//		msgLen += nLen + 1;
//		bBuf[3] = msgLen;
//		bBuf[5] = motor->motor_set.MemAddr;
//		writeSCS(motor,bBuf, 6);
//		
//	}else{
//		bBuf[3] = msgLen;
//		writeSCS(motor,bBuf, 5);
//	}
//	CheckSum =  motor->motor_set.ID + msgLen + motor->motor_set.Fun + motor->motor_set.MemAddr;
//	if(nDat){
//		for(i=0; i<nLen; i++){
//			CheckSum += nDat[i];
//		}
//		writeSCS(motor,nDat, nLen);
//	}
//	CheckSum = ~CheckSum;
//	writeSCS(motor,&CheckSum,1);
//}

//void genWrite(FTMotor_instance *motor)
//{
//	
//	//---------
////	rFlushSCS();
//	//---------
//	 
//	writeBuf(motor);
//	if(motor->wLen){
//		USARTSend(motor->ft_usart_instance,motor->data_send_buff,motor->wLen,USART_TRANSFER_BLOCKING);//???????????????????????????????????????? USART_TRANSFER_MODE_e ???? bsp_usart ???????
//		motor->wLen = 0;
//	}
////                                                                                                  
//}



//void WritePosEx(FTMotor_instance *motor)
//{
//	
//	 uint8_t ID = motor->motor_set.ID;
//	 uint16_t Position = motor->motor_ref.Position;
//	 uint16_t Speed = motor->motor_ref.Speed;//???????????????
//	 uint8_t ACC = motor->motor_ref.ACC;

//	
//	if(Position<0){
//		Position = -Position;
//		Position |= (1<<15);
//	}

//	motor->raw_send_buff[0] = ACC;
//	motor->raw_send_buff[1] = Position>>8;
//	motor->raw_send_buff[2] = Position&0xff;
//	motor->raw_send_buff[3] = 0>>8;
//	motor->raw_send_buff[4] = 0&0xff;
//	motor->raw_send_buff[5] = Speed>>8;
//	motor->raw_send_buff[6] = Speed&0xff;
//	
//	motor->send_buff_len = 7;
//	
//	genWrite(motor);
//}

//void FTMotorControl(){
//	
//	for(int i=0 ;i<FT_MOTOR_CNT;i++){
//		WritePosEx(ft_motor_instance[i]);
//	}
//	
//}
