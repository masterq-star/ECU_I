#include "aux_canbus.h"
CAN_TxHeaderTypeDef TxHeader;





void CAN_SEND_MESS_RPM(CAN_HandleTypeDef *hcan,uint16_t RPM){
	uint8_t tx_canbus[8];
	tx_canbus[0]= 0x00;
	tx_canbus[1]=  RPM >>8;
	tx_canbus[2]= RPM  & 0xFF;
	
	tx_canbus[3]= 0x00;
	
	tx_canbus[4]= 0x00;
	tx_canbus[5]= 0x00;
	tx_canbus[6]= 0x00;
	tx_canbus[7]= 0x00;
	uint32_t TxMailbox;
	
	CAN_TxHeaderTypeDef TxHeader;
		 TxHeader.DLC = 8;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x123;  // ID
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_canbus, &TxMailbox);


}
void CAN_SEND_THROTTLE_SW(CAN_HandleTypeDef *hcan,uint8_t THROTTLE_SW){
	uint8_t tx_canbus[2];
	tx_canbus[0]= 0x00;
	tx_canbus[1]=  THROTTLE_SW;

	uint32_t TxMailbox;
	
	CAN_TxHeaderTypeDef TxHeader;
		 TxHeader.DLC = 2;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x124;  // ID
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_canbus, &TxMailbox);


}
void CAN_SEND_DATA_SENSOR(CAN_HandleTypeDef *hcan,uint8_t MAP_DATA,uint16_t LAMBDA,uint8_t AIR_TEMP,uint8_t COOLANT){
	uint8_t tx_canbus[5];
	tx_canbus[0]= MAP_DATA;
	tx_canbus[1]=  LAMBDA >>8;
  tx_canbus[2]=  LAMBDA &0XFF;
	tx_canbus[3]=  AIR_TEMP;
	tx_canbus[4]=  COOLANT;
	uint32_t TxMailbox;
	
	CAN_TxHeaderTypeDef TxHeader;
		 TxHeader.DLC = 5;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x125;  // ID
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_canbus, &TxMailbox);


}

void CAN_SEND_DATA_PW(CAN_HandleTypeDef *hcan,uint16_t PW_INJECT,uint16_t PW_IG){
	uint8_t tx_canbus[4];
	tx_canbus[0]= PW_INJECT >>8;
	tx_canbus[1]=  PW_INJECT & 0Xff;
  tx_canbus[2]=  PW_IG >>8;
	tx_canbus[3]=  PW_IG & 0Xff;

	uint32_t TxMailbox;
	
	CAN_TxHeaderTypeDef TxHeader;
		 TxHeader.DLC = 4;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x126;  // ID
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_canbus, &TxMailbox);


}

