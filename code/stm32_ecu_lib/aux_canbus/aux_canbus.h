#ifndef _AUX_CANBUS_H
#define _AUX_CANBUS_H
#include "main.h"
extern void CAN_SEND_MESS_RPM(CAN_HandleTypeDef *hcan,uint16_t RPM);
extern void CAN_SEND_THROTTLE_SW(CAN_HandleTypeDef *hcan,uint8_t THROTTLE_SW);
void CAN_SEND_DATA_SENSOR(CAN_HandleTypeDef *hcan,uint8_t MAP_DATA,uint16_t LAMBDA,uint8_t AIR_TEMP,uint8_t COOLANT);
void CAN_SEND_DATA_PW(CAN_HandleTypeDef *hcan,uint16_t PW_INJECT,uint16_t PW_IG);
#endif

