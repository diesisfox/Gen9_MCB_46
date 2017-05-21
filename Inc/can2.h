/*
 * can.h
 *
 * Created on: Dec 4, 2016
 *     Author: jamesliu
 *       Note: the HAL CAN driver is a complete friggin hack job. No respect. Pisses me off.
 */

#ifndef CAN2_H_
#define CAN2_H_

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#ifndef CAN_H_

#define CAN_BANKS 14

typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t Data[8];
  uint8_t isExt; //1 or 0
  uint8_t isRemote;
  int filterNum;
}Can_frame_t;

typedef struct
{
  uint32_t id;
  uint32_t mask;
  uint8_t isRemote;
  uint8_t maskRemote;
  uint8_t isExt;
  uint8_t isMasked;
  int filterNum;
}Can_filter_t; //only used for getFilter()

#endif

void bxCan2_begin(CAN_HandleTypeDef *hcan, osMessageQId *rx, osMessageQId *tx);

int bxCan2_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote/*-1 = don't care*/);
int bxCan2_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote/*-1 = don't care*/);
int bxCan2_addFilterStd(uint16_t id, uint8_t isRemote);
int bxCan2_addFilterExt(uint32_t id, uint8_t isRemote);
int bxCan2_getFilter(Can_filter_t *target, int filterNum);
int bxCan2_removeFilter(int filterNum);
int bxCan2_getFilterNum(uint32_t fmi);

int bxCan2_availableForTx();
uint32_t bxCan2DoTx(uint8_t fromISR);
int bxCan2_sendFrame(Can_frame_t *frame);

void CAN2_TxCpltCallback(CAN_HandleTypeDef* hcan);
void CAN2_RxCpltCallback(CAN_HandleTypeDef* hcan);
void CAN2_ErrorCallback(CAN_HandleTypeDef *hcan);

void bxCan2_setTxCallback(void(*pt)());
void bxCan2_setRxCallback(void(*pt)());
void bxCan2_setErrCallback(void(*pt)(uint32_t erCode));

#endif /* CAN2_H_ */
