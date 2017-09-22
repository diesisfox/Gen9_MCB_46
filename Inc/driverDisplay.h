/*
 * driverDisplay.h
 *
 *  Created on: Aug 11, 2017
 *      Author: jamesliu
 */

#ifndef DRIVERDISPLAY_H_
#define DRIVERDISPLAY_H_


#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "oled2004.h"
#include "nodeConf.h"
#include "nodeMiscHelpers.h"
#include "customChars.h"

void DD_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void DD_init(OLED_HandleTypeDef* holedIn);
void DD_updateSpeed(int32_t rpm);
void DD_updateVolt(int32_t volt);
void DD_updateAmp(int32_t amp);
void DD_updateCellV(uint8_t* data; uint8_t index);


#endif /* DRIVERDISPLAY_H_ */
