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

void DD_init(OLED_HandleTypeDef* holedIn);
void DD_updateRPM(uint32_t rpm);
void DD_updateVolt(int32_t volt);
void DD_updateCrt(int32_t crt);
void DD_updatePwr(int32_t pow);
void DD_updateRadio();


#endif /* DRIVERDISPLAY_H_ */
