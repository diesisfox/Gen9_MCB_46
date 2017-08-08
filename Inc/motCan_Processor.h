/*
 * Can_Processor.h
 *
 *  Created on: May 19, 2017
 *      Author: jamesliu
 */

#ifndef MOTCAN_PROCESSOR_H_
#define MOTCAN_PROCESSOR_H_

#include "../../Can_ID.h"
#include "cmsis_os.h"
// #include "can.h"
#include "serial.h"
#include "nodeMiscHelpers.h"

extern uint32_t reqFrameIds[4];

typedef struct{
	uint16_t batteryVoltage : 10; //0.5V
	uint16_t batteryCurrent : 9; //1A
	uint16_t batteryCurrentSign : 1;
	uint16_t motorCurrentPeakAvg : 10; //1A
	uint16_t FET_tmp : 5; //5 Celcius
	uint16_t motorRPM : 12; //1rpm
	uint16_t PWM_duty : 10; //0.5%
	uint16_t leadAngle : 7; //0.5 deg
} __attribute__ ((__packed__)) MotCanFrm0_t; //8 bytes

typedef struct{
	uint16_t powerMode : 1; // ? power mode : eco mode
	uint16_t motorControlMode : 1; // ? PWM mode : current mode
	uint16_t acceleratorPosition : 10; //0.5%
	uint16_t regen_VR_Position : 10; //0.5%
	uint16_t digit_SW_Position : 4;
	uint16_t outputTargetValue : 10; //0.5A in current mode, 0.5% in PWM mode
	uint16_t driveActionStatus : 2; //0=stop, 1=RFU, 2=forward, 3=reverse
	uint16_t regenStatus : 1; // ? regeneration : drive
	uint16_t : 1;
} __attribute__ ((__packed__)) MotCanFrm1_t; //5 bytes

typedef struct{
	uint16_t analogSensorErr : 1;
	uint16_t U_PhaseSensorErr : 1;
	uint16_t W_PhaseSensorErr : 1;
	uint16_t controllerTempSensorErr : 1;
	uint16_t : 1;
	uint16_t batVoltSensorErr : 1;
	uint16_t batCurrSensorErr : 1;
	uint16_t batCurrReferenceErr : 1;
	uint16_t motCurrReferenceErr : 1;
	uint16_t acceleratorPosErr : 1;
	uint16_t : 1;
	uint16_t line12VSensorErr : 1;
	uint16_t : 4;
	uint16_t powerSystemErr : 1;
	uint16_t overCurrErr : 1;
	uint16_t : 1;
	uint16_t overVoltErr : 1;
	uint16_t : 1;
	uint16_t overCurrLimit : 1;
	uint16_t : 2;
	uint16_t motorSystemErr : 1;
	uint16_t motorLockErr : 1;
	uint16_t hallSensorShortErr : 1;
	uint16_t hallSensorOpenErr : 1;
	uint16_t : 4;
	uint16_t overHeatLevel : 2;
	uint16_t : 6;
} __attribute__ ((__packed__)) MotCanFrm2_t; //5 bytes

void motCan_Processor(void);

#endif /* MOTCAN_PROCESSOR_H_ */
