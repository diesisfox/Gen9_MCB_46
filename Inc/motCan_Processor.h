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

typedef struct{
	uint16_t battVolt		: 10;	// 0.5V/LSB; 0-500V
	uint16_t battCrt		: 9;	// 1A/LSB; 0-250A
	uint16_t battCrtSgn		: 1;	// 1 = minus
	uint16_t motorCrt		: 9;	// 1A/LSB; 0-250A
	uint16_t motorCrtSgn	: 1;	// 1 = minus
	uint16_t tempControl	: 5;	// 5°C/LSB; 0-100°C
	uint16_t tempMotor		: 5;	// 5°C/LSB; 0-100°C
	uint16_t motorRPM		: 12;	// 1rpm/LSB; 0-3000rpm
	uint16_t outputDuty		: 10;	// TODO wtf is dis measured in
	uint16_t outputDutyType	: 1;	// 0=running, 1=regen
	uint16_t				: 1;	// reserved
} __attribute__ ((__packed__)) MotLogFrm0_t; //8 bytes

typedef struct{
	uint16_t motorDriveMode	: 8;	// 0x00 = stop, 0x01 = suspend, 0x10 = forward rotation
	uint16_t failModeInfo	: 5;	// ?|sensor|power|motor|overcharge failModeInfo
	uint16_t tempErrLevel	: 3;	// 0 = normal, 1-3 = levels 1 to 3 of overheating
} __attribute__ ((__packed__)) MotLogFrm1_t; //2 bytes

typedef struct{
	uint16_t presentCorePos	: 8;	// 1%/LSB; core pos is 100% at maximum stroke
	uint16_t failModeInfo	: 4;	// ?|sensor|power|motor error
	uint16_t				: 1;	// reserved
	uint16_t				: 3;	// reserved
} __attribute__ ((__packed__)) MotLogFrm2_t; //2 bytes

void motCan_Processor(void);

#endif /* MOTCAN_PROCESSOR_H_ */
