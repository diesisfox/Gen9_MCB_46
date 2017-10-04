/*
 * Can_Processor.c
 *
 *  Created on: May 19, 2017
 *      Author: jamesliu
 */
#include "motCan_Processor.h"
#include "nodeConf.h"
#include "../CAN_ID.h"
#include "oled2004.h"
#include "driverDisplay.h"

#define LOG_FRAME_0_RL_ID	0x05048225
#define LOG_FRAME_0_RR_ID	0x05048245
#define LOG_FRAME_0_FL_ID	0x05048265
#define LOG_FRAME_0_FR_ID	0x05048285
#define LOG_FRAME_1_RL_ID	0x05148225
#define LOG_FRAME_1_RR_ID	0x05148245
#define LOG_FRAME_1_FL_ID	0x05148265
#define LOG_FRAME_1_FR_ID	0x05148285
#define LOG_FRAME_2_VM_ID	0x05248228
#define WS22_BUS_MES_ID		0x402
#define WS22_VELO_MES_ID	0x403

extern osMessageQId motCanRxQHandle;
extern OLED_HandleTypeDef holed1;
extern osSemaphoreId motReqSemHandle;
extern uint8_t regenOn;

uint32_t reqFrameIds[4] = {0x08f89540,0x08f91540,0x08f99540,0x08fa1540};

static void denomuralize(uint8_t* in, uint8_t* out){ //compress the error frame
	out[0] = (in[0]&0xf) | ((in[0]&0xe)>>1) | ((in[1]<<7));
	out[1] = ((in[1]&0x2)>>1) | ((in[1]&0x8)>>2) | ((in[2]&0x1)<<2) |\
	(in[2]&0x8) | ((in[2]&0x20)>>1) | ((in[3]&0x7)<<5);
	out[2] = ((in[3]&0x8>>3)) | ((in[4]&0x3)<<1);
}

void motCan_Processor(){
	static Can_frame_t inFrame;
	static Can_frame_t newFrame;

	MotCanFrm0_t * data0;
	MotCanFrm1_t * data1;
	MotCanFrm2_t * data2;
	MotLogFrm0_t * logData0;
	MotLogFrm1_t * logData1;
	MotLogFrm2_t * logData2;

	xQueueReceive(motCanRxQHandle, &inFrame, portMAX_DELAY);

	xSemaphoreGive(motReqSemHandle);

	newFrame.isExt = 0;

	uint8_t motNum = ((inFrame.id & 0xf0) >> 1)-1; //2,4,6,8 to 0,1,2,3
	switch (inFrame.id) {
		case Log_Res_Frm0_RL1:
		case Log_Res_Frm0_RR1:
		case Log_Res_Frm0_FL1:
		case Log_Res_Frm0_FR1:

			data0 = (MotCanFrm0_t*)inFrame.Data;
			newFrame.id = mcDiag0;
			newFrame.dlc = mcDiag0_DLC;
			for(uint8_t i=0; i<8; i++){
				newFrame.Data[i] = inFrame.Data[i];
			}
			bxCan_sendFrame(&newFrame);
			DD_updateSpeed(data0->motorRPM);

			break;
		case Log_Res_Frm1_RL1:
		case Log_Res_Frm1_RR1:
		case Log_Res_Frm1_FL1:
		case Log_Res_Frm1_FR1:

			data1 = (MotCanFrm1_t*)inFrame.Data;

			newFrame.id = accelPos;
			newFrame.dlc = accelPos_DLC;
			*(uint16_t*)newFrame.Data = data1->acceleratorPosition;
			bxCan_sendFrame(&newFrame);

			newFrame.id = regenPos;
			newFrame.dlc = regenPos_DLC;
			*(uint16_t*)newFrame.Data = data1->regen_VR_Position;
			bxCan_sendFrame(&newFrame);

			newFrame.id = mcDiag1;
			newFrame.dlc = mcDiag1_DLC;
			for(uint8_t i=0; i<8; i++){
				newFrame.Data[i] = inFrame.Data[i];
			}
			bxCan_sendFrame(&newFrame);

			break;
		case Log_Res_Frm2_RL1:
		case Log_Res_Frm2_RR1:
		case Log_Res_Frm2_FL1:
		case Log_Res_Frm2_FR1:
			data2 = (MotCanFrm2_t*)inFrame.Data;
			denomuralize(inFrame.Data, newFrame.Data);
			newFrame.id = mcError;
			newFrame.dlc = mcError_DLC;
			bxCan_sendFrame(&newFrame);
			break;
		case LOG_FRAME_0_RL_ID:
		case LOG_FRAME_0_RR_ID:
		case LOG_FRAME_0_FL_ID:
		case LOG_FRAME_0_FR_ID:
			logData0 = (MotLogFrm0_t*)inFrame.Data;
			DD_updateSpeed(logData0->motorRPM);
			regenOn = logData0->outputDutyType;
			break;
		case LOG_FRAME_1_RL_ID:
		case LOG_FRAME_1_RR_ID:
		case LOG_FRAME_1_FL_ID:
		case LOG_FRAME_1_FR_ID:
			logData1 = (MotLogFrm1_t*)inFrame.Data;

			break;
		case LOG_FRAME_2_VM_ID:
			logData2 = (MotLogFrm2_t*)inFrame.Data;

			break;
		case WS22_BUS_MES_ID:
			break;
		case WS22_VELO_MES_ID:{
			static float mps;
			static float rpm;
//			*((uint32_t*)&mps) = *((uint32_t*)(inFrame.Data+4));
			*((uint32_t*)&rpm) = *((uint32_t*)(inFrame.Data+0));
//			if(mps<0) mps*=-1;
			if(rpm<0) rpm*=-1;
//			mps = (mps*3600/1000)*1000;
			DD_updateSpeed((int16_t)rpm);
			break;
		}
		default:
			break;
	}
	bxCan_sendFrame(&inFrame);
}
