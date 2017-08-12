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

#define LOG_FRAME_0_RL_ID	0x05048225
#define LOG_FRAME_0_RR_ID	0x05048245
#define LOG_FRAME_0_FL_ID	0x05048265
#define LOG_FRAME_0_FR_ID	0x05048285
#define LOG_FRAME_1_RL_ID	0x05148225
#define LOG_FRAME_1_RR_ID	0x05148245
#define LOG_FRAME_1_FL_ID	0x05148265
#define LOG_FRAME_1_FR_ID	0x05148285
#define LOG_FRAME_2_VM_ID	0x05248228

extern osMessageQId motCanRxQHandle;
extern OLED_HandleTypeDef holed1;
extern osSemaphoreId motReqSemHandle;

uint32_t reqFrameIds[4] = {0x08f89540,0x08f91540,0x08f99540,0x08fa1540};

static void denomuralize(uint8_t* in, uint8_t* out){ //compress the error frame
	out[0] = (in[0]&0xf) | ((in[0]&0xe)>>1) | ((in[1]<<7));
	out[1] = ((in[1]&0x2)>>1) | ((in[1]&0x8)>>2) | ((in[2]&0x1)<<2) |\
	(in[2]&0x8) | ((in[2]&0x20)>>1) | ((in[3]&0x7)<<5);
	out[2] = ((in[3]&0x8>>3)) | ((in[4]&0x3)<<1);
}

static void printUint16(uint16_t x, uint8_t* out){
	uint8_t buf[5];
	uint8_t msd = 4;
	for(int8_t i=4; i>=0; i--){
		buf[i] = (x%10) + 0x30;
		if(x%10 != 0) msd=i;
		x/=10;
	}
	uint8_t j=0;
	for(uint8_t i=msd; i<5; i++){
		out[j] = buf[i];
		j++;
	}
}

void motCan_Processor(){
	static Can_frame_t inFrame;
	static Can_frame_t newFrame;
	static uint8_t oledBuf[4][20];


	MotCanFrm0_t * data0;
	MotCanFrm1_t * data1;
	MotCanFrm2_t * data2;
	MotLogFrm0_t * logData0;
	MotLogFrm1_t * logData1;
	MotLogFrm2_t * logData2;

	xQueueReceive(motCanRxQHandle, &inFrame, portMAX_DELAY);

    xSemaphoreGive(motReqSemHandle);

//      OLED_writeFrame(&holed1, "request sent");

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
			//*(uint64_t*)newFrame.Data = *(uint64_t*)inFrame.Data;
			bxCan_sendFrame(&newFrame);
			for(uint8_t i=0; i<5; i++){
				oledBuf[0][i] = 0x20;
			}

			printUint16(data0->motorRPM, (uint8_t*)oledBuf);
			OLED_writeFrame(&holed1, (uint8_t*)oledBuf);

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

//            OLED_writeFrame(&holed1, "frame 1 get");

			break;
		case Log_Res_Frm2_RL1:
		case Log_Res_Frm2_RR1:
		case Log_Res_Frm2_FL1:
		case Log_Res_Frm2_FR1:

//			if(*((uint64_t*)(inFrame.Data))){
				data2 = (MotCanFrm2_t*)inFrame.Data;
				denomuralize(inFrame.Data, newFrame.Data);
				newFrame.id = mcError;
				newFrame.dlc = mcError_DLC;
				bxCan_sendFrame(&newFrame);

//                OLED_writeFrame(&holed1, "frame 2 get");
//			}
			break;
		case LOG_FRAME_0_RL_ID:
		case LOG_FRAME_0_RR_ID:
		case LOG_FRAME_0_FL_ID:
		case LOG_FRAME_0_FR_ID:
			logData0 = (MotLogFrm0_t*)inFrame.Data
			printUint16(logData0->motorRPM, (uint8_t*)oledBuf);
			OLED_writeFrame(&holed1, (uint8_t*)oledBuf);

			break;
		case LOG_FRAME_1_RL_ID:
		case LOG_FRAME_1_RR_ID:
		case LOG_FRAME_1_FL_ID:
		case LOG_FRAME_1_FR_ID:
			logData1 = (MotLogFrm1_t*)inFrame.Data

			break;
		case LOG_FRAME_2_VM_ID:
			logData2 = (MotLogFrm2_t*)inFrame.Data

			break;
		default:
			break;
	}
	bxCan_sendFrame(&inFrame);
}
