/*
 * Can_Processor.c
 *
 *  Created on: May 19, 2017
 *      Author: jamesliu
 */
#include "motCan_Processor.h"
#include "nodeConf.h"
#include "../CAN_ID.h"

extern osMessageQId motCanRxQHandle;

static void denomuralize(uint8_t* in, uint8_t* out){ //compress the error frame
	out[0] = (in[0]&0xf) | ((in[0]&0xe)>>1) | ((in[1]<<7));
	out[1] = ((in[1]&0x2)>>1) | ((in[1]&0x8)>>2) | ((in[2]&0x1)<<2) |\
	(in[2]&0x8) | ((in[2]&0x20)>>1) | ((in[3]&0x7)<<5);
	out[2] = ((in[3]&0x8>>3)) | ((in[4]&0x3)<<1);
}

void motCan_Processor(){
	static Can_frame_t inFrame;
	static Can_frame_t newFrame;
	xQueueReceive(motCanRxQHandle, &inFrame, portMAX_DELAY);
	uint8_t motNum = ((inFrame.id & 0xf0) >> 1)-1; //2,4,6,8 to 0,1,2,3
	switch (inFrame.id) {
		case Log_Res_Frm0_RL1:
		case Log_Res_Frm0_RR1:
		case Log_Res_Frm0_FL1:
		case Log_Res_Frm0_FR1:
          {
            MotCanFrm0_t* data = (MotCanFrm0_t*)inFrame.Data;
			newFrame.id = mcDiag0;
			newFrame.dlc = mcDiag0_DLC;
			*(uint64_t*)newFrame.Data = *(uint64_t*)inFrame.Data;
			bxCan_sendFrame(&newFrame);
			break;
          }
		case Log_Res_Frm1_RL1:
		case Log_Res_Frm1_RR1:
		case Log_Res_Frm1_FL1:
		case Log_Res_Frm1_FR1:
          {
			MotCanFrm1_t * data = (MotCanFrm1_t*)inFrame.Data;

			newFrame.id = accelPos;
			newFrame.dlc = accelPos_DLC;
			*(uint16_t*)newFrame.Data = data->acceleratorPosition;
			bxCan_sendFrame(&newFrame);

			newFrame.id = regenPos;
			newFrame.dlc = regenPos_DLC;
			*(uint16_t*)newFrame.Data = data->regen_VR_Position;
			bxCan_sendFrame(&newFrame);

			newFrame.id = mcDiag1;
			newFrame.dlc = mcDiag1_DLC;
			*(uint64_t*)newFrame.Data = *(uint64_t*)inFrame.Data;
			bxCan_sendFrame(&newFrame);

			break;
          }
		case Log_Res_Frm2_RL1:
		case Log_Res_Frm2_RR1:
		case Log_Res_Frm2_FL1:
		case Log_Res_Frm2_FR1:
          {
			if(*(uint64_t*)inFrame.Data){
				MotCanFrm2_t * data = (MotCanFrm2_t*)inFrame.Data;
				denomuralize(inFrame.Data, newFrame.Data);
				newFrame.id = mcError;
				newFrame.dlc = mcError_DLC;
				bxCan_sendFrame(&newFrame);
			}
			break;
          }
		default:
			break;
	}
	bxCan_sendFrame(&inFrame);
}
