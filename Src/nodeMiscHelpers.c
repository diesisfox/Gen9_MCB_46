/*
 * nodeMiscHelpers.c
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 */
#include "nodeMiscHelpers.h"
#include "nodeConf.h"

uint32_t 	selfStatusWord;
extern osMutexId 	swMtxHandle;
extern osMessageQId mainCanTxQHandle;
extern osMessageQId mainCanRxQHandle;
extern osMessageQId motCanTxQHandle;
extern osMessageQId motCanRxQHandle;
extern nodeEntry *  nodeTable;
extern const uint32_t *const acceptedFirmware;

/*
 * Command executer for implementing node command responses
 */
void executeCommand(uint8_t cmd){
	switch(cmd){
	// Hard reset
	case NODE_HRESET:
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Soft Reset
	case NODE_RESET:
		node_shutdown();					// Soft shutdown
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Clean shutdown
	// NOTE: CAN command processor is still active!
	case NODE_SHUTDOWN:
		if((selfState == ACTIVE) || (selfState == INIT)){
			node_shutdown();						// Soft shutdown if node is active

			// XXX 4: User must suspend any additional application tasks
			// xTaskSuspend(ApplicationHandle);		// Suspend any active non-CAN tasks
		}
		break;

	// Node start command from shutdown state
	case NODE_START:
		if(selfState == SHUTDOWN){
			setState(INIT);
			// Flush the Rx queue for fresh state on start-up
			xQueueReset(mainCanRxQHandle);
			xQueueReset(motCanTxQHandle);
			// XXX 2: Flush the application queues!
			// xQueueReset();

			// XXX 3: User must resume additional application tasks
			// xTaskResume(ApplicationHandle);			// Resume any application tasks
		}
		break;

	// CC Acknowledgement of node addition attempt
	case CC_ACK:
		if(selfState == INIT){
			setState(ACTIVE);
			static Can_frame_t newFrame;
			newFrame.id = selfNodeID + swOffset;
			newFrame.dlc = CAN_HB_DLC;
			for(int i=0; i<4; i++){
				newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
			}
			bxCan_sendFrame(&newFrame);
		}
		break;

	// CC Negation of node addition attempt
	case CC_NACK:
		if(selfState == INIT){
			setState(SHUTDOWN);
		}
		break;

	default:
		// Do nothing if the command is invalid
		break;
	}
}

/* CHECKED
 * Thread-safe node state accessor
 */
nodeState getSelfState(){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	nodeState ret = (nodeState)(selfStatusWord & 0x07);
	xSemaphoreGive(swMtxHandle);
	return ret;
}

/* CHECKED
 * Thread-safe node state mutator
 */
void setSelfState(nodeState newState){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	selfStatusWord &= 0xfffffff8;	// Clear the current status word
	selfStatusWord |= newState;
	xSemaphoreGive(swMtxHandle);
}

/*
 * Soft shutdown routine that will complete any critical cleanups via callback
 * Assembles node SHUTDOWN statusword CAN frame
 * Flushes CanTx queue via broadcast on bxCAN
 */
void soft_shutdown(void(*usr_clbk)()){
	// Don't care about locking the statusWord here since we are in Critical Area
	setState(SHUTDOWN);

	// User defined shutdown routine
	//	usr_clbk();

	// Broadcast node shutdown state to main CAN
	Can_frame_t newFrame;
	newFrame.id = radio_SW;
	newFrame.isExt = 0;
	newFrame.dlc = CAN_HB_DLC;
	for(int i=0; i<4; i++){
		newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}
	bxCan_sendFrame(&newFrame);
	// TODO: Test if bxCan_sendFrame can successfully send the new frame and flush the queue
}

// Set up the NodeTable initial states
void setupNodeTable(){
	for(uint8_t i = 0; i < MAX_NODE_NUM; i++){
		nodeTable[i].nodeStatusWord = SW_Sentinel;			// Initialize status word to SENTINEL
		nodeTable[i].nodeFirmwareVersion = SW_Sentinel;		// Initialize firm ware version to SENTINEL
	}

	#ifdef cc_nodeID
		nodeTable[cc_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[cc_nodeID].nodeFirmwareVersion = acceptedFirmware[cc_nodeID];
	#endif

	#ifdef mc_nodeID
		nodeTable[mc_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[mc_nodeID].nodeFirmwareVersion = acceptedFirmware[mc_nodeID];
	#endif

	#ifdef bps_nodeID
		nodeTable[bps_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[bps_nodeID].nodeFirmwareVersion = acceptedFirmware[bps_nodeID];
	#endif

	#ifdef ads_nodeID
		nodeTable[ads_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[ads_nodeID].nodeFirmwareVersion = acceptedFirmware[ads_nodeID];
	#endif

	#ifdef radio_nodeID
		nodeTable[radio_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[radio_nodeID].nodeFirmwareVersion = acceptedFirmware[radio_nodeID];
	#endif

    #ifdef radio_nodeID
		nodeTable[dcb_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[dcb_nodeID].nodeFirmwareVersion = acceptedFirmware[radio_nodeID];
	#endif
}

uint32_t readSwitches(){
    uint32_t retval = 0;
    retval |= readSwitch(LEFT_SIG_SWITCH);
    retval |= readSwitch(RIGHT_SIG_SWITCH);
    retval |= readSwitch(HAZARD_SWITCH);
    retval |= readSwitch(BRK_SWITCH);
    return retval;
}

void reportSwitches(uint32_t x){
    Can_frame_t newFrame;
    newFrame.dlc = 4;
    newFrame.id = swPos;
    newFrame.isExt = 0;
    newFrame.isRemote = 0;
    *(uint32_t*)newFrame.Data = x;
    bxCan_sendFrame(&newFrame);
}

void sendAckPressed(){
    Can_frame_t newFrame;
    newFrame.dlc = 0;
    newFrame.id = cmdAck;
    newFrame.isExt = 0;
    newFrame.isRemote = 0;
    bxCan_sendFrame(&newFrame);
}


uint8_t valToHex(uint8_t i){
	return (i<=9 ? '0'+i : 'A'+i-10);
}

uint8_t HexToVal(uint8_t i){//0xff = invalid char
	if(i>='0' && i<='9'){
		return i-'0';
	}else if(i>='A' && i<='F'){
		return i-'A'+10;
	}else if(i>='a' && i<='f'){
		return i-'a'+10;
	}
	return 0xff;
}

void applyStr(uint8_t* buf, uint8_t* str, uint32_t len){
	for(uint32_t i=0; i<len; i++){
		buf[i] = str[i];
	}
}

void intToHex(uint32_t input, uint8_t *str, int length){
	for(int i=0; i<length; i++){
		str[length-1-i]=valToHex(input&0x0F);
		input = input>>4;
	}
}

void bytesToReg(uint8_t * byte, uint32_t * reg){
	*reg =  ((byte[3] | (byte[2]<<8) | (byte[1] << 16) | (byte[0] << 24))) & 0xFFFFFFFF;
}

void regToBytes(uint32_t * reg, uint8_t * bytes){
	bytes[3] = (*reg) & 0xFF;
	bytes[2] = ((*reg) >> 8) & 0xFF;
	bytes[1] = ((*reg) >> 16) & 0xFF;
	bytes[0] = ((*reg) >> 24) & 0xFF;
}

uint8_t printFixedNum(int32_t n, int8_t magnitude, uint8_t* str, uint8_t maxLen){ //returns length written
	static uint8_t digitBuf[10];
	uint8_t len = 0;
	uint8_t nOrder = 0;
	uint32_t nDecimal = n%
	// clear the target
	for(uint8_t i=0; i<maxLen; i++){
		str[i] = ' ';
	}
	// absolute the input
	if(n<0){
		str[len] = '-';
		n = -n;
		len++;
	}
	// find number of sig figs of integer n
	for(uint8_t i=0; digitBuf; i++){
		digitBuf[i] = n%10;
		n/=10;
		nOrder++;
	}
	if(magnitude<0){
		// making n smaller
		uint8_t decimalStart = -(magnitude-1);
		// whole part
		for(uint8_t i=nOrder-1; i>decimalStart; i--){
			if(len<maxLen){
				str[len] = digitBuf[i]+'0';
				len++;
			}else{
				str[len-1] = 0xf6;
				break;
			}
		}
		if(len<maxLen){
			str[len] = '.';
			len++;
		}else break;
		// decimal part
		for(int8_t i=decimalStart; i>=0; i--){ //expecting roll over after 0
			if(len<maxLen){
				str[len] = digitBuf[i]+'0';
				len++;
			}else{
				if(digitBuf[i]>=5&&str[len-1]<'9') str[len-1]++;
				break;
			}
		}
	}else if(magnitude>=0){
		// making n larger
		// n part
		for(int8_t i=nOrder-1; i>=0; i--){
			str[len] = digitBuf[i]+'0';
			len++;
		}else{
			str[len-1] = 0xf6;
			break;
		}
		// 0 part
		for(int8_t i=0; i<magnitude; i++){
			str[len] = '0';
			len++;
		}else{
			str[len-1] = 0xf6;
			break;
		}
	}
	return len;
}
