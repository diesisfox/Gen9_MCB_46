/*
 * oled2004.c
 *
 *  Created on: Apr 12, 2017
 *      Author: jamesliu
 */

#include "oled2004.h"

#define MAX_OLEDS_PHY				4

#define CMD_RAW						0
#define CMD_CLEAR_DISPLAY			0x1	//values in hex double as command template
#define CMD_RETURN_HOME				0x2
#define CMD_ENTRY_MODE_SET			0x4
#define CMD_DISPLAY_ON_OFF_CONTROL	0x8
#define CMD_CURSOR_DISPLAY_SHIFT	0x10
#define CMD_FUNCTION_SET			0x20
#define CMD_SET_CGRAM_ADDRESS		0x40
#define CMD_SET_DDRAM_ADDRESS		0x80
#define CMD_READ_BUSY_FLAG_ADDRESS	3
#define CMD_WRITE_FRAME				5
#define CMD_READ_DATA				6
#define CMD_WRITE_LINES				7
#define CMD_WRITE_CUSTOM_CHAR		9

#define OLED_SPI_CMD_PREAMBLE		0x0000
#define OLED_SPI_BUSY_FLAG_PREAMBLE	0x4000
#define OLED_SPI_WRITE_PREAMBLE		0x8000
#define OLED_SPI_READ_PREAMBLE		0xc000

#define PACKCMD_0(x) ((x)<<6)
#define CGRAM_ADDRESSIFY(c,r) ((((c)&7)<<3)|((r)&7))
#define DDRAM_ADDRESSIFY(r,c) ((((r)&1)?0x40:0x00)+(((r)&2)?0x14:0x00)+((c)%20))

typedef struct{
	uint8_t cmd;
	union{
		struct{
			uint16_t x	:8;
		}rawCommand_Args;
		struct{
			uint8_t i_d	:1;
			uint8_t s	:1;
		}entryModeSet_Args;
		struct{
			uint8_t d	:1;
			uint8_t c	:1;
			uint8_t b	:1;
		}displayOnOff_Args;
		struct{
			uint8_t x	:2;
		}cursorDisplayShift_Args;
		struct{
			uint8_t dl	:1;
			uint8_t ft	:2;
		}functionSet_Args;
		struct{
			uint8_t row	:3;
			uint8_t ch	:3;
		}setCGRamAddr_Args;
		struct{
			uint8_t row	:2;
			uint8_t col	:5;
		}setDDRamAddr_Args;
		struct{
			uint8_t rows:4;
		}writePartialData_Args;
	};
}OLED_Cmd_t;

static uint8_t checkBusyFlag(OLED_HandleTypeDef* holed, uint8_t* addrOut);
static void waitBusyFlag(OLED_HandleTypeDef* holed);
static void doOLED(void * pvParameters);
static void writeLength(OLED_HandleTypeDef* holed, uint8_t len);
static void writeFrame(OLED_HandleTypeDef* holed);
static void sendOneCmd(OLED_HandleTypeDef* holed, uint16_t cmd);
static void sendCommandChain(OLED_HandleTypeDef* holed, uint16_t* buf);
static void sendDataChain(OLED_HandleTypeDef* holed, uint8_t* buf);
static void readDataChain(OLED_HandleTypeDef* holed, uint8_t* buf);

static OLED_HandleTypeDef* holeds[MAX_OLEDS_PHY] = {0};


int OLED_init(OLED_HandleTypeDef* holed, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin){
	for(int i=0; i<MAX_OLEDS_PHY; i++){
		if(holeds[i]==holed) return -2;
		if(holeds[i]==NULL){
			holeds[i] = holed;

			holed->hspi			= hspi;
			holed->csPort		= csPort;
			holed->csPin		= csPin;
			holed->txMtx		= xSemaphoreCreateMutex();
			holed->rxSem		= xSemaphoreCreateBinary();
			holed->awaitingCb	= 0;
			holed->busyFlag		= 0;
			holed->CGRAMAddr.c	= 0;
			holed->CGRAMAddr.r	= 0;
			holed->cursorAddr.c	= 0;
			holed->cursorAddr.r	= 0;
			holed->cmdQ			= xQueueCreate(OLED_CMD_BUF_LEN, sizeof(OLED_Cmd_t));
			holed->dataQ		= xQueueCreate(OLED_FRAME_BUF_LEN, OLED_BUFFER_SIZE);
			holed->config		= (OLED_ConfigTypeDef){0};

			xTaskCreate(doOLED, "OLED", 512, (void*)holed, 3, &(holed->task));
			return 0;
		}
	}
	return -1;
}

void OLED_clearDisplay(OLED_HandleTypeDef* holed){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_CLEAR_DISPLAY;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_returnHome(OLED_HandleTypeDef* holed){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_RETURN_HOME;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_entryModeSet(OLED_HandleTypeDef* holed, uint8_t id, uint8_t s){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_ENTRY_MODE_SET;
	cmd.entryModeSet_Args.i_d = id;
	cmd.entryModeSet_Args.s = s;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_displayOnOff(OLED_HandleTypeDef* holed, uint8_t d, uint8_t c, uint8_t b){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_DISPLAY_ON_OFF_CONTROL;
	cmd.displayOnOff_Args.b = b;
	cmd.displayOnOff_Args.c = c;
	cmd.displayOnOff_Args.d = d;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_cursorDisplayShift(OLED_HandleTypeDef* holed, uint8_t mode){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_CURSOR_DISPLAY_SHIFT;
	cmd.cursorDisplayShift_Args.x = mode;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_setDataLength(OLED_HandleTypeDef* holed, uint8_t dl){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_FUNCTION_SET;
	cmd.functionSet_Args.dl = dl;
	cmd.functionSet_Args.ft = holed->config.fontBank;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_setFontTable(OLED_HandleTypeDef* holed, uint8_t ft){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_FUNCTION_SET;
	cmd.functionSet_Args.dl = holed->config.dataLength;
	cmd.functionSet_Args.ft = ft;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_setCustomChar(OLED_HandleTypeDef* holed, uint8_t num, uint8_t* buf){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_SET_CGRAM_ADDRESS;
	cmd.setCGRamAddr_Args.row = 0;
	cmd.setCGRamAddr_Args.ch = num&7;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
	cmd.cmd = CMD_WRITE_CUSTOM_CHAR;
	xQueueSend(holed->dataQ, buf, portMAX_DELAY);
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_setCursor(OLED_HandleTypeDef* holed, uint8_t r, uint8_t c){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_SET_DDRAM_ADDRESS;
	cmd.setDDRamAddr_Args.row = r;
	cmd.setDDRamAddr_Args.col = c;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_writeFrame(OLED_HandleTypeDef* holed, uint8_t* buf){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_WRITE_FRAME;
	xQueueSend(holed->dataQ, buf, portMAX_DELAY);
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_writeLines(OLED_HandleTypeDef* holed, uint8_t* buf, uint8_t lines){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_WRITE_LINES;
	cmd.writePartialData_Args.rows = lines;
	xQueueSend(holed->dataQ, buf, portMAX_DELAY);
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

void OLED_rawCmd(OLED_HandleTypeDef* holed, uint8_t raw){
	OLED_Cmd_t cmd;
	cmd.cmd = CMD_RAW;
	cmd.rawCommand_Args.x = raw;
	xQueueSend(holed->cmdQ, &cmd, portMAX_DELAY);
}

static uint8_t checkBusyFlag(OLED_HandleTypeDef* holed, uint8_t* addrOut){
	static uint8_t txBuf[3] = {0x40, 0x10, 0x00};
	static uint8_t rxBuf[3];

	(void) holed->hspi->Instance->DR;
	xSemaphoreTake(holed->txMtx, portMAX_DELAY);
	holed->awaitingCb = 1;
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 0);
	HAL_SPI_TransmitReceive(holed->hspi, txBuf, rxBuf, 3, 500);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 1);
	xSemaphoreGive(holed->rxSem);
	xSemaphoreTake(holed->rxSem, portMAX_DELAY);

	if(addrOut) *addrOut = ((rxBuf[1] & 0x3f) << 2) | (rxBuf[2] >> 6);
	holed->busyFlag = (rxBuf[1] & 0x20) >> 5;

	xSemaphoreGive(holed->txMtx);
	return holed->busyFlag;
}

static void waitBusyFlag(OLED_HandleTypeDef* holed){
	while(checkBusyFlag(holed, NULL)){
		osDelay(1);
	}
}

static void doOLED(void * pvParameters){
	OLED_HandleTypeDef* holed = (OLED_HandleTypeDef*)pvParameters;
	OLED_Cmd_t cmd;

	//Init
	OLED_clearDisplay(holed);
	OLED_returnHome(holed);
	OLED_displayOnOff(holed, 1,1,1);
	OLED_entryModeSet(holed, 1,0);
	OLED_setDataLength(holed, 0);
	OLED_setFontTable(holed, 0);

	osDelay(1);

	//Event Loop
	for(;;){
		xQueueReceive(holed->cmdQ, &cmd, portMAX_DELAY);
		while(checkBusyFlag(holed, NULL)){
			osDelay(1);	//TODO: make faster
		}
		switch(cmd.cmd){
			case CMD_RAW:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0(cmd.rawCommand_Args.x));
				break;
			case CMD_CLEAR_DISPLAY:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0(CMD_CLEAR_DISPLAY));
				break;
			case CMD_RETURN_HOME:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0(CMD_RETURN_HOME));
				break;
			case CMD_ENTRY_MODE_SET:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0( \
						CMD_ENTRY_MODE_SET | \
						(cmd.entryModeSet_Args.i_d	<<1) | \
						(cmd.entryModeSet_Args.s	<<0) ));
				break;
			case CMD_DISPLAY_ON_OFF_CONTROL:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0( \
						CMD_DISPLAY_ON_OFF_CONTROL | \
						(cmd.displayOnOff_Args.d	<<2) | \
						(cmd.displayOnOff_Args.c	<<1) | \
						(cmd.displayOnOff_Args.b	<<0) ));
				break;
			case CMD_CURSOR_DISPLAY_SHIFT:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0( \
						CMD_CURSOR_DISPLAY_SHIFT | \
						(cmd.cursorDisplayShift_Args.x	<<2) ));
				break;
			case CMD_FUNCTION_SET:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0( \
						CMD_FUNCTION_SET | \
						(cmd.functionSet_Args.dl	<<4) | \
						(cmd.functionSet_Args.ft	<<0) ));
				break;
			case CMD_SET_CGRAM_ADDRESS:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0( \
						CMD_SET_CGRAM_ADDRESS | \
						CGRAM_ADDRESSIFY(cmd.setCGRamAddr_Args.ch, cmd.setCGRamAddr_Args.row) ));
				break;
			case CMD_SET_DDRAM_ADDRESS:
				sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE | PACKCMD_0( \
						CMD_SET_DDRAM_ADDRESS | \
						DDRAM_ADDRESSIFY(cmd.setDDRamAddr_Args.row, cmd.setDDRamAddr_Args.col) ));
				break;
			case CMD_WRITE_FRAME:
				writeFrame(holed);
				break;
			case CMD_WRITE_CUSTOM_CHAR:
				writeLength(holed, 8);
				break;
			default:
				break;
		}
	}
}

static void writeFrame(OLED_HandleTypeDef* holed){
	static uint8_t frame[4][20];
	static uint8_t buf[129];

	xQueueReceive(holed->dataQ, frame, portMAX_DELAY); //TODO: make faster

	sendOneCmd(holed, OLED_SPI_CMD_PREAMBLE| \
			PACKCMD_0(CMD_SET_DDRAM_ADDRESS|DDRAM_ADDRESSIFY(0,0)));

	waitBusyFlag(holed);

	buf[0] = OLED_SPI_WRITE_PREAMBLE>>8;
	for(int i=0; i<20; i++){
		buf[i+0x00] |= frame[0][i] >> 2;
		buf[i+0x01] = frame[0][i] << 6;
	}
	for(int i=0; i<20; i++){
		buf[i+0x40] |= frame[1][i] >> 2;
		buf[i+0x41] = frame[1][i] << 6;
	}
	for(int i=0; i<20; i++){
		buf[i+0x14] |= frame[2][i] >> 2;
		buf[i+0x15] = frame[2][i] << 6;
	}
	for(int i=0; i<20; i++){
		buf[i+0x54] |= frame[3][i] >> 2;
		buf[i+0x55] = frame[3][i] << 6;
	}

	xSemaphoreTake(holed->txMtx, portMAX_DELAY);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 0);
	HAL_SPI_Transmit(holed->hspi, buf, 129, 500);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 1);
	xSemaphoreGive(holed->rxSem);
	xSemaphoreTake(holed->rxSem, portMAX_DELAY);
	xSemaphoreGive(holed->txMtx);
}

static void writeLength(OLED_HandleTypeDef* holed, uint8_t len){
	static uint8_t buf[80];
	static uint8_t buf2[81];

	xQueueReceive(holed->dataQ, buf, portMAX_DELAY);
	buf2[0] = OLED_SPI_WRITE_PREAMBLE>>8;
	for(int i=0; i<len; i++){
		buf2[i] |= buf[i] >> 2;
		buf2[i+1] = buf[i] << 6;
	}
	xSemaphoreTake(holed->txMtx, portMAX_DELAY);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 0);
	HAL_SPI_Transmit(holed->hspi, buf2, len+1, 500);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 1);
	xSemaphoreGive(holed->rxSem);
	xSemaphoreTake(holed->rxSem, portMAX_DELAY);
	xSemaphoreGive(holed->txMtx);
}

static void sendOneCmd(OLED_HandleTypeDef* holed, uint16_t cmd){
	static uint8_t temp[2];
	temp[0] = (cmd & 0xff00) >> 8;
	temp[1] = cmd & 0x00ff;

	xSemaphoreTake(holed->txMtx, portMAX_DELAY);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 0);
	HAL_SPI_Transmit(holed->hspi, temp, 2, 500);
	HAL_GPIO_WritePin(holed->csPort, holed->csPin, 1);
	xSemaphoreGive(holed->rxSem);
	xSemaphoreTake(holed->rxSem, portMAX_DELAY);
	xSemaphoreGive(holed->txMtx);
}

static void sendCommandChain(OLED_HandleTypeDef* holed, uint16_t* buf){

}

static void sendDataChain(OLED_HandleTypeDef* holed, uint8_t* buf){

}

static void readDataChain(OLED_HandleTypeDef* holed, uint8_t* buf){

}
