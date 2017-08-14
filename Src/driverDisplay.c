/*
 * driverDisplay.c
 *
 *  Created on: Aug 11, 2017
 *      Author: jamesliu
 */

#include "driverDisplay.h"

#define MAX_FPS 60
#define FPS_DELAY 1000/MAX_FPS
#define FILTER_SIZE 16

/*
AaaaaAAA•Bbbb.bbB••E
GggGG•HhhH•IiiiIiiiI
Ccc.ccC•Dddddd.ddD•F
JjjjjjjjjjjjjjjjjjjJ

A: rpm/speed; B: voltage; C: current; D:power; E:radio; F:ack
G: temp; H: SoC; I: cells; J: trips&status
*/

/*
AAAAAA SPEED: BBBBBB
AAAAAA-999.99 BBBBBB
AAAAAA POWER: BBBBBB
AAAAAA-9999.99BBBBBB

A: left signal zone; B: right signal zone
*/

#define RPM_ADDR 0][1
#define VOLT_ADDR 0][10
#define CRT_ADDR 2][1
#define PWR_ADDR 2][9
#define RAD_ADDR 0][20
#define ACK_ADDR 2][20

static uint8_t buf[4][20];
static uint8_t* buff;
static SemaphoreHandle_t updateSem, radioSem;
static uint8_t linesToUpdate = 0;
static OLED_HandleTypeDef* holed;
static TaskHandle_t ddTask, radioAnimTask, rpmAnimTask, speedAnimTask;
static uint16_t rpm = 0;
static int32_t voltBuf[FILTER_SIZE], crtBuf[FILTER_SIZE], powBuf[FILTER_SIZE];
static uint8_t voltBufInd = 0, crtBufInd = 0, powBufInd = 0;
static uint8_t bufFilled = 0; //?|?|?|?|?|p|i|v

static void doDD(void* pvParameters);
static void doRadioAnim(void* arg);
static void doRpmAnim(void* arg);
static void doSpeedAnim(void* arg);
static void setupIcons();

void DD_init(OLED_HandleTypeDef* holedIn){
	buff = (uint8_t*) buf;
	for(uint8_t i=0; i<sizeof(buf); i++){
		buff[i] = ' ';
	}
	updateSem = xSemaphoreCreateBinary();
	radioSem = xSemaphoreCreateBinary();
	holed = holedIn;
	setupIcons();
	OLED_displayOnOff(holed, 1, 0, 0);
	xTaskCreate(doDD, "DDTask", 1024, NULL, 3, &ddTask);
	xTaskCreate(doRadioAnim, "RadioAnimTask", 512, NULL, 2, &radioAnimTask);
	xTaskCreate(doRpmAnim, "RpmAnimTask", 512, NULL, 2, &rpmAnimTask);
}

void DD_updateRPM(uint16_t rpmIn){
	rpm = rpmIn;
	uint8_t len = 0;
	for(uint8_t i=0; i<7; i++){
		buf[RPM_ADDR+i] = ' ';
	}
	len += intToDec(rpm, &(buf[RPM_ADDR]));
	applyStr(&(buf[RPM_ADDR+len]), "rpm", 3);
	if(updateSem) xSemaphoreGive(updateSem);
}

void DD_updateVolt(int32_t volt){
	voltBuf[voltBufInd] = volt;
	volt = 0;
	for(int i=0; i<FILTER_SIZE; i++){
		volt += voltBuf[i]/((bufFilled&0x01)?FILTER_SIZE:(voltBufInd+1));
	}
	voltBufInd++;
	if(voltBufInd == FILTER_SIZE){
		voltBufInd = 0;
		bufFilled |= 0x01;
	}
	uint8_t len = 0;
	for(uint8_t i=0; i<7; i++){
		buf[VOLT_ADDR+i] = ' ';
	}
	if(volt<0){
		buf[VOLT_ADDR+len] = '-';
		len++;
		volt = -volt;
	}
	int32_t volt1 = volt/1000000;
	len += intToDec(volt1, &(buf[VOLT_ADDR+len]));
	volt1 = (volt%1000000)/10000;
	if(volt1){
		buf[VOLT_ADDR+len] = '.';
		len++;
		len += intToDec(volt1, &(buf[VOLT_ADDR+len]));
	}
	buf[VOLT_ADDR+len] = 'V';
	if(updateSem) xSemaphoreGive(updateSem);
}

void DD_updateCrt(int32_t crt){
	crtBuf[crtBufInd] = crt;
	crt = 0;
	for(int i=0; i<FILTER_SIZE; i++){
		crt += crtBuf[i]/((bufFilled&0x02)?FILTER_SIZE:(crtBufInd+1));
	}
	crtBufInd++;
	if(crtBufInd == FILTER_SIZE){
		crtBufInd = 0;
		bufFilled |= 0x02;
	}
	uint8_t len = 0;
	for(uint8_t i=0; i<6; i++){
		buf[CRT_ADDR+i] = ' ';
	}
	if(crt<0){
		buf[CRT_ADDR+len] = '-';
		len++;
		crt = -crt;
	}
	int32_t crt1 = crt/1000000;

	len += intToDec(crt1, &(buf[CRT_ADDR+len]));
	crt1 = (crt%1000000)/10000;

	if(crt1){
		buf[CRT_ADDR+len] = '.';
		len++;
		len += intToDec(crt1, &(buf[CRT_ADDR+len]));
	}
	buf[CRT_ADDR+len] = 'A';
	if(updateSem) xSemaphoreGive(updateSem);
}

void DD_updatePwr(int32_t pow){
	powBuf[powBufInd] = pow;
	pow = 0;
	for(int i=0; i<FILTER_SIZE; i++){
		pow += powBuf[i]/((bufFilled&0x04)?FILTER_SIZE:(powBufInd+1));
	}
	powBufInd++;
	if(powBufInd == FILTER_SIZE){
		powBufInd = 0;
		bufFilled |= 0x04;
	}
	uint8_t len = 0;
	for(uint8_t i=0; i<9; i++){
		buf[PWR_ADDR+i] = ' ';
	}
	if(pow<0){
		buf[PWR_ADDR+len] = '-';
		len++;
		pow = -pow;
	}
	int32_t pow1 = pow/1000000;

	len += intToDec(pow1, &(buf[PWR_ADDR+len]));
	pow1 = (pow%1000000)/10000;

	if(pow1){
		buf[PWR_ADDR+len] = '.';
		len++;
		len += intToDec(pow1, &(buf[PWR_ADDR+len]));
	}
	buf[PWR_ADDR+len] = 'W';
	if(updateSem) xSemaphoreGive(updateSem);
}

void DD_updateRadio(){
	if(radioSem) xSemaphoreGive(radioSem);
}

static void doDD(void* pvParameters){
	for(;;){
		xSemaphoreTake(updateSem, portMAX_DELAY);
		OLED_writeFrame(holed, buff);
		osDelay(FPS_DELAY);
	}
}

static void setupIcons(){
	OLED_setCustomChar(holed, 0, cc_rpm0);
	buf[RPM_ADDR-1] = 0;
	OLED_setCustomChar(holed, 1, cc_lightning);
	buf[VOLT_ADDR-1] = 1;
	OLED_setCustomChar(holed, 2, cc_plug);
	buf[CRT_ADDR-1] = 2;
	OLED_setCustomChar(holed, 3, cc_powerOn);
	buf[PWR_ADDR-1] = 3;
	OLED_setCustomChar(holed, 4, cc_blank);
	buf[RAD_ADDR-1] = 4;
	OLED_setCustomChar(holed, 5, cc_blank);
	buf[ACK_ADDR-1] = 5;
}

static void doRadioAnim(void* arg){
	for(;;){
		xSemaphoreTake(radioSem, portMAX_DELAY);
		OLED_setCustomChar(holed, 4, cc_wifi6);
		osDelay(100);
		OLED_setCustomChar(holed, 4, cc_wifi5);
		osDelay(100);
		OLED_setCustomChar(holed, 4, cc_wifi3);
		osDelay(100);
		OLED_setCustomChar(holed, 4, cc_wifi7);
		osDelay(100);
	}
}

static void doRpmAnim(void* arg){
	uint8_t frame;
	uint8_t* frames[4] = {cc_rpm0, cc_rpm1, cc_rpm2, cc_rpm3};
	uint16_t delay = 0;
	for(;;){
		if(rpm == 0){
			frame = 3;
			delay = 500;
		}else if(rpm < 50){
			delay = 500;
		}else if(rpm < 100){
			delay = 333;
		}else if(rpm < 200){
			delay = 200;
		}else if(rpm < 300){
			delay = 150;
		}else{
			delay = 100;
		}
		frame = (frame+1)%4;
		OLED_setCustomChar(holed, 0, frames[frame]);
		osDelay(delay);
	}
}

static void doSpeedAnim(void* arg){
	for(;;){

	}
}
