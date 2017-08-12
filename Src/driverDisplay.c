/*
 * driverDisplay.c
 *
 *  Created on: Aug 11, 2017
 *      Author: jamesliu
 */

#include "driverDisplay.h"

#define MAX_FPS 60
#define FPS_DELAY 1000/MAX_FPS

/*
AaaaaAAA•Bbbb.bbB••E
••••••••••••••••••••
Ccc.ccC•Dddddd.ddD•F
••••••••••••••••••••

A: rpm; B: voltage; C: current; D:power; E:radio; F:ack
*/
#define RPM_ADDR 0][1
#define VOLT_ADDR 0][10
#define CRT_ADDR 3][1
#define PWR_ADDR 3][9
#define RAD_ADDR 1][20
#define ACK_ADDR 3][20

static uint8_t buf[4][20];
static uint8_t* buff;
static SemaphoreHandle_t updateSem, radioSem;
static uint8_t linesToUpdate = 0;
static OLED_HandleTypeDef* holed;
static TaskHandle_t ddTask, radioAnimTask, rpmAnimTask, speedAnimTask;
static uint16_t rpm = 0;

static void doDD(void* pvParameters);
static void doRadioAnim(void* arg);
static void doRpmAnim(void* arg);
static doSpeedAnim(void* arg);
static void setupIcons();

void DD_init(OLED_HandleTypeDef* holedIn){
	buff = (uint8_t*) buf;
	for(uint8_t i=0; i<sizeof(buf); i++){
		buff[i] = ' ';
	}
	updateSem = xSemaphoreCreateBinary();
	radioSem = xSemaphoreCreateBinary();
	holed = holedIn;
	xTaskCreate(doDD, "DDTask", 1024, NULL, 3, &ddTask);
	xTaskCreate(doRadioAnim, "RadioAnimTask", 512, NULL, 2, &radioAnimTask);
	xTaskCreate(doRpmAnim, "RpmAnimTask", 512, NULL, 2, &rpmAnimTask);
}

void DD_updateRPM(uint16_t rpm){
	for(uint8_t i=0; i<7; i++){
		buf[RPM_ADDR+i] = ' ';
	}
	uint8_t len = intToDec(rpm, &(buf[RPM_ADDR]));
	applyStr(&(buf[RPM_ADDR+len]), "rpm", 3);
	xSemaphoreGive(updateSem);
}

void DD_updateVolt(uint32_t volt){
	for(uint8_t i=0; i<7; i++){
		buf[VOLT_ADDR+i] = ' ';
	}
	uint16_t volt1 = volt/1000000;
	uint8_t len = intToDec(volt1, &(buf[VOLT_ADDR]));
	volt1 = (volt%1000000)/1000;
	if(volt1){
		buf[VOLT_ADDR+len] = '.';
		len++;
		len += intToDec(volt1, &(buf[VOLT_ADDR+len]));
	}
	buf[VOLT_ADDR+len] = 'V';
	xSemaphoreGive(updateSem);
}

void DD_updateCrt(uint32_t crt){
	for(uint8_t i=0; i<6; i++){
		buf[CRT_ADDR+i] = ' ';
	}
	uint16_t crt1 = crt/1000000;
	uint8_t len = intToDec(crt1, &(buf[CRT_ADDR]));
	crt1 = (crt%1000000)/1000;
	if(crt1){
		buf[CRT_ADDR+len] = '.';
		len++;
		len += intToDec(crt1, &(buf[CRT_ADDR+len]));
	}
	buf[CRT_ADDR+len] = 'A';
	xSemaphoreGive(updateSem);
}

void DD_updatePwr(uint32_t pow){
	for(uint8_t i=0; i<9; i++){
		buf[PWR_ADDR+i] = ' ';
	}
	uint16_t pow1 = pow/1000;
	uint8_t len = intToDec(pow1, &(buf[PWR_ADDR]));
	pow1 = (pow%1000)/10;
	if(pow1){
		buf[PWR_ADDR+len] = '.';
		len++;
		len += intToDec(pow1, &(buf[PWR_ADDR+len]));
	}
	buf[PWR_ADDR+len] = 'W';
	xSemaphoreGive(updateSem);
}

static void doDD(void* pvParameters){
	for(;;){
		xSemaphoreTake(updateSem, portMAX_DELAY);
		OLED_writeFrame(holed, buff);
		osDelay(FPS_DELAY);
	}
}

static void setupIcons(){
	buf[RPM_ADDR-1] = 0;
	buf[VOLT_ADDR-1] = 1;
	buf[CRT_ADDR-1] = 2;
	buf[PWR_ADDR-1] = 3;
	buf[RAD_ADDR-1] = 4;
	buf[ACK_ADDR-1] = 5;
}

static void doRadioAnim(void* arg){
	for(;;){
		xSemaphoreTake(radioSem, portMAX_DELAY);
	}
}

static void doRpmAnim(void* arg){
	for(;;){

	}
}

static doSpeedAnim(void* arg){
	for(;;){

	}
}
