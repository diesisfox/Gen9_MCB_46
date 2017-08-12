/*
 * driverDisplay.c
 *
 *  Created on: Aug 11, 2017
 *      Author: jamesliu
 */

#include "driverDisplay.h"

#define MAX_FPS 60
#define FPS_DELAY 1000/MAX_FPS

static uint8_t buf[4][20];
static uint8_t* buff;
static SemaphoreHandle_t updateSem;
static uint8_t linesToUpdate = 0;
static OLED_HandleTypeDef* holed;
static TaskHandle_t ddTask, radioAnimTask, rpmAnimTask, speedAnimTask;

void DD_init(OLED_HandleTypeDef* holedIn){
	buff = (uint8_t*) buf;
	for(uint8_t i=0; i<sizeof(buf); i++){
		buff[i] = ' ';
	}
	updateSem = xSemaphoreCreateBinary();
	holed = holedIn;
	xTaskCreate(doDD, "DDTask", 1024, NULL, 3, &ddTask);
}

static doDD(void* pvParameters){
	for(;;){
		xSemaphoreTake(updateSem, portMAX_DELAY);

		osDelay(FPS_DELAY);
	}
}

static doRadioAnim(void* arg){

}

static doRpmAnim(void* arg){

}

static doSpeedAnim(void* arg){

}
