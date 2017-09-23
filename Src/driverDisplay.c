/*
 * driverDisplay.c
 *
 *  Created on: Aug 11, 2017
 *      Author: jamesliu
 */

#include "driverDisplay.h"

#define MAX_FPS 30
#define FPS_DELAY 1000/MAX_FPS
#define FILTER_SIZE 16
#define NUM_VIEWS 2
#define WHEEL_DIAMETER_MM 559
#define WHEEL_CIRC_MM 1755
#define SCREEN_HOME_DELAY 3000

static uint8_t buf[NUM_VIEWS][4][20];
static uint8_t* buf1d;
static SemaphoreHandle_t updateSem, radioSem, screenSem, viewResetSem;
static TimerHandle_t viewResetTmr;
static uint8_t currentView = 0;
static uint8_t lastView = 0xff;
static OLED_HandleTypeDef* holed;
static TaskHandle_t ddTask;

static void doDD(void* pvParameters);
static void viewResetCb(TimerHandle_t xTimer);


//HOME VIEW
static void home_Init();
static void home_Prep();
static void home_Render();
static void status_Render();
//VIEW 1
static void view1_Init();
static void view1_Prep();
static void view1_Render();


void DD_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(screenSem) xSemaphoreGiveFromISR(screenSem, NULL);
}

void DD_init(OLED_HandleTypeDef* holedIn){
	buf1d = (uint8_t*) buf;
//	for(uint8_t i=0; i<sizeof(buf)/sizeof(uint32_t); i+=4){
//		*(uint32_t*)(&buf1d[i]) = 0x23202320;
//	}
	screenSem = xSemaphoreCreateBinary();
	viewResetSem = xSemaphoreCreateBinary();
	viewResetTmr = xTimerCreate("VRT", SCREEN_HOME_DELAY, pdFALSE, 0, viewResetCb);
	holed = holedIn;
	OLED_displayOnOff(holed, 1, 0, 0);
	OLED_setFontTable(holed, 3);
	home_Init();
	view1_Init();
	// view2_Init();
	xTaskCreate(doDD, "DDTask", 1024, NULL, 3, &ddTask);
}

static void doDD(void* pvParameters){
	osDelay(1000);
	for(;;){
		if(xSemaphoreTake(screenSem, 0)){
			currentView++;
			currentView %= NUM_VIEWS;
			if(currentView != 0) xTimerReset(viewResetTmr, portMAX_DELAY);
		}
		if(xSemaphoreTake(viewResetSem, 0)) currentView = 0;
		switch(currentView){
			case 0:
				if(lastView != 0) home_Prep();
				home_Render();
				lastView = currentView;
				break;
			case 1:
				if(lastView != 1) view1_Prep();
				view1_Render();
				lastView = currentView;
				break;
			// case 2:
			// 	if(lastView != 2) view2_Prep();
			// 	view2_Render();
			// 	lastView = currentView;
			// 	break;
			default:
				break;
		}
		OLED_writeFrame(holed, &buf[currentView][0][0]);
		osDelay(FPS_DELAY);
	}
}

static void viewResetCb(TimerHandle_t xTimer){
	xSemaphoreGive(viewResetSem);
}


/* HOME VIEW
┌────────────────────┐
│Aaaa.aAAA•Bbbbbb.bbB│
│--------------------│
│cccccccccccccccc•DEF│
│--------------------│
└────────────────────┘
A:speed; B:power; C:trips&status; D:BPS HB; E:ADS HB; F:radio HB;
*/
#define KPH_ICO_ADDR	0][0][0
#define KPH_ADDR 		0][0][1
#define KPH_MAX_LEN		5
#define PWR_ICO_ADDR	0][0][10
#define PWR_ADDR 		0][0][11
#define PWR_MAX_LEN		8
#define STAT_ADDR		0][2][0
#define STAT_MAX_LEN	16
#define BPS_HB_ADDR		0][2][17
#define ADS_HB_ADDR		0][2][18
#define RDL_HB_ADDR		0][2][19
static uint8_t home_updateFlags; //||||||pow|kph
static uint32_t tripFlags; //[over|warn|en] => ||||||cellT|cellV|battC|battV
static int32_t kph_micro, pwr_milli;
static uint8_t* stat_str;
static SemaphoreHandle_t bpsHBSem, adsHBSem, rdlHBSem, statusScrollSem;
static uint8_t* warnStr = "\x07WARN:";
static uint8_t* tripStr = "\x07\x07TRIP:";
static uint8_t* lowStr = "\x05@";
static uint8_t* highStr = "\x06@";
static uint8_t* bvStr = "Battery Voltage";
static uint8_t* bcStr = "Battery Current";
static uint8_t* cvStr = "Cell Voltage";
static uint8_t* ctStr = "Cell temperature";
static uint8_t* okStr = "Systems A-OK!";

static void home_Init(){
	kph_micro = pwr_milli = tripFlags = 0;
	for(uint32_t i=0; i<80; i++){
		buf1d[i] = ' ';
	}
	bpsHBSem = xSemaphoreCreateBinary();
	adsHBSem = xSemaphoreCreateBinary();
	rdlHBSem = xSemaphoreCreateBinary();
	statusScrollSem = xSemaphoreCreateBinary();
	buf[KPH_ICO_ADDR] = 0;
	buf[PWR_ICO_ADDR] = 1;
	buf[BPS_HB_ADDR] = 2;
	buf[ADS_HB_ADDR] = 3;
	buf[RDL_HB_ADDR] = 4;
}

static void home_Prep(){
	OLED_setCustomChar(holed, 0, cc_speed0);
	OLED_setCustomChar(holed, 1, cc_plug);
	OLED_setCustomChar(holed, 2, cc_blank);
	OLED_setCustomChar(holed, 3, cc_blank);
	OLED_setCustomChar(holed, 4, cc_blank);
	OLED_setCustomChar(holed, 5, cc_arrowBottom);
	OLED_setCustomChar(holed, 6, cc_arrowTop);
	OLED_setCustomChar(holed, 7, cc_exclamation);
}

static void home_Render(){
	uint8_t tempUF = home_updateFlags;
	home_updateFlags = 0;
	uint8_t len;
	if(tempUF & 0x01){
		len = printFixedNum(kph_micro, -6, &buf[KPH_ADDR], KPH_MAX_LEN);
		buf[KPH_ADDR+len+0] = 'k';
		buf[KPH_ADDR+len+1] = 'p';
		buf[KPH_ADDR+len+2] = 'h';
	}
	if(tempUF & 0x02) buf[PWR_ADDR+printFixedNum(pwr_milli, -3, &buf[PWR_ADDR], PWR_MAX_LEN)] = 'W';
	status_Render();
}

static void status_Render(){
	static uint8_t tripNum;
	if(tripFlags){
		for(uint8_t i=0; i<10; i++){
			if(tripFlags & (1<<(i*3))){

			}
		}
	}else{
		strcpyN(okStr, &buf[STAT_ADDR], STAT_MAX_LEN);
	}
}

void DD_updateSpeed(int32_t rpm){
	kph_micro = rpm * WHEEL_DIAMETER_MM * 3141593 * 60;
	home_updateFlags |= 0x01;
}


/* VIEW 1: BATTERY MAIN STATS
┌────────────────────┐
│BAT:•Aaaa.aA•Bbb.bbB│
│--------------------│
│FGCc.cccDd.dddEe.eee│
│--------------------│
└────────────────────┘
A:voltage; B:current; C:low cell volt; D: avg cell volt; E: high cell volt; F:🔋; G:⚡️;
*/
#define VOLT_ICO_ADDR		1][0][5
#define VOLT_ADDR			1][0][6
#define VOLT_MAX_LEN		5
#define CUR_ICO_ADDR		1][0][13
#define CUR_ADDR			1][0][14
#define CUR_MAX_LEN			5
#define CELLV_ICO_ADDR0		1][2][0
#define CELLV_ICO_ADDR1		1][2][1
#define CELLV_L_ICO_ADDR	1][2][2
#define CELLV_L_ADDR		1][2][3
#define CELLV_L_MAX_LEN		5
#define CELLV_M_ICO_ADDR	1][2][8
#define CELLV_M_ADDR		1][2][9
#define CELLV_M_MAX_LEN		5
#define CELLV_H_ICO_ADDR	1][2][14
#define CELLV_H_ADDR		1][2][15
#define CELLV_H_MAX_LEN		5
static uint8_t view1_updateFlags; //|||cellv_h|cellv_m|cellv_l|amp|volt
static int32_t volt_micro, amp_micro, cellv_l_micro, cellv_m_micro, cellv_h_micro;

static void view1_Init(){
	view1_updateFlags = volt_micro = amp_micro = cellv_l_micro = cellv_m_micro = cellv_h_micro = 0;
	for(uint32_t i=80; i<160; i++){
		buf1d[i] = ' ';
	}
	buf[1][0][0] = 'B'; buf[1][0][1] = 'A'; buf[1][0][2] = 'T'; buf[1][0][3] = ':';
	buf[VOLT_ICO_ADDR] = 0;
	buf[CUR_ICO_ADDR] = 1;
	buf[CELLV_ICO_ADDR0] = 3;
	buf[CELLV_ICO_ADDR1] = 0;
	buf[CELLV_L_ICO_ADDR] = 4; buf[CELLV_M_ICO_ADDR] = 6; buf[CELLV_H_ICO_ADDR] = 5;
}

static void view1_Prep(){
	OLED_setCustomChar(holed, 0, cc_lightning);
	OLED_setCustomChar(holed, 1, cc_plug);
	OLED_setCustomChar(holed, 3, cc_battery80);
	OLED_setCustomChar(holed, 4, cc_arrowBottom);
	OLED_setCustomChar(holed, 5, cc_arrowTop);
	OLED_setCustomChar(holed, 6, cc_boxMiddle);
}

static void view1_Render(){
	uint8_t tempUF = view1_updateFlags;
	view1_updateFlags = 0;
	if(tempUF & 0x01) buf[VOLT_ADDR+printFixedNum(volt_micro, -6, &buf[VOLT_ADDR], VOLT_MAX_LEN)] = 'V';
	if(tempUF & 0x02) buf[CUR_ADDR+printFixedNum(amp_micro, -6, &buf[CUR_ADDR], CUR_MAX_LEN)] = 'A';
	if(tempUF & 0x04) printFixedNum(cellv_l_micro, -6, &buf[CELLV_L_ADDR], CELLV_L_MAX_LEN);
	if(tempUF & 0x08) printFixedNum(cellv_m_micro, -6, &buf[CELLV_M_ADDR], CELLV_M_MAX_LEN);
	if(tempUF & 0x10) printFixedNum(cellv_h_micro, -6, &buf[CELLV_H_ADDR], CELLV_H_MAX_LEN);
}

void DD_updateVolt(int32_t volt){
	static int32_t voltBuf[FILTER_SIZE];
	static uint8_t voltBufInd, voltBufFilled;
	voltBuf[voltBufInd] = volt;
	voltBufInd++;
	if(voltBufInd>=FILTER_SIZE){
		voltBufFilled = 1;
		voltBufInd = 0;
	}
	volt_micro = 0;
	for(uint8_t i=0; i<(voltBufFilled?16:voltBufInd); i++){
		volt_micro += voltBuf[i]/(voltBufFilled?16:voltBufInd);
	}
	pwr_milli = (volt_micro/1000)*(amp_micro/1000)/1000;
	view1_updateFlags |= 0x01;
	home_updateFlags |= 0x02;
}

void DD_updateAmp(int32_t amp){
	static int32_t ampBuf[FILTER_SIZE];
	static uint8_t ampBufInd, ampBufFilled;
	ampBuf[ampBufInd] = amp;
	ampBufInd++;
	if(ampBufInd>=FILTER_SIZE){
		ampBufFilled = 1;
		ampBufInd = 0;
	}
	amp_micro = 0;
	for(uint8_t i=0; i<(ampBufFilled?16:ampBufInd); i++){
		amp_micro += ampBuf[i]/(ampBufFilled?16:ampBufInd);
	}
	pwr_milli = (volt_micro/1000)*(amp_micro/1000)/1000;
	view1_updateFlags |= 0x02;
	home_updateFlags |= 0x02;
}

void DD_updateCellV(uint8_t* data, uint8_t index){
	static uint16_t voltages[36];
	uint8_t divisor = 0;
	for(uint8_t i=0; i<4; i++){
		voltages[index*4+i] = data[i*2]<<8 | data[i*2+1];
	}
	cellv_m_micro = cellv_h_micro = 0;
	cellv_l_micro = 99999999;
	for(uint8_t i=0; i<36; i++){
		int32_t temp = voltages[i] * 100;
		if(temp > 1000){
			cellv_m_micro+=temp;
			divisor++;
			if(temp > cellv_h_micro){
				cellv_h_micro = temp;
				view1_updateFlags |= 0x10;
			}if(temp < cellv_l_micro){
				cellv_l_micro = temp;
				view1_updateFlags |= 0x04;
			}
		}
	}
	cellv_m_micro /= divisor;
	view1_updateFlags |= 0x08;
	if(cellv_l_micro > cellv_h_micro){
		cellv_l_micro = cellv_h_micro;
		view1_updateFlags |= 0x04;
	}
}


/* VIEW 2: BPS TEMPERATURES
┌────────────────────┐
│TEMP1:[°C]•AaaaBbb.b│
│--------------------│
│FGCcc.ccDdd.ddEee.ee│
│--------------------│
└────────────────────┘
A:mot temp; B:driver temp; C:low cel temp; D:avg cel temp; E:high cel temp; F:🔋; G:🌡;
*/
#define MOTTEMP_ICO_ADDR	2][0][11
#define MOTTEMP_ADDR		2][0][12
#define MOTTEMP_MAX_LEN		3
#define DRVTEMP_ICO_ADDR	2][0][15
#define DRVTEMP_ADDR		2][0][16
#define DRVTEMP_MEX_LEN		4
#define CELLT_ICO_ADDR0		1][2][0
#define CELLT_ICO_ADDR1		1][2][1
#define CELLT_L_ICO_ADDR	1][2][2
#define CELLT_L_ADDR		1][2][3
#define CELLT_L_MAX_LEN		5
#define CELLT_M_ICO_ADDR	1][2][8
#define CELLT_M_ADDR		1][2][9
#define CELLT_M_MAX_LEN		5
#define CELLT_H_ICO_ADDR	1][2][14
#define CELLT_H_ADDR		1][2][15
#define CELLT_H_MAX_LEN		5
static int32_t mottemp_milli, drvtemp_milli, cellt_l_milli, cellt_m_milli, cellt_h_milli;


/* VIEW 3: PPT STATS
┌────────────────────┐
│PPT:•D[°C]•AaaBbbCcc│
│--------------------│
│H[W]•Eeee•Ffff•Gggg•│
│--------------------│
└────────────────────┘
A:temp A; B:temp B; C: temp C; D:🌡; E:power A; F:power B; G:power C; H:🔌;
*/


/* VIEW X0
┌────────────────────┐
│AaaaaAAA•Bbbb.bbB••E│
│Ggg.gGG•HhhH•IiiiI••│
│Ccc.ccC•Dddddd.ddD•F│
│JjjjjjjjjjjjjjjjjjjJ│
└────────────────────┘
A: rpm/speed; B: voltage; C: current; D:power; E:radio HB; F:ack;
G: driver temp; H: SoC; I: avg cell volt; J: trips&status;
*/


/* VIEW X1
┌────────────────────┐
│YYYYYY SPEED: ZZZZZZ│
│YYYYYY-999.99 ZZZZZZ│
│YYYYYY POWER: ZZZZZZ│
│YYYYYY-9999.99ZZZZZZ│
└────────────────────┘
Y: left signal zone; Z: right signal zone;
*/
