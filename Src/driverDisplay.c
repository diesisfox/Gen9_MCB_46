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
#define NUM_VIEWS 3

static uint8_t buf[NUM_VIEWS][4][20];
static uint8_t* buf1d;
static SemaphoreHandle_t updateSem, radioSem, screenSem;
static uint8_t currentView = 0;
static OLED_HandleTypeDef* holed;
static TaskHandle_t ddTask, radioAnimTask, rpmAnimTask, speedAnimTask;
static uint16_t rpm = 0;
// static int32_t voltBuf[FILTER_SIZE], crtBuf[FILTER_SIZE], powBuf[FILTER_SIZE];
// static uint8_t voltBufInd = 0, crtBufInd = 0, powBufInd = 0;
static uint8_t bufFilled = 0; //?|?|?|?|?|p|i|v

static void doDD(void* pvParameters);
static void doRadioAnim(void* arg);
static void doRpmAnim(void* arg);
static void doSpeedAnim(void* arg);
static void setupIcons();
static void writeIcons();
static void setIconChars();

void DD_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(screenSem) xSemaphoreGiveFromISR(screenSem, NULL);
}

void DD_init(OLED_HandleTypeDef* holedIn){
	buf1d = (uint8_t*) buf;
	for(uint8_t i=0; i<sizeof(buf)/sizeof(uint32_t); i+=4;){
		*(uint32_t*)(&buf1d[i]) = 0x20202020;
	}
	updateSem = xSemaphoreCreateBinary();
	radioSem = xSemaphoreCreateBinary();
	screenSem = xSemaphoreCreateBinary();
	holed = holedIn;
	OLED_displayOnOff(holed, 1, 0, 0);
	OLED_setFontTable(holed, EURO_2);
	home_Init();
	view1_Init();
	view2_Init();
	xTaskCreate(doDD, "DDTask", 1024, NULL, 3, &ddTask);
	// xTaskCreate(doRadioAnim, "RadioAnimTask", 512, NULL, 2, &radioAnimTask);
	// xTaskCreate(doRpmAnim, "RpmAnimTask", 512, NULL, 2, &rpmAnimTask);
}

static void doDD(void* pvParameters){
	uint8_t lastView = 0xff;
	for(;;){
		switch(currentView){
			case 0:
				if(lastView != 0) home_Prep();
				home_Render();
				lastView = 0;
				break;
			case 1:
				if(lastView != 1) view1_Prep();
				view1_Render();
				lastView = 1;
				break;
			case 2:
				if(lastView != 2) view2_Prep();
				view2_Render();
				lastView = 2;
				break;
			default:
				break;
		}
		xSemaphoreTake(updateSem, portMAX_DELAY);
		OLED_writeFrame(holed, buf1d);
		osDelay(FPS_DELAY);
	}
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
#define PWR_ADDR 		0][0][9
#define PWR_MAX_LEN		8
#define STAT_ADDR		0][2][0
#define STAT_MAX_LEN	16
#define BPS_HB_ADDR		0][2][17
#define ADS_HB_ADDR		0][2][18
#define RDL_HB_ADDR		0][2][19
static int32_t kph_milli, pwr_milli;
static uint8_t* stat_str;
static SemaphoreHandle_t bpsHBSem, adsHBSem, rdlHBSem, statusScrollSem;
static uint8_t* warnStr = "\x07WARN:", tripStr = "\x07\x07TRIP:", lowStr = "\x05@", highStr = "\x06@";
static uint8_t* bvStr = "Battery Voltage", bcStr = "Battery Current", cvStr = "Cell Voltage", ctStr = "Cell temperature";

static void home_Init(){
	kph_milli = pwr_milli = 0;
	for(uint i=0; i<80; i++){
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

static void view1_Init(){
	view1_updateFlags = volt_micro = amp_micro = cellv_l_micro = cellv_m_micro = cellv_h_micro = 0;

	buf[1][0][0] = 'B'; buf[1][0][1] = 'A'; buf[1][0][2] = 'T'; buf[1][0][3] = ':';
	buf[VOLT_ICO_ADDR] = 0;
	buf[CUR_ICO_ADDR] = 1;
	buf[CELLV_ICO_ADDR0] = 3;
	buf[CELLV_ICO_ADDR1] = 0;
	buf[CELLV_L_ICO_ADDR] = 4; buf[CELLV_M_ICO_ADDR] = 5; buf[CELLV_H_ICO_ADDR] = 6;
}

static void view1_Prep(){
	OLED_setCustomChar(holed, 0, cc_lightning);
	OLED_setCustomChar(holed, 1, cc_plug);
	OLED_setCustomChar(holed, 3, cc_battery80);
	OLED_setCustomChar(holed, 4, cc_arrowBottom);
	OLED_setCustomChar(holed, 5, cc_arrowTop);
	OLED_setCustomChar(holed, 6, cc_boxMiddle);
}

static void page1_Render(){
	buf[VOLT_ADDR+printFixedNum(volt_micro, -6, &buf[VOLT_ADDR], VOLT_MAX_LEN)] = 'V';
	buf[CUR_ADDR+printFixedNum(amp_micro, -6, &buf[CUR_ADDR], CUR_MAX_LEN)] = 'A';
	printFixedNum(cellv_l_micro, -6, &buf[CELLV_L_ADDR], CELLV_L_MAX_LEN);
	printFixedNum(cellv_m_micro, -6, &buf[CELLV_M_ADDR], CELLV_M_MAX_LEN);
	printFixedNum(cellv_h_micro, -6, &buf[CELLV_H_ADDR], CELLV_H_MAX_LEN);
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
static uint8_t view1_updateFlags; //|||||pow|amp|volt
static int32_t volt_micro, amp_micro, cellv_l_micro, cellv_m_micro, cellv_h_micro;
static void view1_Init();
static void view1_Prep();
static void view1_Render();

static void view1_Init(){
	view1_updateFlags = volt_micro = amp_micro = cellv_l_micro = cellv_m_micro = cellv_h_micro = 0;
	for(uint i=80; i<160; i++){
		buf1d[i] = ' ';
	}
	buf[1][0][0] = 'B'; buf[1][0][1] = 'A'; buf[1][0][2] = 'T'; buf[1][0][3] = ':';
	buf[VOLT_ICO_ADDR] = 0;
	buf[CUR_ICO_ADDR] = 1;
	buf[CELLV_ICO_ADDR0] = 3;
	buf[CELLV_ICO_ADDR1] = 0;
	buf[CELLV_L_ICO_ADDR] = 4; buf[CELLV_M_ICO_ADDR] = 5; buf[CELLV_H_ICO_ADDR] = 6;
}

static void view1_Prep(){
	OLED_setCustomChar(holed, 0, cc_lightning);
	OLED_setCustomChar(holed, 1, cc_plug);
	OLED_setCustomChar(holed, 3, cc_battery80);
	OLED_setCustomChar(holed, 4, cc_arrowBottom);
	OLED_setCustomChar(holed, 5, cc_arrowTop);
	OLED_setCustomChar(holed, 6, cc_boxMiddle);
}

static void page1_Render(){
	buf[VOLT_ADDR+printFixedNum(volt_micro, -6, &buf[VOLT_ADDR], VOLT_MAX_LEN)] = 'V';
	buf[CUR_ADDR+printFixedNum(amp_micro, -6, &buf[CUR_ADDR], CUR_MAX_LEN)] = 'A';
	printFixedNum(cellv_l_micro, -6, &buf[CELLV_L_ADDR], CELLV_L_MAX_LEN);
	printFixedNum(cellv_m_micro, -6, &buf[CELLV_M_ADDR], CELLV_M_MAX_LEN);
	printFixedNum(cellv_h_micro, -6, &buf[CELLV_H_ADDR], CELLV_H_MAX_LEN);
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
	view1_updateFlags |= 0x05;
}

void DD_updateAmp(int32_t amp){
	static int32_t ampBuf[FILTER_SIZE];
	static uint8_t ampBufInd, voltBufFilled;
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
	view1_updateFlags |= 0x06;
}

void DD_updateCellV(uint8_t* data; uint8_t index){
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
			if(temp > cellv_h_micro) cellv_h_micro = temp;
			if(temp < cellv_l_micro) cellv_l_micro = temp;
		}
	}
	cellv_m_micro /= divisor;
	if(cellv_l_micro > cellv_h_micro) cellv_l_micro = cellv_h_micro;
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



void DD_updateRPM(uint32_t rpmIn){
	rpm = rpmIn/1000;
	uint8_t len = 0;
	for(uint8_t i=0; i<7; i++){
		buf[KPH_ADDR+i] = ' ';
	}
	len += intToDec(rpm, &(buf[KPH_ADDR]));
	applyStr(&(buf[KPH_ADDR+len]), "kph", 3);
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

void DD_updateDriverTemp(int32_t uC){
	uint8_t len = 0;
	for(uint8_t i=0; i<6; i++){
		buf[DRVTEMP_ADDR+i] = ' ';
	}
	if(uC<0){
		buf[DRVTEMP_ADDR+len] = '-';
		len++;
		uC = -uC;
	}
	int32_t uC1 = uC/1000000;

	len += intToDec(uC1, &(buf[DRVTEMP_ADDR+len]));
	uC1 = (uC%1000000)/100000;

	if(uC1){
		buf[DRVTEMP_ADDR+len] = '.';
		len++;
		len += intToDec(uC1, &(buf[DRVTEMP_ADDR+len]));
	}
	buf[DRVTEMP_ADDR+len] = 0xdf;
	buf[DRVTEMP_ADDR+len+1] = 'C';
	if(updateSem) xSemaphoreGive(updateSem);
}

static void writeIcons(){
	buf[KPH_ADDR-1] = 0;
	buf[VOLT_ADDR-1] = 1;
	buf[CRT_ADDR-1] = 2;
	buf[PWR_ADDR-1] = 3;
	buf[RDL_HB_ADDR-1] = 4;
}

static void setIconChars(){
	OLED_setCustomChar(holed, 0, cc_rpm0);
	OLED_setCustomChar(holed, 1, cc_lightning);
	OLED_setCustomChar(holed, 2, cc_plug);
	OLED_setCustomChar(holed, 3, cc_powerOn);
	OLED_setCustomChar(holed, 4, cc_blank);
	OLED_setCustomChar(holed, 5, cc_blank);
}

static void setupIcons(){
	setIconChars();
	writeIcons();
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
		}
		delay = 2500/rpm;
		if(delay > 500) delay = 500;
		frame = (frame+1)%4;
		OLED_setCustomChar(holed, 0, frames[frame]);
		osDelay(delay);
	}
}

static void doSpeedAnim(void* arg){
	for(;;){
		osDelay(10000);
	}
}
