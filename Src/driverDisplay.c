/*
 * driverDisplay.c
 *
 *  Created on: Aug 11, 2017
 *      Author: jamesliu
 */

#include "driverDisplay.h"


// ########  ########  ######  ##          ###    ########     ###    ######## ####  #######  ##    ##  ######
// ##     ## ##       ##    ## ##         ## ##   ##     ##   ## ##      ##     ##  ##     ## ###   ## ##    ##
// ##     ## ##       ##       ##        ##   ##  ##     ##  ##   ##     ##     ##  ##     ## ####  ## ##
// ##     ## ######   ##       ##       ##     ## ########  ##     ##    ##     ##  ##     ## ## ## ##  ######
// ##     ## ##       ##       ##       ######### ##   ##   #########    ##     ##  ##     ## ##  ####       ##
// ##     ## ##       ##    ## ##       ##     ## ##    ##  ##     ##    ##     ##  ##     ## ##   ### ##    ##
// ########  ########  ######  ######## ##     ## ##     ## ##     ##    ##    ####  #######  ##    ##  ######

#define MAX_FPS 30
#define FPS_DELAY 1000/MAX_FPS
#define FILTER_SIZE 16
#define NUM_VIEWS 3
#define WHEEL_DIAMETER_MM 559
#define WHEEL_CIRC_MM 1755
#define SCREEN_HOME_DELAY 3000
#define ACK_REFRESH_PERIOD 50

static uint8_t buf[NUM_VIEWS][4][20];
static uint8_t* buf1d;
static SemaphoreHandle_t radioSem, screenSem, viewResetSem;
static TimerHandle_t viewResetTmr, ackMonitorTmr;
static uint8_t currentView = 0;
static uint8_t lastView = 0xff;
static OLED_HandleTypeDef* holed;
static TaskHandle_t ddTask;

static void doDD(void* pvParameters);
static void viewResetCb(TimerHandle_t xTimer);
static void ackMonitorCb(TimerHandle_t xTimer);

//HOME VIEW
static void home_Init();
static void home_Prep();
static void home_Render();
static void status_Render();
static void statusScrollCb(TimerHandle_t xTimer);
static uint8_t renderNextTrip(uint32_t flags, uint8_t num);
static uint8_t home_updateFlags; //||||||pow|kph
static uint32_t tripFlags; //[over|warn|en] => ||||||cellT|cellV|battC|battV
static int32_t kph_micro, pwr_milli;
static SemaphoreHandle_t bpsHBSem, adsHBSem, rdlHBSem, statusScrollSem;
static TimerHandle_t statusScrollTmr;
static int32_t volt_micro_trip, amp_micro_trip, cellt_micro_trip, cellv_micro_trip;
//VIEW 1
static void view1_Init();
static void view1_Prep();
static void view1_Render();
static uint8_t view1_updateFlags; //|||cellv_h|cellv_m|cellv_l|amp|volt
static int32_t volt_micro, amp_micro, cellv_l_micro, cellv_m_micro, cellv_h_micro;
//VIEW 2
static void view2_Init();
static void view2_Prep();
static void view2_Render();
static uint8_t view2_updateFlags; //|||cellt_h|cellt_m|cellt_l|drvTemp|motTemp
static int32_t mottemp_micro, drvtemp_micro, cellt_l_micro, cellt_m_micro, cellt_h_micro;


//  ######   ######## ##    ## ######## ########     ###    ##
// ##    ##  ##       ###   ## ##       ##     ##   ## ##   ##
// ##        ##       ####  ## ##       ##     ##  ##   ##  ##
// ##   #### ######   ## ## ## ######   ########  ##     ## ##
// ##    ##  ##       ##  #### ##       ##   ##   ######### ##
// ##    ##  ##       ##   ### ##       ##    ##  ##     ## ##
//  ######   ######## ##    ## ######## ##     ## ##     ## ########

void DD_init(OLED_HandleTypeDef* holedIn){
	buf1d = (uint8_t*) buf;
	screenSem = xSemaphoreCreateBinary();
	viewResetSem = xSemaphoreCreateBinary();
	viewResetTmr = xTimerCreate("VRT", SCREEN_HOME_DELAY, pdFALSE, 0, viewResetCb);
	ackMonitorTmr = xTimerCreate("AMT", ACK_REFRESH_PERIOD, pdTRUE, 0, ackMonitorCb);
	xTimerStart(ackMonitorTmr, portMAX_DELAY);
	holed = holedIn;
	OLED_displayOnOff(holed, 1, 0, 0);
	OLED_setFontTable(holed, 3);
	home_Init();
	view1_Init();
	view2_Init();
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
			case 2:
				if(lastView != 2) view2_Prep();
				view2_Render();
				lastView = currentView;
				break;
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

static void ackMonitorCb(TimerHandle_t xTimer){
	static uint8_t lastState;
	uint8_t currentState = readPin(ACK_BTN);
	if(lastState==1 && ~currentState==0) xSemaphoreGive(screenSem);
	lastState = currentState;
}

/*
##     ##  #######  ##     ## ########    ##     ## #### ######## ##      ##
##     ## ##     ## ###   ### ##          ##     ##  ##  ##       ##  ##  ##
##     ## ##     ## #### #### ##          ##     ##  ##  ##       ##  ##  ##
######### ##     ## ## ### ## ######      ##     ##  ##  ######   ##  ##  ##
##     ## ##     ## ##     ## ##           ##   ##   ##  ##       ##  ##  ##
##     ## ##     ## ##     ## ##            ## ##    ##  ##       ##  ##  ##
##     ##  #######  ##     ## ########       ###    #### ########  ###  ###

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
#define STAT_SCROLL_DELAY	1000
// static uint8_t home_updateFlags; //||||||pow|kph
// static uint32_t tripFlags; //[over|warn|en] => ||||||cellT|cellV|battC|battV
// static int32_t kph_micro, pwr_milli;
// static SemaphoreHandle_t bpsHBSem, adsHBSem, rdlHBSem, statusScrollSem;
// static TimerHandle_t statusScrollTmr;
#define warnStr ("\x07")
#define tripStr ("\x07\x07")
#define lowStr ("\x05@")
#define highStr ("\x06@")
#define bvStr ("BAT-V")
#define bcStr ("BAT-I")
#define cvStr ("CEL-V")
#define ctStr ("CEL-T")
#define okStr ("Systems A-OK!")

static void home_Init(){
	kph_micro = pwr_milli = tripFlags = 0;
	for(uint32_t i=0; i<80; i++){
		buf1d[i] = ' ';
	}
	bpsHBSem = xSemaphoreCreateBinary();
	adsHBSem = xSemaphoreCreateBinary();
	rdlHBSem = xSemaphoreCreateBinary();
	statusScrollSem = xSemaphoreCreateBinary();
	statusScrollTmr = xTimerCreate("SST", STAT_SCROLL_DELAY, pdTRUE, 0, statusScrollCb);
	xTimerStart(viewResetTmr, portMAX_DELAY);
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

void DD_updateSpeed(int16_t rpm){
	kph_micro = rpm * WHEEL_DIAMETER_MM * 3141593 * 60;
	home_updateFlags |= 0x01;
}

void DD_updateTrip(uint8_t* data){
	uint16_t tripID = data[0]<<8 | data[1];
	if(tripID>=0x500 && tripID<=0x51f){
		// cell temp
		int32_t temp = __REV(*(int32_t*)(data+2));
		tripFlags |= 1<<3*3;
		tripFlags &= ~(2<<3*3);
		(temp<CELLT_MID_THRESHOLD)?(tripFlags&=~(4<<3*3)):(tripFlags|=4<<3*3);
		cellt_micro_trip = temp;
	}else if(tripID<=0x008){
		// cell volt
		uint16_t temp = data[4]<<8 | data[5];
		tripFlags |= 1<<3*2;
		tripFlags &= ~(2<<3*2);
		(temp<CELLV_MID_THRESHOLD)?(tripFlags&=~(4<<3*2)):(tripFlags|=4<<3*2);
		cellv_micro_trip = temp*100;
	}else if(tripID == 0x200){
		// batt volt
		int32_t temp = __REV(*(int32_t*)(data+2));
		tripFlags |= 1<<3*0;
		tripFlags &= ~(2<<3*0);
		(temp<CATTV_MID_THRESHOLD)?(tripFlags&=~(4<<3*0)):(tripFlags|=4<<3*0);
		volt_micro_trip = temp;
	}else if(tripID == 0x201){
		// batt amp
		int32_t temp = __REV(*(int32_t*)(data+2));
		tripFlags |= 1<<3*1;
		tripFlags &= ~(2<<3*1);
		tripFlags|=4<<3*1;
		amp_micro_trip = temp;
	}
}

static void status_Render(){
	static uint8_t tripNum;
	static uint32_t lastTripFlags;
	uint32_t tempTripFlags = tripFlags;
	if(tempTripFlags){
		if((lastTripFlags != tempTripFlags) && !(tempTripFlags & tripNum<<(tripNum*3))){
			// trips updated and current trip cancelled
			tripNum = renderNextTrip(tempTripFlags, tripNum);
			xTimerReset(statusScrollTmr, portMAX_DELAY);
		}else if(lastTripFlags == tempTripFlags){
			if(xSemaphoreTake(statusScrollSem, 0)){
				// trip swapping timeout reached
				tripNum = renderNextTrip(tempTripFlags, tripNum);
			}
		}
	}else{
		strcpyN(okStr, &buf[STAT_ADDR], strsz(okStr));
		tripNum = 0;
		xTimerStop(statusScrollTmr, portMAX_DELAY);
	}
	lastTripFlags = tempTripFlags;
}

static uint8_t renderNextTrip(uint32_t flags, uint8_t num){
	uint8_t len = 0;
	for(uint8_t i=(num+1)%10; i!=num; i=(i+1)%10){
		if(flags & 1<<(i*3)){
			num = i;
			break;
		}
	}
	if(flags & 2<<(num*3)){ //warn
		strcpyN(warnStr, &buf[STAT_ADDR+len], strsz(warnStr));
		len+=strsz(warnStr);
	}else{ //trip
		strcpyN(tripStr, &buf[STAT_ADDR+len], strsz(tripStr));
		len+=strsz(tripStr);
	}
	switch (num) {
		case 0: //battV
			strcpyN(bvStr, &buf[STAT_ADDR+len], strsz(bvStr));
			len+=strsz(bvStr);
			break;
		case 1: //battC
			strcpyN(bcStr, &buf[STAT_ADDR+len], strsz(bcStr));
			len+=strsz(bcStr);
			break;
		case 2: //cellV
			strcpyN(cvStr, &buf[STAT_ADDR+len], strsz(cvStr));
			len+=strsz(cvStr);
			break;
		case 3: //cellT
			strcpyN(ctStr, &buf[STAT_ADDR+len], strsz(ctStr));
			len+=strsz(ctStr);
			break;
		default: break;
	}
	if(flags & 4<<(num*3)){ //over
		strcpyN(highStr, &buf[STAT_ADDR+len], strsz(highStr));
		len+=strsz(highStr);
	}else{ //under
		strcpyN(lowStr, &buf[STAT_ADDR+len], strsz(lowStr));
		len+=strsz(lowStr);
	}
	switch (num) {
		case 0: //battV
			len += printFixedNum(volt_micro_trip, -6, &buf[STAT_ADDR+len], STAT_MAX_LEN-len-1);
			buf[STAT_ADDR+len] = 'V';
			break;
		case 1: //battC
			len += printFixedNum(amp_micro_trip, -6, &buf[STAT_ADDR+len], STAT_MAX_LEN-len-1);
			buf[STAT_ADDR+len] = 'A';
			break;
		case 2: //cellV
			len += printFixedNum(cellv_micro_trip, -6, &buf[STAT_ADDR+len], STAT_MAX_LEN-len-1);
			buf[STAT_ADDR+len] = 'V';
			break;
		case 3: //cellT
			len += printFixedNum(cellt_micro_trip, -6, &buf[STAT_ADDR+len], STAT_MAX_LEN-len-2);
			buf[STAT_ADDR+len] = 0xb2;
			buf[STAT_ADDR+len] = 'C';
			break;
		default: break;
	}
	return num;
}

static void statusScrollCb(TimerHandle_t xTimer){
	xSemaphoreGive(statusScrollSem);
}

/*
##     ## #### ######## ##      ##       ##    ##     ########     ###    ########
##     ##  ##  ##       ##  ##  ##     ####   ####    ##     ##   ## ##      ##
##     ##  ##  ##       ##  ##  ##       ##    ##     ##     ##  ##   ##     ##
##     ##  ##  ######   ##  ##  ##       ##           ########  ##     ##    ##
 ##   ##   ##  ##       ##  ##  ##       ##    ##     ##     ## #########    ##
  ## ##    ##  ##       ##  ##  ##       ##   ####    ##     ## ##     ##    ##
   ###    #### ########  ###  ###      ######  ##     ########  ##     ##    ##

:: BATTERY MAIN STATS
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
// static uint8_t view1_updateFlags; //|||cellv_h|cellv_m|cellv_l|amp|volt
// static int32_t volt_micro, amp_micro, cellv_l_micro, cellv_m_micro, cellv_h_micro;

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
	if(volt_micro>VOLT_WARN_HIGH){
		if(tripFlags & 1<<3*0 == 0 || tripFlags & 2<<3*0){
			tripFlags |= 7<<3*0;
			volt_micro_trip = volt_micro;
		}
	}else if(volt_micro<VOLT_WARN_LOW){
		if(tripFlags & 1<<3*0 == 0 || tripFlags & 2<<3*0){
			tripFlags |= 7<<3*0;
			tripFlags &= ~(4<<3*0);
			volt_micro_trip = volt_micro;
		}
	}else{
		if(tripFlags & 1<<3*0 && tripFlags & 2<<3*0){
			tripFlags &= ~(7<<3*0);
		}
	}
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
	if(amp_micro>CUR_WARN_HIGH){
		if(tripFlags & 1<<3*1 == 0 || tripFlags & 2<<3*1){
			tripFlags |= 7<<3*1;
			amp_micro_trip = amp_micro;
		}
	}else if(amp_micro<CUR_WARN_LOW){
		if(tripFlags & 1<<3*1 == 0 || tripFlags & 2<<3*1){
			tripFlags |= 7<<3*1;
			tripFlags &= ~(4<<3*1);
			amp_micro_trip = amp_micro;
		}
	}else{
		if(tripFlags & 1<<3*1 && tripFlags & 2<<3*1){
			tripFlags &= ~(7<<3*1);
		}
	}
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
			if(temp>CELLV_WARN_HIGH){
				if(tripFlags & 1<<3*2 == 0 || tripFlags & 2<<3*2){
					tripFlags |= 7<<3*2;
					cellv_micro_trip = temp;
				}
			}else if(temp<CELLV_WARN_LOW){
				if(tripFlags & 1<<3*2 == 0 || tripFlags & 2<<3*2){
					tripFlags |= 7<<3*2;
					tripFlags &= ~(4<<3*2);
					cellv_micro_trip = temp;
				}
			}else{
				if(tripFlags & 1<<3*2 && tripFlags & 2<<3*2){
					tripFlags &= ~(7<<3*2);
				}
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


/*
##     ## #### ######## ##      ##     #######   ##     ######## ######## ##     ## ########     ##
##     ##  ##  ##       ##  ##  ##    ##     ## ####       ##    ##       ###   ### ##     ##  ####
##     ##  ##  ##       ##  ##  ##           ##  ##        ##    ##       #### #### ##     ##    ##
##     ##  ##  ######   ##  ##  ##     #######             ##    ######   ## ### ## ########     ##
 ##   ##   ##  ##       ##  ##  ##    ##         ##        ##    ##       ##     ## ##           ##
  ## ##    ##  ##       ##  ##  ##    ##        ####       ##    ##       ##     ## ##           ##
   ###    #### ########  ###  ###     #########  ##        ##    ######## ##     ## ##         ######

:: BPS TEMPERATURES
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
// static uint8_t view2_updateFlags; //|||cellt_h|cellt_m|cellt_l|drvTemp|motTemp
// static int32_t mottemp_micro, drvtemp_micro, cellt_l_micro, cellt_m_micro, cellt_h_micro;

static void view2_Init(){
	view2_updateFlags = mottemp_micro = drvtemp_micro = cellt_l_micro = cellt_m_micro = cellt_h_micro = 0;
	for(uint32_t i=160; i<240; i++){
		buf1d[i] = ' ';
	}
	buf[2][0][0] = 'T'; buf[2][0][1] = 'E'; buf[2][0][2] = 'M'; buf[2][0][3] = 'P'; buf[2][0][4] = '1'; buf[2][0][5] = ':';
	buf[2][0][6] = '['; buf[2][0][7] = 0xb2; buf[2][0][8] = 'C'; buf[2][0][9] = ']';
	buf[MOTTEMP_ICO_ADDR] = 0;
	buf[DRVTEMP_ICO_ADDR] = 1;
	buf[CELLT_ICO_ADDR0] = 2;
	buf[CELLT_ICO_ADDR1] = 3;
	buf[CELLT_L_ICO_ADDR] = 4; buf[CELLT_M_ICO_ADDR] = 5; buf[CELLT_H_ICO_ADDR] = 6;
}

static void view2_Prep(){
	OLED_setCustomChar(holed, 0, cc_motor);
	OLED_setCustomChar(holed, 1, cc_user);
	OLED_setCustomChar(holed, 2, cc_battery80);
	OLED_setCustomChar(holed, 3, cc_thermometer);
	OLED_setCustomChar(holed, 4, cc_arrowBottom);
	OLED_setCustomChar(holed, 5, cc_boxMiddle);
	OLED_setCustomChar(holed, 6, cc_arrowTop);
}

static void view2_Render(){
	uint8_t tempUF = view1_updateFlags;
	view1_updateFlags = 0;
	if(tempUF & 0x01) printFixedNum(mottemp_micro, -6, &buf[MOTTEMP_ADDR], MOTTEMP_MAX_LEN);
	if(tempUF & 0x02) printFixedNum(drvtemp_micro, -6, &buf[DRVTEMP_ADDR], DRVTEMP_MEX_LEN);
	if(tempUF & 0x04) printFixedNum(cellt_l_micro, -6, &buf[CELLT_L_ADDR], CELLT_L_MAX_LEN);
	if(tempUF & 0x08) printFixedNum(cellt_m_micro, -6, &buf[CELLT_M_ADDR], CELLT_M_MAX_LEN);
	if(tempUF & 0x10) printFixedNum(cellt_h_micro, -6, &buf[CELLT_H_ADDR], CELLT_H_MAX_LEN);
}

void DD_updateMotTemp(int32_t tmp){
	mottemp_micro = tmp;
	view1_updateFlags |= 0x01;
}

void DD_updateDrvTemp(int32_t tmp){
	drvtemp_micro = tmp;
	view1_updateFlags |= 0x02;
}

void DD_updateCellT(uint8_t* data, uint8_t index){
	static int32_t temps[32];
	uint8_t divisor = 0;
	temps[index*2] = __REV(*(int32_t*)(data));
	temps[index*2+1] = __REV(*(int32_t*)(data+4));
	cellt_m_micro = cellt_h_micro = 0;
	cellt_l_micro = 99999999;
	for(uint8_t i=0; i<32; i++){
		if(temps[i]){
			cellt_m_micro+=temps[i];
			divisor++;
			if(temps[i] > cellt_h_micro){
				cellt_h_micro = temps[i];
				view2_updateFlags |= 0x10;
			}if(temps[i] < cellt_l_micro){
				cellt_l_micro = temps[i];
				view2_updateFlags |= 0x04;
			}
			if(temps[i]>CELLT_WARN_HIGH){
				if(tripFlags & 1<<3*3 == 0 || tripFlags & 2<<3*3){
					tripFlags |= 7<<3*3;
					cellt_micro_trip = temps[i];
				}
			}else if(temps[i]<CELLT_WARN_LOW){
				if(tripFlags & 1<<3*3 == 0 || tripFlags & 2<<3*3){
					tripFlags |= 7<<3*3;
					tripFlags &= ~(4<<3*3);
					cellt_micro_trip = temps[i];
				}
			}else{
				if(tripFlags & 1<<3*3 && tripFlags & 2<<3*3){
					tripFlags &= ~(7<<3*3);
				}
			}
		}
	}
	cellt_m_micro /= divisor;
	view2_updateFlags |= 0x08;
	if(cellt_l_micro > cellt_h_micro){
		cellt_l_micro = cellt_h_micro;
		view2_updateFlags |= 0x04;
	}
}

/*
##     ## #### ######## ##      ##     #######   ##     ########  ########  ########
##     ##  ##  ##       ##  ##  ##    ##     ## ####    ##     ## ##     ##    ##
##     ##  ##  ##       ##  ##  ##           ##  ##     ##     ## ##     ##    ##
##     ##  ##  ######   ##  ##  ##     #######          ########  ########     ##
 ##   ##   ##  ##       ##  ##  ##           ##  ##     ##        ##           ##
  ## ##    ##  ##       ##  ##  ##    ##     ## ####    ##        ##           ##
   ###    #### ########  ###  ###      #######   ##     ##        ##           ##

:: PPT STATS
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
