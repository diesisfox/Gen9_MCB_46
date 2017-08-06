/*
 * oled2004.h
 *
 *  Created on: Apr 12, 2017
 *      Author: jamesliu
 */

#ifndef OLED2004_H_
#define OLED2004_H_


#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define OLED_BUFFER_SIZE (sizeof(uint8_t[4][20]))
#define OLED_CMD_BUF_LEN 16
#define OLED_FRAME_BUF_LEN 8


#define CLEAR_DISPLAY	0b00000001

#define RETURN_HOME		0b00000010


/* Entry Mode Set
 * c = Increment/Decrement, s = Shift Entire Display
 *
 * When c = [1/0], the DDRAM or CGRAM Address with automatically [increment/decrement]
 * when a character code is written into or read from the DDRAM or CGRAM.
 * The auto-[increment/decrement] will move the cursor one character space to the [right/left].
 *
 * When s = 1, the entire display is shifted to the right (when c = 0) or left (when c = 1).
 */
#define ENTRY_MODE(c,s)	(0b00000100|c|s)
#define CURS_INC	0b00000010
#define CURS_DEC	0b00000000
#define TEXT_SHIFT	0b00000001
#define TEXT_STAY	0b00000000


/* Display ON/OFF
 * d = Display ON/OFF, c = Cursor ON/OFF, b = Blinking Cursor
 *
 * When d = 1, the display is turned ON. When d = 0, the display is turned OFF.
 * Contents in DDRAM are not changed.
 *
 * When c = 1, the cursor is displayed. The cursor is displayed as 5 dots on the 8th line of a character.
 * When c = 0, the cursor is OFF.
 *
 * When b = 1, the entire character specified by the cursor blinks at a speed of 409.6ms intervals.
 * When b = 0, the character does not blink, the cursor remains on.
 */
#define DISP_ON_OFF(d,c,b)	(0b00001000|d|c|b)
#define DISP_ON		0b00000100
#define CURS_ON		0b00000010
#define BLINK_ON	0b00000001
#define DISP_OFF	0b00000000
#define CURS_OFF	0b00000000
#define BLINK_OFF	0b00000000


/* Cursor/Display Shift
 *
 * When the display is shifted repeatedly, each line moves only horizontally.
 * The second line display does not shift into the first line.
 * The Address Counter does not change during a Display Shift.
 */
#define SHIFT_CURS_LEFT		0b00010000	//Shifts the cursor position to the left (AC is decremented by 1).
#define SHIFT_CURS_RIGHT	0b00010100	//Shifts the cursor position to the right (AC is incremented by 1).
#define SHIFT_DISP_LEFT		0b00011000	//Shifts the entire display to the left. The cursor follows the display shift.
#define SHIFT_DISP_RIGHT	0b00011100	//Shifts the entire display to the right. The cursor follows the display shift.


/* Function Set
 * l = data length (use 8), f = font bank
 *
 * When l = 1, the data is sent or received in 8-bit length via DB7...DB0.
 * When l = 0, the data is sent or received in 4-bit length via DB7...DB4.
 * When the 4-bit data length is used, the data must be sent or received in two consecutive writes/reads
 * to combine the data into full 8-bits.
 */
#define FUNCTION_SET(l,f)	(0b00101000|l|f)
#define DL_8	0b00010000
#define DL_4	0b00000000
#define ENG_JAP	0b00000000
#define EURO_1	0b00000001
#define ENG_RUS	0b00000010
#define EURO_2	0b00000011


/* Set CGRAM Address
 * c = char num, r = row num
 */
#define SET_CGRAM_ADDR(x)	(0b01000000|((x)&0x3f))


/* Set DDRAM Address
 * r = row, c = column
 *
 * Row 1 = Address 0x00 through 0x13
 * Row 2 = Address 0x40 through 0x53
 * Row 3 = Address 0x14 through 0x27
 * Row 4 = Address 0x54 through 0x67
 */
#define SET_DDRAM_ADDR(a)	(0b10000000|((a)&0x7f))


/* Read Busy Flag and Address Counter
 *
 * When BF = “1”, the controller is busy and the next instruction will be ignored.
 * When BF = “0”, the controller is not busy and is ready to accept instructions.
 *
 * 0b01[bf][AC6...0]
 */


/* Write Data to CGRAM or DDRAM
 *
 * 0b10[d7...d0]
 */

/* Read Data from CGRAM or DDRAM
 *
 * 0b11[d7...d0]
 */


typedef struct{
	uint8_t	cursorShiftMode		:1;
	uint8_t	textShiftMode		:1;
	uint8_t	displayEnable		:1;
	uint8_t	cursorEnable		:1;
	uint8_t	CursorBlinkEnable	:1;
	uint8_t	displayShiftMode	:2;
	uint8_t	dataLength			:1;
	uint8_t	fontBank			:2;
}OLED_ConfigTypeDef;

typedef struct{
	uint8_t	r	:4;	//row
	uint8_t	c	:4;	//char or col
}byteAddr_t;

typedef uint8_t OLED_Glyph_t[8][10]; //used: [8][5]. it's actually quite a waste, putting 40 bits into 80 bytes
typedef uint8_t OLED_Frame_t[4][20];

typedef struct{
	SPI_HandleTypeDef*	hspi;
	GPIO_TypeDef*		csPort;
	uint16_t			csPin;
	SemaphoreHandle_t	txMtx;
	SemaphoreHandle_t	rxSem;
	TaskHandle_t		task;
	QueueHandle_t		cmdQ;
	QueueHandle_t		dataQ;
	OLED_ConfigTypeDef	config;
	byteAddr_t			cursorAddr;
	byteAddr_t			CGRAMAddr;
	uint8_t				DDRAMSel	:1; // 0 = cgram selected, 1 = ddram selected
	uint8_t				awaitingCb	:1;
	uint8_t				busyFlag	:1;
}OLED_HandleTypeDef;

int OLED_init(OLED_HandleTypeDef* holed, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);
void OLED_clearDisplay(OLED_HandleTypeDef* holed);
void OLED_returnHome(OLED_HandleTypeDef* holed);
void OLED_entryModeSet(OLED_HandleTypeDef* holed, uint8_t id, uint8_t s);
void OLED_displayOnOff(OLED_HandleTypeDef* holed, uint8_t d, uint8_t c, uint8_t b);
void OLED_cursorDisplayShift(OLED_HandleTypeDef* holed, uint8_t mode);
void OLED_setDataLength(OLED_HandleTypeDef* holed, uint8_t dl);
void OLED_setFontTable(OLED_HandleTypeDef* holed, uint8_t ft);
void OLED_setCursor(OLED_HandleTypeDef* holed, uint8_t r, uint8_t c);
void OLED_writeFrame(OLED_HandleTypeDef* holed, uint8_t* buf);
void OLED_writeLines(OLED_HandleTypeDef* holed, uint8_t* buf, uint8_t lines);


#endif /* OLED2004_H_ */
