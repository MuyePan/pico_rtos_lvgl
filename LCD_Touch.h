/*****************************************************************************
 * | File      	:	LCD_Touch.h
 * | Author      :   Waveshare team
 * | Function    :	LCD Touch Pad Driver and Draw
 * | Info        :
 *   Image scanning
 *      Please use progressive scanning to generate images or fonts
 *----------------
 * |	This version:   V1.0
 * | Date        :   2017-08-16
 * | Info        :   Basic version
 *
 ******************************************************************************/
#ifndef __LCD_TOUCH_H_
#define __LCD_TOUCH_H_

#include "DEV_Config.h"
#include "LCD_Driver.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/float.h"

#define TP_PRESS_DOWN 0x80
#define TP_PRESSED 0x40

// Touch screen structure
typedef struct
{
	POINT Xpoint0;
	POINT Ypoint0;
	POINT Xpoint;
	POINT Ypoint;
	uint8_t chStatus;
	uint8_t chType;
	int16_t iXoff;
	int16_t iYoff;
	float fXfac;
	float fYfac;
	//Select the coordinates of the XPT2046 touch \
	  screen relative to what scan direction
	LCD_SCAN_DIR TP_Scan_Dir;
} TP_DEV;

void TP_GetAdFac(void);
void TP_Init(LCD_SCAN_DIR Lcd_ScanDir);
void TP_Scan(uint32_t *x, uint32_t *y);

#endif
