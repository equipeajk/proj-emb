/*
 * lcd44780_LP.h
 *		Header file for the lcd44780_LP.c
 *		Version 1.00
 *      Author: NYH
 *      Reference: Robot Head to Toe Vol. 11 - pg 35-36
 */

#ifndef LCD44780_LP_H_
#define LCD44780_LP_H_

// commands
	#define LCD_CLEARDISPLAY 0x01
	#define LCD_RETURNHOME 0x02
	#define LCD_ENTRYMODESET 0x04
	#define LCD_DISPLAYCONTROL 0x08
	#define LCD_CURSORSHIFT 0x10
	#define LCD_FUNCTIONSET 0x20
	#define LCD_SETCGRAMADDR 0x40
	#define LCD_SETDDRAMADDR 0x80

	// flags for display entry mode
	#define LCD_ENTRYRIGHT 0x00
	#define LCD_ENTRYLEFT 0x02
	#define LCD_ENTRYSHIFTINCREMENT 0x01
	#define LCD_ENTRYSHIFTDECREMENT 0x00

	// flags for display on/off control
	#define LCD_DISPLAYON 0x04
	#define LCD_DISPLAYOFF 0x00
	#define LCD_CURSORON 0x02
	#define LCD_CURSOROFF 0x00
	#define LCD_BLINKON 0x01
	#define LCD_BLINKOFF 0x00

	// flags for display/cursor shift
	#define LCD_DISPLAYMOVE 0x08
	#define LCD_CURSORMOVE 0x00
	#define LCD_MOVERIGHT 0x04
	#define LCD_MOVELEFT 0x00

	// flags for function set
	#define LCD_8BITMODE 0x10
	#define LCD_4BITMODE 0x00
	#define LCD_2LINE 0x08
	#define LCD_1LINE 0x00
	#define LCD_5x10DOTS 0x04
	#define LCD_5x8DOTS 0x00



#define LCDPORT  		 GPIO_PORTM_BASE
#define LCDPORTENABLE    SYSCTL_PERIPH_GPIOM
#define RS 		 		 GPIO_PIN_0
#define E 				 GPIO_PIN_1
#define D4 				 GPIO_PIN_4
#define D5 				 GPIO_PIN_5
#define D6				 GPIO_PIN_6
#define D7 				 GPIO_PIN_7
#define CLKSPEED		 120000000

void initLCD();
void LCDCommand(unsigned char);
void LCDWrite(unsigned char);
void LCDWriteText(char*,unsigned char, unsigned char);
void LCDWritePos(unsigned char,unsigned char, unsigned char );
void LCDScrollLeft();
void LCDScrollRight();
void LCD_build(unsigned char location, unsigned char *ptr);
void LCD_BuildCustomCharacters();
void LCDWriteData(unsigned char inputData) ;

#endif /* LCD44780_LP_H_ */

