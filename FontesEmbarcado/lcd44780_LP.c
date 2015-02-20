/*
 *       lcd44780_LP.c
 *		Basic HD44780 driver for Stellaris Launchpad
 *      Version 1.00
 *      Author: NYH
 *      Reference: Robot Head to Toe Vol. 11 - pg 35-36
 *      Note: One full port must be used for this LCD. In this driver PORTB is used.
 */

//Modified by Don Le
//Added function to create custom character
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "lcd44780_LP.h"

void initLCD() {

	//SysCtlPeripheralEnable(LCDPORTENABLE);
	GPIOPinTypeGPIOOutput(LCDPORT,
				0xff);

	// Please refer to the HD44780 datasheet for how these initializations work!
	int tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 30){
	}
	//SysCtlDelay((500e-3)*CLKSPEED/3);

	GPIOPinWrite(LCDPORT, RS,  0x00 );

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7,  0x30 );
	GPIOPinWrite(LCDPORT, E, 0x02);

	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
	}
	//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 30){
	}
	//SysCtlDelay((50e-3)*CLKSPEED/3);

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7,  0x30 );
	GPIOPinWrite(LCDPORT, E, 0x02);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 30){
	}
	//SysCtlDelay((50e-3)*CLKSPEED/3);

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7,  0x30 );
	GPIOPinWrite(LCDPORT, E, 0x02);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);

	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 30){
	}
	//SysCtlDelay((10e-3)*CLKSPEED/3);

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7,  0x20 );
	GPIOPinWrite(LCDPORT, E, 0x02);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 30){
	}
	//SysCtlDelay((10e-3)*CLKSPEED/3);

	LCDCommand(0x01);	// Clear the screen.
	LCDCommand(0x06);	// Cursor moves right.
	LCDCommand(0x0f);	// Cursor blinking, turn on LCD.


}

void LCDCommand(unsigned char command) {

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7, (command & 0xf0) );
	GPIOPinWrite(LCDPORT, RS, 0x00);
	GPIOPinWrite(LCDPORT, E, 0x02);
	int tempo;
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7, (command & 0x0f) << 4 );
	GPIOPinWrite(LCDPORT, RS, 0x00);
	GPIOPinWrite(LCDPORT, E, 0x02);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);

}

void LCDWrite(unsigned char inputData) {
	int tempo;
	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7, (inputData & 0xf0) );
	GPIOPinWrite(LCDPORT, RS, 0x01);
	GPIOPinWrite(LCDPORT, E, 0x02);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7, (inputData & 0x0f) << 4 );
	GPIOPinWrite(LCDPORT, RS, 0x01);
	GPIOPinWrite(LCDPORT, E, 0x02);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);
	GPIOPinWrite(LCDPORT, E, 0x00);
	tempo = Clock_getTicks();
	while (Clock_getTicks() - tempo < 1){
		}
		//SysCtlDelay((20e-6)*CLKSPEED/3);

}


void LCDWriteText(char* inputText,unsigned char row, unsigned char col) {
	unsigned char address_d = 0;		// address of the data in the screen.
	switch(row)
	{
	case 0: address_d = 0x80 + col;		// at zeroth row
	break;
	case 1: address_d = 0xC0 + col;		// at first row
	break;
	case 2: address_d = 0x94 + col;		// at second row
	break;
	case 3: address_d = 0xD4 + col;		// at third row
	break;
	default: address_d = 0x80 + col;	// returns to first row if invalid row number is detected
	break;
	}

	LCDCommand(address_d);

	while(*inputText)					// Place a string, letter by letter.
		LCDWrite(*inputText++);
}


void LCDWScrollText(char* inputText,unsigned char row, unsigned char col) {
	unsigned char address_d = 0;		// address of the data in the screen.
	switch(row)
	{
	case 0: address_d = 0x80 + col;		// at zeroth row
	break;
	case 1: address_d = 0xC0 + col;		// at first row
	break;
	case 2: address_d = 0x94 + col;		// at second row
	break;
	case 3: address_d = 0xD4 + col;		// at third row
	break;
	default: address_d = 0x80 + col;	// returns to first row if invalid row number is detected
	break;
	}

	LCDCommand(address_d);

	while(*inputText)					// Place a string, letter by letter.
		LCDWrite(*inputText++);
}

void LCD_BuildCustomCharacters()
{

	unsigned char pattern1[8] ={0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f};
	unsigned char pattern2[8] ={0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x1f};
	unsigned char pattern3[8] ={0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x01f, 0x1f};
	unsigned char pattern4[8] ={0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x1f, 0x1f};
	unsigned char pattern5[8] ={0x0, 0x0, 0x0, 0x0, 0x1f, 0x1f, 0x1f, 0x1f};
	unsigned char pattern6[8] ={0x0, 0x0, 0x0, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
	unsigned char pattern7[8] ={0x0, 0x0, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
	unsigned char pattern8[8] ={0x0, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};

	LCD_build(0,pattern1);
	LCD_build(1,pattern2);
	LCD_build(2,pattern3);
	LCD_build(3,pattern4);
	LCD_build(4,pattern5);
	LCD_build(5,pattern6);
	LCD_build(6,pattern7);
	LCD_build(7,pattern8);



}




void LCD_build(unsigned char location, unsigned char *ptr){
       unsigned char i;
       if(location<8){
    	   LCDCommand(0x40+(location*8));
           for(i=0;i<8;i++)
        	   LCDWrite(ptr[ i ] );
           LCDCommand(0x80);
      }

 }



void LCDWritePos(unsigned char inputData,unsigned char row, unsigned char col) {
	  unsigned char address_d = 0;		// address of the data in the screen.
		switch(row)
		{
		case 0: address_d = 0x80 + col;		// at zeroth row
		break;
		case 1: address_d = 0xC0 + col;		// at first row
		break;
		case 2: address_d = 0x94 + col;		// at second row
		break;
		case 3: address_d = 0xD4 + col;		// at third row
		break;
		default: address_d = 0x80 + col;	// returns to first row if invalid row number is detected
		break;
		}

	LCDCommand(address_d);
	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7, (inputData & 0xf0) );
	GPIOPinWrite(LCDPORT, RS, 0x01);
	GPIOPinWrite(LCDPORT, E, 0x02);
	SysCtlDelay((20e-6)*CLKSPEED/100);
	GPIOPinWrite(LCDPORT, E, 0x00);

	//SysCtlDelay((100e-6)*CLKSPEED/5);

	GPIOPinWrite(LCDPORT, D4 | D5 | D6 | D7, (inputData & 0x0f) << 4 );
	GPIOPinWrite(LCDPORT, RS, 0x01);
	GPIOPinWrite(LCDPORT, E, 0x02);
	//SysCtlDelay((20e-6)*CLKSPEED/5);
	GPIOPinWrite(LCDPORT, E, 0x00);

	//SysCtlDelay((5e-3)*CLKSPEED/5);

}




void LCDScrollLeft(){
	LCDCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
	}


void LCDScrollRight(){
	LCDCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
	}

