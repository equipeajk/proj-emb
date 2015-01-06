/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== spiloopback.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* TI-RTOS Header files */


#include <ti/drivers/SPI.h>

/* Example/Board Header files */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "Board.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "MFRC522.h"
#define SPI_MSG_LENGTH	1;

SPI_Handle spiHandle;
SPI_Transaction masterTransaction;
extern Uid uid;

void hardware_init(void);



int main(void)
{
	/* Call board init functions. */
		Board_initGeneral();
		// Board_initEMAC();
		Board_initGPIO();
		// Board_initI2C();
		// Board_initSDSPI();
//		byte * TxBufferTeste;
//		TxBufferTeste = Memory_alloc(NULL, 2*sizeof(byte), 8, NULL);
//
//		TxBufferTeste[0] =2;
		Board_initSPI();
		// Board_initUART();
		// Board_initUSB(Board_USBDEVICE);
		// Board_initUSBMSCHFatFs();
		// Board_initWatchdog();
		/* Turn on user LED */
		hardware_init();

		/* Start BIOS */
		BIOS_start();
}


void hardware_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);

	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);
//	for(x = 0; x < 6000000; x++);

	//for(x = 0; x < 6000000; x++);
}

void SPI_Master()
{

	spiHandle = SPI_open(Board_SPI0, NULL);
	if (spiHandle == NULL) {
		System_abort("Error initializing SPI\n");
	}
	else {
		System_printf("SPI initialized\n");
	}
	Task_sleep(50);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
	Task_sleep(50);
//	PCD_WriteRegister(CommandReg, PCD_SoftReset);

	while (PCD_ReadRegister(CommandReg) & (1<<4)) {
			// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	}

	/* Initialize master SPI transaction structure */
	byte TMode = PCD_ReadRegister(TModeReg);
	PCD_WriteRegister(TModeReg, 0x80);	// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	TMode = PCD_ReadRegister(TModeReg);
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);

	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

 	PCD_AntennaOn();
 	PCD_SetAntennaGain(RxGain_max);
 	byte gain = PCD_GetAntennaGain();
	uint8_t ver = PCD_ReadRegister(VersionReg);
	System_printf("%x",ver);
	System_flush();


	for(;;){
		if(PICC_IsNewCardPresent()){
			System_printf("Novo cartao encontrado\n");
			System_flush();
			byte serial = PICC_ReadCardSerial();
			if(serial){
				PICC_DumpToSerial(&uid);
			}

		}
		Task_sleep(100);
	}
	SPI_close(spiHandle);

}

