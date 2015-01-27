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
#include <ti/sysbios/knl/Clock.h>

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
Uint32 tempoLED;
bool led = false;

int hardware_init(void);



int main(void)
{
	/* Call board init functions. */
	Board_initGeneral();
	Board_initGPIO();
	Board_initSPI();

	//Inicializa as portas que serao utilizadas e o SPI
	if(hardware_init())
	{
		/* Start BIOS */
		BIOS_start();
	}
}


int hardware_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);

	//Inicializa SPI
	spiHandle = SPI_open(Board_SPI0, NULL);
	if (spiHandle == NULL) {
		System_abort("Error initializing SPI\n");
		return 0;
	}
	else {
		System_printf("SPI initialized\n");
	}
	return 1;
}




void PCDCheck_task()
{

	//Hard reset no PCD
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
	Task_sleep(5);

	while (PCD_ReadRegister(CommandReg) & (1<<4)) {
		// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	}
	//Faz o auto-test
	bool test = digitalSelfTestPass();
	if(test){
		System_printf("\nSelf Test Successful\n");
		System_flush();
	}

	PCD_WriteRegister(CommandReg, PCD_SoftReset);


	//Inicializa o PCD, liga a antena
	byte TMode = PCD_ReadRegister(TModeReg);
	PCD_WriteRegister(TModeReg, 0x80);	// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	TMode = PCD_ReadRegister(TModeReg);
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25ï¿½s.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);

	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

	PCD_AntennaOn();

	//Loop da task
	GPIO_write(Board_LED0, Board_LED_ON);
	for(;;){
		//Desliga o LED de confirmação após 2 segundos.
		if(Clock_getTicks() - tempoLED > 200 && led){
			GPIO_write(Board_LED1, Board_LED_OFF);
			led = false;
		}
		if(PICC_IsNewCardPresent()){
			System_printf("Novo cartao encontrado\n");
			System_flush();
			byte serial = PICC_ReadCardSerial();
			if(serial){
				//Faz o que quiser com o cartao, ja possui o UID do cartao.

				/*TODO:
				Calcula a senha baseado no UID
				Autentica com a senha encontrada
				Lê os dados relevantes
				Decodifica os dados
				Decrementa o possível contador de transações
				Codifica o novo contador e escreve no cartão
				Chama a task que envia para o servidor
				*/

				PICC_DumpToSerial(&uid);
				GPIO_write(Board_LED1, Board_LED_ON);
				tempoLED = Clock_getTicks();
				led = true;
				//Exemplo de como escrever em um bloco do cartao
				/*MIFARE_Key key;
				byte i = 0;
				for (i = 0; i < 6; i++) {
					key.keyByte[i] = 0xFF;
				}
				byte status;
				status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, 0x3, &key, &uid);
				if (status != STATUS_OK) {
					System_printf("PCD_Authenticate() failed: ");
					System_printf(GetStatusCodeName(status));
					System_flush();
				}
				else{
					byte write[] = { 0xBA, 0xBA, 0xCA, 0xBA, 0xBA, 0xCA, 0xBA,
							0xBA, 0xCA, 0xBA, 0xBA, 0xCA, 0xBA, 0xBA, 0xCA, 0xBA };
					MIFARE_Write(0x1, &write, 16);
				}
				PCD_StopCrypto1();*/

			}

		}
		Task_sleep(20);
	}
	SPI_close(spiHandle);

}

