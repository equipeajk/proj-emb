/*
 * This file contains contributions from both TI and wolfSSL
 *
 * Copyright (c) 2014, Texas Instruments Incorporated
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 *
 * Copyright (c) 2006-2014, wolfSSL Inc.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
 */

/*
 *    ======== tcpClient.c ========
 */
#include <stdbool.h>
#include <string.h>
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* NDK Header files */
#include <sys/socket.h>
#include <ti/ndk/nettools/mytime/mytime.h>
#include <ti/ndk/inc/nettools/inc/inet.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Example/Board Header files */
#include "Board.h"

/* wolfSSL Header files */
#include <wolfssl/ssl.h>
#include <wolfssl/certs_test.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "MFRC522.h"
#include "crypto.h"
#include "cardData.h"

#define TCPPACKETSIZE 1024
#define TCPPORT 11111
#define NUMTCPWORKERS 3

/* Set to the IP of the computer running the wolfssl command from the 
 * wolfssl_root directory: "./examples/server/server -b" 
 * the "-b" tells the server to bind to any interface, not just 127.0.0.1
 */
#define IP_ADDR "104.41.3.85"



#define SPI_MSG_LENGTH	1;

SPI_Handle spiHandle;
SPI_Transaction masterTransaction;
extern Uid uid;
Uint32 tempoLED;
bool led = false;

int hardware_init(void);
void exitApp(WOLFSSL_CTX* ctx);

/*
 *  ======== tcpHandler ========
 *  Creates new Task to handle new TCP connections.
 */
Void tcpHandler_task(UArg arg0, UArg arg1) {
	int sockfd;
	int ret;
	struct sockaddr_in servAddr;
	Error_Block eb;
	bool flag = true;
	bool internal_flag = true;
	int nbytes;
	char *buffer;
	char msg[] = "Hello from TM4C1294XL Connected Launchpad";
	WOLFSSL* ssl = (WOLFSSL *) arg0;

	fdOpenSession(Task_self());

	wolfSSL_Init();
	WOLFSSL_CTX* ctx = NULL;

	ctx = wolfSSL_CTX_new(wolfTLSv1_2_client_method());
	if (ctx == 0) {
		System_printf("tcpHandler: wolfSSL_CTX_new error.\n");
		exitApp(ctx);
	}

	if (wolfSSL_CTX_load_verify_buffer(ctx, ca_cert_der_2048,
			sizeof(ca_cert_der_2048) / sizeof(char), SSL_FILETYPE_ASN1)
			!= SSL_SUCCESS) {
		System_printf("tcpHandler: Error loading ca_cert_der_2048"
				" please check the wolfssl/certs_test.h file.\n");
		exitApp(ctx);
	}

	if (wolfSSL_CTX_use_certificate_buffer(ctx, client_cert_der_2048,
			sizeof(client_cert_der_2048) / sizeof(char), SSL_FILETYPE_ASN1)
			!= SSL_SUCCESS) {
		System_printf("tcpHandler: Error loading client_cert_der_2048,"
				" please check the wolfssl/certs_test.h file.\n");
		exitApp(ctx);
	}

	if (wolfSSL_CTX_use_PrivateKey_buffer(ctx, client_key_der_2048,
			sizeof(client_key_der_2048) / sizeof(char), SSL_FILETYPE_ASN1)
			!= SSL_SUCCESS) {
		System_printf("tcpHandler: Error loading client_key_der_2048,"
				" please check the wolfssl/certs_test.h file.\n");
		exitApp(ctx);
	}

	/* Init the Error_Block */
	Error_init(&eb);

	do {
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0) {
			System_printf("tcpHandler: socket failed\n");
			Task_sleep(2000);
			continue;
		}

		memset((char *) &servAddr, 0, sizeof(servAddr));
		servAddr.sin_family = AF_INET;
		servAddr.sin_port = htons(TCPPORT);

		inet_aton(IP_ADDR, &servAddr.sin_addr);

		ret = connect(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr));

		if (ret < 0) {
			fdClose((SOCKET) sockfd);
			Task_sleep(2000);
			continue;
		}
	} while (ret != 0);

	if ((ssl = wolfSSL_new(ctx)) == NULL) {
		System_printf("tcpHandler: wolfSSL_new error.\n");
		exitApp(ctx);
	}

	wolfSSL_set_fd(ssl, sockfd);

	ret = wolfSSL_connect(ssl);

	/* Delete "TOP_LINE" and "END_LINE" for debugging. */

	/* TOP_LINE

	 System_printf("looked for: %d.\n", SSL_SUCCESS);
	 System_printf("return was: %d.\n", ret);
	 int err;
	 char err_buffer[80];
	 err = wolfSSL_get_error(ssl, 0);
	 System_printf("wolfSSL error: %d\n", err);
	 System_printf("wolfSSL error string: %s\n", wolfSSL_ERR_error_string(err, err_buffer));

	 END_LINE */

	if (ret == SSL_SUCCESS) {

		sockfd = wolfSSL_get_fd(ssl);

		/* Get a buffer to receive incoming packets. Use the default heap. */
		buffer = Memory_alloc(NULL, TCPPACKETSIZE, 0, &eb);

		if (buffer == NULL) {
			System_printf("tcpWorker: failed to alloc memory\n");
			exitApp(ctx);
		}

		/* Say hello to the server */
		while (flag) {
			if (wolfSSL_write(ssl, msg, strlen(msg)) != strlen(msg)) {
				ret = wolfSSL_get_error(ssl, 0);
				System_printf("Write error: %i.\n", ret);
			}
			while (internal_flag) {
				nbytes = wolfSSL_read(ssl, (char *) buffer, TCPPACKETSIZE);
				if (nbytes > 0) {
					internal_flag = false;
				}
			}
			/* success */
			System_printf("Heard: \"%s\".\n", buffer);
			wolfSSL_free(ssl);
			fdClose((SOCKET) sockfd);
			flag = false;
		}

		/* Free the buffer back to the heap */
		Memory_free(NULL, buffer, TCPPACKETSIZE);

		/*
		 *  Since deleteTerminatedTasks is set in the cfg file,
		 *  the Task will be deleted when the idle task runs.
		 */
		exitApp(ctx);

	} else {
		wolfSSL_free(ssl);
		fdClose((SOCKET) sockfd);
		System_printf("wolfSSL_connect failed.\n");
		fdCloseSession(Task_self());
		exitApp(ctx);
	}
}

/*
 *  ======== exitApp ========
 *  Cleans up the SSL context and exits the application
 */
void exitApp(WOLFSSL_CTX* ctx) {
	if (ctx != NULL) {
		wolfSSL_CTX_free(ctx);
		wolfSSL_Cleanup();
	}

	//BIOS_exit(-1);
}


void PCDCheck_task()
{
	//Hard reset no PCD
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
	Task_sleep(50);
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
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();

	//Loop da task
	GPIO_write(Board_LED0, Board_LED_ON);
	for(;;)
	{
		//Desliga o LED de confirma��o ap�s 2 segundos.
		if(Clock_getTicks() - tempoLED > 2000 && led)
		{
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
				L� os dados relevantes
				Decodifica os dados
				Decrementa o poss�vel contador de transa��es
				Codifica o novo contador e escreve no cart�o
				Chama a task que envia para o servidor
				 */


				Card card;
				byte codigo[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
						0x0, 0x0, 0x1, 0x04, 0x78, 0x41 };
				byte curso[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
										0x0, 0x0, 0x0, 0x0, 0x2, 0x12 };
				byte flag[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
						0x0, 0x0, 0x0, 0x0, 0x0, 0x1 };
				memcpy(card.cod, codigo, sizeof(codigo) );
				memcpy(card.curso, curso, sizeof(curso) );
				memcpy(card.flag, flag, sizeof(flag) );

				//regNewCard(&card, &uid);

				genAESKey(&uid);

				byte plainText[] = {0x30, 0x0E, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0C, 0x07, 0x4D,
						0x6F, 0x6E, 0x74, 0x61, 0x6E, 0x61 };
				byte cipher[16];
				//encryptAES(plainText, 16, cipher);
				//decryptAES(plainText, 16, cipher);
				genCardKey(&uid, 5);
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
		Task_sleep(200);
	}
	SPI_close(spiHandle);
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

/*
 *  ======== main ========
 */
int main(void) {
	Task_Handle taskHandlePCDCheck;
	Task_Params taskParamsPCDCheck;
	Error_Block ebPCDCheck;

	Task_Handle taskHandletcp;
	Task_Params taskParamstcp;
	Error_Block ebtcp;
#ifdef TIVAWARE
	/*
	 *  This is a work-around for EMAC initialization issues found on
	 *  the TM4C129 devices. The bug number is:
	 *  SDOCM00107378: NDK examples for EK-TM4C1294XL do not work
	 *
	 *  The following disables the flash pre-fetch. It is enable within the
	 *  EMAC driver (in the EMACSnow_NIMUInit() function).
	 */
	UInt32 ui32FlashConf;

	ui32FlashConf = HWREG(0x400FDFC8);
	ui32FlashConf &= ~(0x00020000);
	ui32FlashConf |= 0x00010000;
	HWREG(0x400FDFC8) = ui32FlashConf;
#endif

	/* Call board init functions */
	Board_initGeneral();
	Board_initGPIO();
	Board_initEMAC();
	Board_initSPI();
	hardware_init();

	/*
	 * wolfSSL library needs time() for validating certificates.
	 * USER STEP: Set up the current time in seconds below.
	 */
	MYTIME_init();
	MYTIME_settime(1423096611);

	System_printf("Starting the TCP Echo example\nSystem provider is set to "
			"SysMin. Halt the target to view any SysMin contents in"
			" ROV.\n");
	/* SysMin will only print to the console when you call flush or exit */
	System_flush();

	/*
	 *  Create the Task that farms out incoming TCP connections.
	 *  arg0 will be the port that this task listens to.
	 */
	Task_Params_init(&taskParamstcp);
	Error_init(&ebtcp);

	taskParamstcp.stackSize = 32768;
	taskParamstcp.priority = 1;
	taskParamstcp.arg0 = TCPPORT;
	taskHandletcp = Task_create((Task_FuncPtr) tcpHandler_task, &taskParamstcp, &ebtcp);
	if (taskHandletcp == NULL) {
		System_printf("main: Failed to create tcpHandler Task\n");
	}

	Task_Params_init(&taskParamsPCDCheck);
	Error_init(&ebPCDCheck);
	taskParamsPCDCheck.stackSize = 2048;
	taskParamsPCDCheck.priority = 1;
	taskHandlePCDCheck = Task_create((Task_FuncPtr) PCDCheck_task, &taskParamsPCDCheck, &ebPCDCheck);
	if (taskHandlePCDCheck == NULL) {
		System_printf("main: Failed to create PCDCheck_task Task\n");
	}
	/* Start BIOS */
	BIOS_start();

	return (0);
}
