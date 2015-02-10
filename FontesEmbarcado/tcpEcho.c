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
#include <ti/sysbios/knl/Mailbox.h>

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
#include "lcd44780_LP.h"

//TODO: tamanho 1
#define TCPPACKETSIZE 64
#define TCPPORT 11111
#define IP_ADDR "104.41.3.85"

SPI_Handle spiHandle;
Semaphore_Handle semNet; // eh postado qnd esta conectado
SPI_Transaction masterTransaction;
Mailbox_Handle myMailBox;
extern Uid uid;
Uint32 tempoLED;
bool led = false;
byte serverAnswer;

int hardware_init(void);
void exitApp(WOLFSSL_CTX* ctx);
void intTobyteArray(uint32_t n, byte* array);
uint32_t byteArrayToInt(byte* array);

typedef struct MsgObj {
	char* val;
} MsgObj, *Msg;

const unsigned char EA_id[]  =
{
		0x30, 0x82, 0x04, 0xAA, 0x30
};

void intTobyteArray(uint32_t n, byte* array)
{
	array[0] = (n >> 24) & 0xFF;
	array[1] = (n >> 16) & 0xFF;
	array[2] = (n >> 8) & 0xFF;
	array[3] = n & 0xFF;
}

uint32_t byteArrayToInt(byte* array)
{
	uint32_t n = 0;
	n +=	(uint32_t)(array[0] << 24) +
			(uint32_t)(array[1] << 16) +
			(uint32_t)(array[2] << 8) +
			(uint32_t) array[3];
	return n;
}

/*
 *  ======== tcpHandler ========
 *  Creates new Task to handle new TCP connections.
 */
Void tcpHandler_task()
{
	int ret;
	struct sockaddr_in servAddr;
	int sockfd;
	WOLFSSL_CTX* ctx;
	WOLFSSL* ssl;
	Error_Block eb;
	/* Init the Error_Block */
	Error_init(&eb);
	ssl = (WOLFSSL *) TCPPORT;
	fdOpenSession(Task_self());
	wolfSSL_Init();
	ctx = NULL;
	/* The application is a client and will only support the TLS 1.2 protocol.
	 * Allocates memory for and initializes a new WOLFSSL_CTX structure */
	ctx = wolfSSL_CTX_new(wolfTLSv1_2_client_method());
	if (ctx == 0) {
		System_printf("tcpHandler: wolfSSL_CTX_new error.\n");
		exitApp(ctx);
	}
	/* Loads a CA certificate buffer into the CYASSL Context */
	if (wolfSSL_CTX_load_verify_buffer(ctx, ca_cert_der_2048,
			sizeof(ca_cert_der_2048) / sizeof(char), SSL_FILETYPE_ASN1)
			!= SSL_SUCCESS) {
		System_printf("tcpHandler: Error loading ca_cert_der_2048"
				" please check the wolfssl/certs_test.h file.\n");
		exitApp(ctx);
	}
	/* Loads a certificate buffer into the CYASSL Context */
	if (wolfSSL_CTX_use_certificate_buffer(ctx, client_cert_der_2048,
			sizeof(client_cert_der_2048) / sizeof(char), SSL_FILETYPE_ASN1)
			!= SSL_SUCCESS) {
		System_printf("tcpHandler: Error loading client_cert_der_2048,"
				" please check the wolfssl/certs_test.h file.\n");
		exitApp(ctx);
	}
	/* Loads a private key buffer into the SSL Context */
	if (wolfSSL_CTX_use_PrivateKey_buffer(ctx, client_key_der_2048,
			sizeof(client_key_der_2048) / sizeof(char), SSL_FILETYPE_ASN1)
			!= SSL_SUCCESS) {
		System_printf("tcpHandler: Error loading client_key_der_2048,"
				" please check the wolfssl/certs_test.h file.\n");
		exitApp(ctx);
	}

	/* Try to connect to the server */
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

	/* Creates a new SSL session, taking an already created SSL context (ctx) as input. */
	if ((ssl = wolfSSL_new(ctx)) == NULL) {
		System_printf("tcpHandler: wolfSSL_new error.\n");
		exitApp(ctx);
	}
	/* Assigns a socket file descriptor (sockfd) as the input/output facility for the SSL connection. */
	wolfSSL_set_fd(ssl, sockfd);
	/* Initiates an SSL/TLS handshake with a server.
	 * When this function is called, the underlying communication channel has already been set up. */
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
		/* Get a buffer to receive incoming packets. Use the default heap. */
		char *buffer;
		buffer = Memory_alloc(NULL, TCPPACKETSIZE, 0, &eb);
		if (buffer == NULL) {
			System_printf("tcpWorker: failed to alloc memory\n");
			exitApp(ctx);
		}
		MsgObj msg;
		int nbytes;
		bool internal_flag = true;
		/* Inicia sessão */
		char initMsg[6];
		initMsg[0] = 1;
		memcpy(initMsg+1, EA_id, 5);
		sockfd = wolfSSL_get_fd(ssl);
		if (wolfSSL_write(ssl, initMsg, strlen(initMsg)) != strlen(initMsg)) {
			ret = wolfSSL_get_error(ssl, 0);
			System_printf("Write error: %i.\n", ret);
			System_flush();
		}
		/* Wait for server response */
		//TODO: timeout
		nbytes = wolfSSL_read(ssl, (char *) buffer, TCPPACKETSIZE);
		if (nbytes > 0) {
			if(buffer[0] == 2) {
				System_printf("Erro servidor, resposta: %s", buffer);
				System_flush();
			}
			else if(buffer[0] == 1) {
				/* conexão estabelecida, pode comecar a ler cartoes */
				Semaphore_post(semNet);
			}
		}

		/* Wait for message */
		while(Mailbox_pend(myMailBox, &msg, BIOS_WAIT_FOREVER))
		{
			// msg recebida
			sockfd = wolfSSL_get_fd(ssl);
			if (sockfd < 0) {
				System_printf("tcpSendReceive: socket failed\n");
				exitApp(ctx);
			}
			//TODO: verificar se vai bloquear ou retornar erro e timeout
			/* Send message to the server */
			if (wolfSSL_write(ssl, msg.val, strlen(msg.val)) != strlen(msg.val)) {
				ret = wolfSSL_get_error(ssl, 0);
				System_printf("Write error: %i.\n", ret);
				System_flush();
			}
			/* Wait for server response */
			while (internal_flag) {
				nbytes = wolfSSL_read(ssl, (char *) buffer, TCPPACKETSIZE);
				if (nbytes > 0) {
					internal_flag = false;
				}
			}
			/* success */
			System_printf("Heard: \"%s\".\n", buffer);
			serverAnswer = buffer[0];
			Semaphore_post(semNet);
		}
		Memory_free(NULL, buffer, TCPPACKETSIZE);
		/* Encerra conexão */
		wolfSSL_free(ssl);
		fdClose((SOCKET) sockfd);
		/* Free the buffer back to the heap */
		fdCloseSession(Task_self());
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
	System_flush();
	if (ctx != NULL) {
		wolfSSL_CTX_free(ctx);
		wolfSSL_Cleanup();
	}
	//BIOS_exit(-1);
}

/*
 *  ======== PCDCheck ========
 *  Aguarda um cartao e realiza a autenticacao
 */
void PCDCheck_task()
{
	//Hard reset no PCD
	initLCD();
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
	//TODO: else
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
	/*TODO Espera conectar com o servidor */
	//Semaphore_pend(semNet, BIOS_WAIT_FOREVER);
	LCDWriteText("Passe o cartao", 0, 0);

	//Loop da task
	GPIO_write(Board_LED0, Board_LED_ON);
	for(;;)
	{
		//Desliga o LED de confirmacao apos 2 segundos.
		if(Clock_getTicks() - tempoLED > 3000 && led)
		{
			GPIO_write(Board_LED1, Board_LED_OFF);
			led = false;
			LCDCommand(LCD_CLEARDISPLAY);
			LCDCommand(LCD_BLINKON|LCD_CURSORON);
			LCDWriteText("Passe o cartao", 0, 0);
		}
		if(PICC_IsNewCardPresent()){
			System_printf("Novo cartao encontrado\n");
			System_flush();
			/* Preenche uid */
			byte serial = PICC_ReadCardSerial();
			if(serial)
			{
				genAESKey(&uid);

				//teste
				//				byte codigo[16] = {0};
				//				byte contador[16] = {0};
				//				intTobyteArray(840416, codigo+12);
				//				intTobyteArray(10000, contador+12);
				//				writeBlock(20, &uid, codigo);
				//				writeBlock(21, &uid, contador);
				//
				//				genCardKey(&uid, 5);
				//				byte trailer[16];
				//				memcpy(trailer, mifareCardKey.keyByte, 6);
				//				trailer[6] = 0xFF;
				//				trailer[7] = 0x07;
				//				trailer[8] = 0x80;
				//				trailer[9] = 0x69;
				//				int x = 0;
				//				for(x = 0; x < 6; x++)
				//				{
				//					trailer[10+x] = 0xFF;
				//				}
				//				writeBlock(23, &uid, trailer);
				//teste fim

				byte block4[16];
				readBlock(20, &uid, block4); //bloco 3 cod aluno
				byte block5[16];
				readBlock(21, &uid, block5); //bloco 4 contador
				LCDCommand(LCD_CLEARDISPLAY);
				char lcdcodigo[10];
				sprintf(lcdcodigo, "%d", byteArrayToInt(block4+12));
				LCDWriteText(lcdcodigo, 0, 4);
				/* Cria a msg a ser enviada para o servidor */
				MsgObj msg;
				char msgAuth[18];
				msgAuth[0] = 2; //operacao
				memcpy(msgAuth+1, &(uid.uidByte), 4); //uid do cartao
				memcpy(msgAuth+5, block4+12, 4); //codigo aluno
				memcpy(msgAuth+9, block5+12, 4); //contador
				memcpy(msgAuth+13, EA_id, 5); //uid da EA
				msg.val = msgAuth;
				Mailbox_post(myMailBox, &msg, BIOS_WAIT_FOREVER);

				/*TODO: espera resposta o servidor */
				//Semaphore_pend(semNet, BIOS_WAIT_FOREVER);
				serverAnswer = 1;
				switch(serverAnswer){
					case 1:
					{
						LCDWriteText("OK", 1, 7);
						// se resposta ok
						/* incrementa e salva contador */
						uint32_t newCont = byteArrayToInt(block5 + 12);
						newCont++;
						intTobyteArray(newCont, block5 + 12);
						writeBlock(21, &uid, block5); // bloco 4
						break;
					}
					default:
						LCDWriteText("ERRO", 1, 5);
				}
				//PICC_DumpToSerial(&uid);
				GPIO_write(Board_LED1, Board_LED_ON);
				tempoLED = Clock_getTicks();
				led = true;
				PICC_HaltA();
				PCD_StopCrypto1();
				PICC_HaltA();
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
	 */
	Task_Params_init(&taskParamstcp);
	Error_init(&ebtcp);
	taskParamstcp.stackSize = 32768; //32768
	taskParamstcp.priority = 1;
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

	Mailbox_Params myMBParams;
	Mailbox_Params_init(&myMBParams);
	Error_Block eb;
	Error_init(&eb);
	myMailBox = Mailbox_create(sizeof(MsgObj), 3, &myMBParams,&eb);
	if (myMailBox == NULL) {
		System_printf("main: Failed to create myMailBox\n");
	}

	Semaphore_Params semParams;
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	semNet = Semaphore_create(0, &semParams, &eb);
	if (semNet == NULL) {
		System_printf("main: Failed to create semNet\n");
	}
	/* Start BIOS */
	BIOS_start();
	return (0);
}
