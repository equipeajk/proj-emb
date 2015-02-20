/*
 * cardData.c
 *
 *  Created on: 06/02/2015
 *      Author: Jo�o
 */
#include "cardData.h"

void regNewCard(Card* card, Uid *uid)
{
	//Formatar cartao
	//Gerar senha do cartao
	byte setor = 1;
	genCardKey(uid, setor);


	//Autenticar com senha padrao FFFFFF
	MIFARE_Key key;
	int i;
	for (i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;
	}

	byte status;
	status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, setor*4+3, &key, uid);
	if (status != STATUS_OK) {
		System_printf("PCD_Authenticate() failed: ");
		System_printf(GetStatusCodeName(status));
		System_flush();
	}
	else{
		//Alterar senha do cartao, access bits e dados para cada setor
		//Estes dados devem estar criptografados usando AESKey
		MIFARE_Write(setor*4, card->cod, 16);
		MIFARE_Write(setor*4+1, card->curso, 16);
		MIFARE_Write(setor*4+2, card->flag, 16);

		byte trailer[16];
		memcpy(trailer, cardKey, 6);
		trailer[6] = 0xFF;
		trailer[7] = 0x07;
		trailer[8] = 0x80;
		trailer[9] = 0xFF;
		memcpy(trailer+10, key.keyByte, 6);

		MIFARE_Write(setor*4+3, trailer, 16);
	}
	PCD_StopCrypto1();
}

/* Le o conteudo do block e o decifra */
byte readBlock(int block, Uid *uid, byte* buffer)
{
	genCardKey(uid, block/4);
	byte sizeBlock = 18;
	byte blockBuffer[sizeBlock];
	byte status;
	status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, ((block/4)*4)+3, &mifareCardKey, uid);
	if (status != STATUS_OK)
	{
		System_printf("PCD_Authenticate() failed: ");
		System_printf(GetStatusCodeName(status));
		System_printf("\n");
		System_flush();
		return 0;
	}
	status = MIFARE_Read(block, blockBuffer, &sizeBlock);
	if (status != STATUS_OK)
	{
		System_printf("MIFARE_Read() failed: ");
		System_printf(GetStatusCodeName(status));
		System_printf("\n");
		System_flush();
		return 0;
	}
	decryptAES(buffer, 16, blockBuffer);
	return 1;
}

/* cifra o buffer e escreve no bloco */
byte writeBlock(int block, Uid *uid, byte* buffer)
{
	genCardKey(uid, block/4);
	byte cipher[16];
	if(block != ((block/4)*4)+3)
		encryptAES(buffer, 16, cipher);
	byte status;
//	int x = 0;
//	for(x = 0; x < 6; x++)
//	{
//		mifareCardKey.keyByte[x] = 0xFF;
//	}
	status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, ((block/4)*4)+3, &mifareCardKey, uid);
	if (status != STATUS_OK)
	{
		System_printf("PCD_Authenticate() failed: ");
		System_printf(GetStatusCodeName(status));
		System_printf("\n");
		System_flush();
		return 0;
	}
	if(block != ((block/4)*4)+3)
		status = MIFARE_Write(block, cipher, 16);
	else
		status = MIFARE_Write(block, buffer, 16);
	if (status != STATUS_OK)
	{
		System_printf("MIFARE_Write() failed: ");
		System_printf(GetStatusCodeName(status));
		System_printf("\n");
		System_flush();
		return 0;
	}
	return 1;
}
