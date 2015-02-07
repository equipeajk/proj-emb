/*
 * cardData.c
 *
 *  Created on: 06/02/2015
 *      Author: João
 */
#include "cardData.h"

void regNewCard(Card* card, Uid *uid)
{
	//Formatar cartao
	//Gerar senha do cartao
	byte setor = 5;
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

void incrementaContador(Card* card, Uid *uid)

