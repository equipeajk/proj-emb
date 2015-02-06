/*
 * crypto.c
 *
 *  Created on: 05/02/2015
 *      Author: Kelvin
 */
#include "crypto.h"

const unsigned char masterKey[]  =
{
		0x30, 0x82, 0x04, 0xAA, 0x30, 0x82, 0x03, 0x92,
		0xA0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x09, 0x00
};

void genAESKey(Uid *uid)
{
	byte key[20];
	memcpy(key, masterKey, sizeof(masterKey));
	memcpy(key+sizeof(masterKey), uid->uidByte, uid->size);

	Sha    hash;
	int    ret;

	ret = wc_InitSha(&hash);
	if (ret != 0) {
		return;
	}
	wc_ShaUpdate(&hash, key, sizeof(key));
	wc_ShaFinal(&hash, aesKey);
}

void encryptAES(byte *plainText, int size, byte *cipher)
{
	Aes    enc;
	int    ret;

	ret = wc_AesSetKey(&enc, aesKey, 16, masterKey, AES_ENCRYPTION);
	if (ret != 0) {
		return;
	}
	wc_AesCbcEncrypt(&enc, cipher, plainText, size);
}

void decryptAES(byte *plainText, int size, byte *cipher)
{
	Aes    enc;
	int    ret;

	ret = wc_AesSetKey(&enc, aesKey, 16, masterKey, AES_DECRYPTION);
	if (ret != 0) {
		return;
	}
	wc_AesCbcDecrypt(&enc, plainText, cipher, size);
}
