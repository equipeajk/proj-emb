/*
 * crypto.h
 *
 *  Created on: 05/02/2015
 *      Author: Kelvin
 */

#ifndef CRYPTO_H_
#define CRYPTO_H_



#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/sha.h>
#include <wolfssl/wolfcrypt/aes.h>
#include <string.h>
#include "MFRC522.h"

void genAESKey(Uid *uid);
void encryptAES(byte *plainText, int size, byte *cipher);
void decryptAES(byte *plainText, int size, byte *cipher);
void genCardKey(Uid *uid, byte setor);

byte aesKey[SHA_DIGEST_SIZE];
byte cardKey[SHA_DIGEST_SIZE];
MIFARE_Key mifareCardKey;

#endif /* CRYPTO_H_ */
