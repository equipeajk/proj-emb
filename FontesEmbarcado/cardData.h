/*
 * cardData.h
 *
 *  Created on: 06/02/2015
 *      Author: Jo�o
 */

#ifndef CARDDATA_H_
#define CARDDATA_H_

#include "crypto.h"
#include <xdc/runtime/System.h>

//Estes dados devem estar criptografados usando AESKey
typedef struct {
		byte		cod[16];			// codigo do aluno ou professor
		byte		curso[16];			// curso ou departamento
		byte		flag[16];			    // professor ou aluno
	} Card;

void regNewCard(Card* card, Uid *uid);
byte readBlock(int block, Uid *uid, byte* buffer);
byte writeBlock(int block, Uid *uid, byte* buffer);

#endif /* CARDDATA_H_ */
