/*
 * Transmisores.h
 *
 *  Created on: 5/10/2015
 *      Author: Ruben_User
 */

#ifndef QUADROTOR_V9_2_TRANSMISORES_H_
#define QUADROTOR_V9_2_TRANSMISORES_H_

#include <ti/drivers/SPI.h>
#include "Parametros.h"

typedef struct{
	SPI_Handle SPI;
	unsigned int PIN_CE;
	unsigned int PIN_CSN;
	unsigned int PIN_IRQ;
}tp_nRF24L01;

bool Iniciar_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t RX, uint8_t Canal, uint8_t Payload);
void Activar_nRF24L01(tp_nRF24L01 nRF24L01);
void Desactivar_nRF24L01(tp_nRF24L01 nRF24L01);

bool Flush_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t RX);

bool Escribir_Registro_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t Reg, uint8_t Dato);
bool Leer_Registro_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t Reg, uint8_t* Dato);

//bool WritePayLoad_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t* PayLoad, size_t n_PayLoad );
//bool ReadPayLoad_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t* PayLoad, size_t n_PayLoad );

bool MandarByte_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t Dato);
bool RecibirByte_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t* Dato);

#endif /* QUADROTOR_V9_2_TRANSMISORES_H_ */
