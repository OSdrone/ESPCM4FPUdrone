/*
 * Transmisores.c
 *
 *  Created on: 7/10/2015
 *      Author: Ruben_User
 */

#include <ti/drivers/SPI.h>
#include <ti/sysbios/Knl/Clock.h>
#include "Transmisores.h"
#include "Parametros.h"
#include <ti/drivers/GPIO.h>

bool Iniciar_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t RX, uint8_t Canal, uint8_t Payload){
	uint8_t Dato;
	uint32_t systick = 0;
	bool Transmision_OK;

	Transmision_OK = Escribir_Registro_nRF24L01(nRF24L01, 0x00 , (0b00001010 | (0x01 & RX))); // Inicio Transmisor, CRC, NO Int
	systick = Clock_getTicks();
	while(Clock_getTicks() < systick+2);
	Transmision_OK &= Escribir_Registro_nRF24L01(nRF24L01, 0x05 , Canal);
	Transmision_OK &= Escribir_Registro_nRF24L01(nRF24L01, 0x11 , Payload);
	Transmision_OK &= Flush_nRF24L01(nRF24L01, true);
	Transmision_OK &= Leer_Registro_nRF24L01(nRF24L01, 0x00, &Dato);

	 return (Transmision_OK);
}

void Activar_nRF24L01(tp_nRF24L01 nRF24L01){
	GPIO_write(nRF24L01.PIN_CE, 1);
}

void Desactivar_nRF24L01(tp_nRF24L01 nRF24L01){
	GPIO_write(nRF24L01.PIN_CE, 0);
}

bool Flush_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t RX){
	SPI_Transaction SPI_Transmision;
	uint8_t Buffer_Tx = 0xE1;
	bool Transmision_OK;

	Buffer_Tx += RX;

	SPI_Transmision.count = 1;
	SPI_Transmision.txBuf = &Buffer_Tx;
	SPI_Transmision.rxBuf = NULL;

	GPIO_write(nRF24L01.PIN_CSN, 0);
	Transmision_OK = (SPI_transfer(nRF24L01.SPI, &SPI_Transmision));
	GPIO_write(nRF24L01.PIN_CSN, 1);

	return(Transmision_OK);
}

bool Escribir_Registro_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t Reg, uint8_t Dato){
	SPI_Transaction SPI_Transmision;
	uint8_t Buffer_Tx[2];
	bool Transmision_OK;
	uint8_t Estado_Anterior;

	Estado_Anterior = GPIO_read(nRF24L01.PIN_CE);

	Buffer_Tx[0] = 0x20 | Reg;
	Buffer_Tx[1] = Dato;

	SPI_Transmision.count = 2;
	SPI_Transmision.txBuf = Buffer_Tx;
	SPI_Transmision.rxBuf = NULL;

	GPIO_write(nRF24L01.PIN_CE, 0);
	GPIO_write(nRF24L01.PIN_CSN, 0);
	Transmision_OK = (SPI_transfer(nRF24L01.SPI, &SPI_Transmision));
	GPIO_write(nRF24L01.PIN_CSN, 1);
	GPIO_write(nRF24L01.PIN_CE, Estado_Anterior);

	return(Transmision_OK);
}

bool Leer_Registro_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t Reg, uint8_t* Dato){
	SPI_Transaction SPI_Transmision;
	bool Transmision_OK;
	uint8_t Buffer_Tx[2] = {0, 0};
	uint8_t Buffer_Rx[2];

	Buffer_Tx[0] = (Reg & 0x1F);

	SPI_Transmision.count = 2;
	SPI_Transmision.txBuf = Buffer_Tx;
	SPI_Transmision.rxBuf = Buffer_Rx;

	GPIO_write(nRF24L01.PIN_CSN, 0);
	Transmision_OK =  SPI_transfer(nRF24L01.SPI, &SPI_Transmision);
	GPIO_write(nRF24L01.PIN_CSN, 1);
	*Dato = Buffer_Rx[1];

	return(Transmision_OK);
}
/*
bool WritePayLoad_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t* PayLoad, size_t n_PayLoad){
	SPI_Transaction SPI_Transmision;

	PayLoad[0] = 0b10100000;
	SPI_Transmision.count = n_PayLoad;
	SPI_Transmision.txBuf = PayLoad;
	SPI_Transmision.rxBuf = NULL;

	GPIO_write(nRF24L01.PIN_CSN, 0);
	return (SPI_transfer(nRF24L01.SPI, &SPI_Transmision));
	GPIO_write(nRF24L01.PIN_CSN, 1);
}

bool ReadPayLoad_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t* PayLoad, size_t n_PayLoad){
	SPI_Transaction SPI_Transmision;

	PayLoad[0] = 0b01100001;
	SPI_Transmision.count = n_PayLoad;
	SPI_Transmision.rxBuf = PayLoad;
	SPI_Transmision.txBuf = NULL;

	GPIO_write(nRF24L01.PIN_CSN, 1);
	return (SPI_transfer(nRF24L01.SPI, &SPI_Transmision));
}
*/
bool MandarByte_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t Dato){
	SPI_Transaction SPI_Transmision;
	bool Transmision_OK;
	uint8_t Tx[2] = {0b10100000, 0};

	Tx[1] = Dato;
	GPIO_write(nRF24L01.PIN_CSN, 0);

	SPI_Transmision.count = 2;
	SPI_Transmision.txBuf = Tx;
	SPI_Transmision.rxBuf = NULL;

	GPIO_write(nRF24L01.PIN_CSN, 0);

	Transmision_OK =  SPI_transfer(nRF24L01.SPI, &SPI_Transmision);

	GPIO_write(nRF24L01.PIN_CSN, 1);

	return(Transmision_OK);
}

bool RecibirByte_nRF24L01(tp_nRF24L01 nRF24L01, uint8_t* Dato){
	SPI_Transaction SPI_Transmision;
	bool Transmision_OK;
	uint8_t Tx[2] = {0b01100001, 0};
	uint8_t Rx[2] = {0, 0};

	SPI_Transmision.count = 2;
	SPI_Transmision.txBuf = Tx;
	SPI_Transmision.rxBuf = Rx;

	GPIO_write(nRF24L01.PIN_CSN, 0);

	Transmision_OK =  SPI_transfer(nRF24L01.SPI, &SPI_Transmision);
	*Dato = Rx[1];

	GPIO_write(nRF24L01.PIN_CSN, 1);

	return(Transmision_OK);
}

