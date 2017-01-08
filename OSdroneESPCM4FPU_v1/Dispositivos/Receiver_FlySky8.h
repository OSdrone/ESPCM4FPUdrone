/*
 * Receiver_FlySky8.h
 *
 *  Created on: 30/12/2016
 *      Author: Ruben
 */

#ifndef DISPOSITIVOS_RECEIVER_FLYSKY8_H_
#define DISPOSITIVOS_RECEIVER_FLYSKY8_H_

//............RECEPTOR............................//
typedef struct{
	uint16_t Canal_PWM[8];
	uint8_t Voltaje_Bat[4];
	uint8_t Error_conexion;
}tpLectura_Radio;

typedef struct {
	q15_t Rango_Salida[2];   //min max
	uint16_t Rango_Entrada[2]; //min max
}tpCalibracion_canal_PWM;

typedef tpCalibracion_canal_PWM tpCalibracion_Receptor[numCanales];


#endif /* DISPOSITIVOS_RECEIVER_FLYSKY8_H_ */
