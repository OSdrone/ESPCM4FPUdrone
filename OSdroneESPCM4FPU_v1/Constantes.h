/*
 * Constantes.h
 *
 *  Created on: 12 de oct. de 2016
 *      Author: Ruben
 */
#include "arm_math.h"

#ifndef CONSTANTES_H_
#define CONSTANTES_H_

//.........CONTROL............//
#define K_MAX_CONTROL 256
#define SHIFT_K_CONTROL 8

#define NUM_MOTORES 4
#define NUM_EJES_CONTROL 3

#define NUM_VAR_ESTADO 10


//.........IMU..............//
#define NUM_ELEMENTOS_BUFFER_IMU 32

// Filtros IMU
#define COEF_FILTRO_VEL_VALUES {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
#define SHIFT_FILTRO_VELOCIDAD 0
#define NUM_ETAPAS_FILTRO_VEL_ANG 2
//q15_t COEF_FILTRO_VEL[5*NUM_ETAPAS_FILTRO_VEL_ANG];

#define COEF_FILTRO_ACEL_VALUES {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
#define SHIFT_FILTRO_ACEL 0
#define NUM_ETAPAS_FILTRO_ACEL_ANG 2
//q15_t COEF_FILTRO_ACEL[5*NUM_ETAPAS_FILTRO_ACEL_ANG];

//..........AHRS..............//
#define MAX_GANANCIA_AHRS 128
#define SHIFT_GANANCIA_AHRS 7
#endif /* CONSTANTES_H_ */
