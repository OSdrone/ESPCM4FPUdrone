/*
 * AHRS.h
 *
 *  Created on: 22/12/2015
 *      Author: Ruben
 */
#include "math.h"
#include "OSmath.h"
#include "arm_math.h"
#include "Constantes.h"

#ifndef AHRS_H_
#define AHRS_H_

typedef struct{
	q16_16_t TiempoMuestreo;

	//Valor maximo en MAX_GANANCIA_AHRS ,siendo q15_t*MAX_GANANCIA_AHRS
	q15_t Kp_Roll_Pitch;
	q15_t Ki_Roll_Pitch;
	q15_t Kp_Yaw;
	q15_t Ki_Yaw;

	q16_4_t Correccion_Proporcional[3];
	q16_4_t Correccion_Integral[3];
}tpConfiguracionAHRS;

typedef struct{
	float32_t Roll;
	float32_t Pitch;
	float32_t Yaw;

	q15_t DCM_matriz[3][3];
	arm_matrix_instance_q15 DCM;
}tpOrientacionAHRS;

typedef struct{
	tpConfiguracionAHRS ConfiguracionAHRS;
	tpOrientacionAHRS OrientacionAHRS;
}tpAHRS;

q31_t Compensacion_Sensor_magnetico(q15_t Roll, q15_t Pitch, int16_t VectorMagnetico[3]);
void Actualizar_Matriz_DCM_V2(tpAHRS *AHRS, q16_4_t VelocidadAngular[3]);
void Normalizar_DCM(tpAHRS *AHRS);
void Correccion_deriva(tpAHRS *AHRS, int16_t AceleracionLineal[3], q31_t OrientacionMagentica);
void Angulos_Euler(tpAHRS *AHRS);

#endif /* AHRS_H_ */
