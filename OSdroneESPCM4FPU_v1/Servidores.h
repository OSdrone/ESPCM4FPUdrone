/*
 * Servidores.h
 *
 *  Created on: 17/5/2015
 *      Author: Ruben
 */

#ifndef QUADROTOR_V1_3_1_SERVIDORES_H_
#define QUADROTOR_V1_3_1_SERVIDORES_H_

#include "arm_math.h"
#include "Dispositivos/IMU_MPU9250.h"
#include "AHRS.h"

void Iniciar_Servidores();

void Leer_servidor_Lecturas_IMU_9DOF(tpLecturas9DOFIMU *Lecturas9DOFIMU);

void Escribir_servidor_Lecturas_IMU_9DOF(tpLecturas9DOFIMU *Lecturas9DOFIMU);
void Escribir_servidor_Lecturas_IMU(tpLecturasIMU *LecturasIMU);
void Escribir_servidor_Lecturas_Brujula(tpLecturasBrujula *LecturasBrujula);

void Leer_servidor_DCM(int16_t* DCM);
void Escribir_servidor_DCM(int16_t* DCM);

void Leer_servidor_RPY(float32_t *Roll, float32_t *Pitch, float32_t *Yaw);
void Escribir_servidor_RPY(float32_t *Roll, float32_t *Pitch, float32_t *Yaw);

void Leer_servidor_Variables_Estado_Medidas(float32_t* Variables_Estado_Medidas);
void Escribir_servidor_Variables_Estado_Medidas(float32_t* Variables_Estado_Medidas);

void Leer_servidor_Variables_Estado_Estimadas(float32_t* Variables_Estado_Estimadas);
void Escribir_servidor_Variables_Estado_Estimadas(float32_t* Variables_Estado_Estimadas);

void Leer_servidor_Perturbaciones_Estimadas(float32_t* Perturbaciones_Estimadas);
void Escribir_servidor_Perturbaciones_Estimadas(float32_t* Perturbaciones_Estimadas);
void Resetear_servidor_Perturbaciones_Estimadas();

void LeerVariableAHRS_Orientacion_q15(tpOrientacionAHRS* OrientacionAHRS);
void EscribirVariableAHRS_Orientacion_q15(tpOrientacionAHRS* OrientacionAHRS);

void Leer_servidor_Referencia(float32_t* Referencia, int16_t* Referencia_Entero);
void Escribir_servidor_Referencia(float32_t* Referencia, int16_t* Referencia_Entero);
float32_t* Direccion_servidor_Referencia();

#endif /* QUADROTOR_V1_3_1_SERVIDORES_H_ */
