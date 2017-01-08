/*
 * Servidores.c
 *
 *  Created on: 17/5/2015
 *      Author: Ruben
 */
#include "Servidores.h"
#include "arm_math.h"
#include <ti/sysbios/gates/GateMutexPri.h>

static volatile GateMutexPri_Handle SERVIDOR_Datos_9DOF_IMU;
static volatile GateMutexPri_Handle SERVIDOR_Datos_Giroscopo;
static volatile GateMutexPri_Handle SERVIDOR_Datos_Brujula;
static volatile GateMutexPri_Handle SERVIDOR_DCM;
static volatile GateMutexPri_Handle SERVIDOR_YPR;
static volatile GateMutexPri_Handle SERVIDOR_Variables_Estado_Medidas;
static volatile GateMutexPri_Handle SERVIDOR_Variables_Estado_Estimadas;
static volatile GateMutexPri_Handle SERVIDOR_Perturbaciones_Estimadas;
static volatile GateMutexPri_Handle SERVIDOR_Orientacion_q15;
static volatile GateMutexPri_Handle SERVIDOR_Referencia;

static tpLecturas9DOFIMU Lecturas9DOFIMU_Almacenada = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int16_t DCM_Almacenada[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float32_t Roll_Almacenado = 0;
static float32_t Pitch_Almacenado = 0;
static float32_t Yaw_Almacenado= 0;
static float32_t Variables_Estado_Medidas_Almacenadas[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float32_t Variables_Estado_Estimadas_Almacenadas[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float32_t Perturbaciones_Estimadas_Almacenadas[4] = {0, 0, 0, 0};
static tpOrientacionAHRS OrientacionAHRS_q15Almacenado;
static float32_t Referencia_Almacenada[4] = {0, 0, 0, 0};
static int16_t Referencia_Almacenada_Entero[4] = {0, 0, 0, 0};

void Iniciar_Servidores(){
	SERVIDOR_Datos_9DOF_IMU = GateMutexPri_create(NULL, NULL) ;
	SERVIDOR_Datos_Giroscopo = GateMutexPri_create(NULL, NULL) ;
	SERVIDOR_Datos_Brujula = GateMutexPri_create(NULL, NULL) ;
	SERVIDOR_DCM = GateMutexPri_create(NULL, NULL);
	SERVIDOR_YPR = GateMutexPri_create(NULL, NULL) ;
	SERVIDOR_Variables_Estado_Medidas = GateMutexPri_create(NULL, NULL) ;
	SERVIDOR_Variables_Estado_Estimadas = GateMutexPri_create(NULL, NULL) ;
	SERVIDOR_Perturbaciones_Estimadas = GateMutexPri_create(NULL, NULL);
	SERVIDOR_Orientacion_q15 = GateMutexPri_create(NULL, NULL);
	SERVIDOR_Referencia = GateMutexPri_create(NULL, NULL) ;
}


void Leer_servidor_Lecturas_IMU_9DOF(tpLecturas9DOFIMU *Lecturas9DOFIMU){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Datos_9DOF_IMU);
	memcpy(Lecturas9DOFIMU, &Lecturas9DOFIMU_Almacenada, sizeof(tpLecturas9DOFIMU));
	GateMutexPri_leave(SERVIDOR_Datos_9DOF_IMU, Key);
}
void Escribir_servidor_Lecturas_IMU_9DOF(tpLecturas9DOFIMU *Lecturas9DOFIMU){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Datos_9DOF_IMU);
	memcpy(&Lecturas9DOFIMU_Almacenada, Lecturas9DOFIMU, sizeof(tpLecturas9DOFIMU));
	GateMutexPri_leave(SERVIDOR_Datos_9DOF_IMU, Key);
}

void Escribir_servidor_Lecturas_IMU(tpLecturasIMU *LecturasIMU){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Datos_9DOF_IMU);
	memcpy(&Lecturas9DOFIMU_Almacenada.LecturasIMU.x_acel, LecturasIMU, sizeof(tpLecturasIMU));
	GateMutexPri_leave(SERVIDOR_Datos_9DOF_IMU, Key);
}

void Escribir_servidor_Lecturas_Brujula(tpLecturasBrujula *LecturasBrujula){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Datos_9DOF_IMU);
	memcpy(&Lecturas9DOFIMU_Almacenada.LecturasBrujula, LecturasBrujula, sizeof(tpLecturasBrujula));
	GateMutexPri_leave(SERVIDOR_Datos_9DOF_IMU, Key);
}


void Leer_servidor_DCM(int16_t* DCM){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_DCM);
	arm_copy_q15(DCM_Almacenada, DCM, sizeof(DCM_Almacenada)/sizeof(float32_t));
	GateMutexPri_leave(SERVIDOR_DCM, Key);
}

void Escribir_servidor_DCM(int16_t* DCM){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_DCM);
	arm_copy_q15(DCM, DCM_Almacenada, sizeof(DCM_Almacenada)/sizeof(float32_t));
	GateMutexPri_leave(SERVIDOR_DCM, Key);
}

void Leer_servidor_RPY(float32_t *Roll, float32_t *Pitch, float32_t *Yaw){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_YPR);
	*Roll = Roll_Almacenado;
	*Pitch = Pitch_Almacenado;
	*Yaw = Yaw_Almacenado;
	GateMutexPri_leave(SERVIDOR_YPR, Key);
}

void Escribir_servidor_RPY(float32_t *Roll, float32_t *Pitch, float32_t *Yaw){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_YPR);
	Roll_Almacenado = *Roll;
	Pitch_Almacenado = *Pitch;
	Yaw_Almacenado = *Yaw;
	GateMutexPri_leave(SERVIDOR_YPR, Key);
}

void Leer_servidor_Variables_Estado_Medidas(float32_t* Variables_Estado_Medidas){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Variables_Estado_Medidas);
	arm_copy_f32(Variables_Estado_Medidas_Almacenadas, Variables_Estado_Medidas, sizeof(Variables_Estado_Medidas_Almacenadas)/sizeof(float32_t));
//	memcpy(Variables_Estado_Medidas, Variables_Estado_Medidas_Almacenadas, sizeof(Variables_Estado_Medidas_Almacenadas));
	GateMutexPri_leave(SERVIDOR_Variables_Estado_Medidas, Key);
}

void Escribir_servidor_Variables_Estado_Medidas(float32_t* Variables_Estado_Medidas){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Variables_Estado_Medidas);
	memcpy(Variables_Estado_Medidas_Almacenadas, Variables_Estado_Medidas, sizeof(Variables_Estado_Medidas_Almacenadas));
	GateMutexPri_leave(SERVIDOR_Variables_Estado_Medidas, Key);
}

void Leer_servidor_Variables_Estado_Estimadas(float32_t* Variables_Estado_Estimadas){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Variables_Estado_Estimadas);
	memcpy(Variables_Estado_Estimadas, Variables_Estado_Estimadas_Almacenadas, sizeof(Variables_Estado_Estimadas_Almacenadas));
	GateMutexPri_leave(SERVIDOR_Variables_Estado_Estimadas, Key);
}

void Escribir_servidor_Variables_Estado_Estimadas(float32_t* Variables_Estado_Estimadas){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Variables_Estado_Estimadas);
	memcpy(Variables_Estado_Estimadas_Almacenadas, Variables_Estado_Estimadas, sizeof(Variables_Estado_Estimadas_Almacenadas));
	GateMutexPri_leave(SERVIDOR_Variables_Estado_Estimadas, Key);
}

void Leer_servidor_Perturbaciones_Estimadas(float32_t* Perturbaciones_Estimadas){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Perturbaciones_Estimadas);
	arm_copy_f32(Perturbaciones_Estimadas_Almacenadas, Perturbaciones_Estimadas, sizeof(Perturbaciones_Estimadas_Almacenadas)/sizeof(float32_t));
	GateMutexPri_leave(SERVIDOR_Perturbaciones_Estimadas, Key);
}

void Escribir_servidor_Perturbaciones_Estimadas(float32_t* Perturbaciones_Estimadas){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Perturbaciones_Estimadas);
	arm_copy_f32(Perturbaciones_Estimadas, Perturbaciones_Estimadas_Almacenadas, sizeof(Perturbaciones_Estimadas_Almacenadas)/sizeof(float32_t));
	GateMutexPri_leave(SERVIDOR_Perturbaciones_Estimadas, Key);
}

void Resetear_servidor_Perturbaciones_Estimadas(){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Perturbaciones_Estimadas);
	arm_fill_f32(0, Perturbaciones_Estimadas_Almacenadas, sizeof(Perturbaciones_Estimadas_Almacenadas)/sizeof(float32_t));
	GateMutexPri_leave(SERVIDOR_Perturbaciones_Estimadas, Key);
}

void LeerVariableAHRS_Orientacion_q15(tpOrientacionAHRS* OrientacionAHRS_q15){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Orientacion_q15);
	memcpy(OrientacionAHRS_q15, &OrientacionAHRS_q15Almacenado, sizeof(OrientacionAHRS_q15Almacenado));
	GateMutexPri_leave(SERVIDOR_Orientacion_q15, Key);
}

void EscribirVariableAHRS_Orientacion_q15(tpOrientacionAHRS* OrientacionAHRS_q15){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Orientacion_q15);
	memcpy(&OrientacionAHRS_q15Almacenado, OrientacionAHRS_q15, sizeof(OrientacionAHRS_q15Almacenado));
	GateMutexPri_leave(SERVIDOR_Orientacion_q15, Key);
}

void Leer_servidor_Referencia(float32_t* Referencia , int16_t* Referencia_Entero){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Referencia);
	memcpy(Referencia, Referencia_Almacenada, sizeof(Referencia_Almacenada));
	memcpy(Referencia_Entero, Referencia_Almacenada_Entero, sizeof(Referencia_Almacenada_Entero));
	GateMutexPri_leave(SERVIDOR_Referencia, Key);
}

void Escribir_servidor_Referencia(float32_t* Referencia , int16_t* Referencia_Entero){
	IArg Key;

	Key = GateMutexPri_enter(SERVIDOR_Referencia);
	memcpy(Referencia_Almacenada, Referencia, sizeof(Referencia_Almacenada));
	memcpy(Referencia_Almacenada_Entero, Referencia_Entero, sizeof(Referencia_Almacenada_Entero));
	GateMutexPri_leave(SERVIDOR_Referencia, Key);
}

float32_t* Direccion_servidor_Referencia(){
	return Referencia_Almacenada;
}
