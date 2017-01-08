/*
 * AHRS_TASK.c
 *
 *  Created on: 31/12/2016
 *      Author: Ruben
 */

#include "TIRTOS_FCN.h"
#include "../AHRS.h"
#include "../OSmath.h"

extern tpIMU9250 IMU9250;
tpAHRS AHRS;

void Calculo_AHRS(){

	tpAHRS AHRS;
	q31_t OrientacionMagnetica = 0;
	q16_4_t VelocidadAngular[3];
	tpLecturas9DOFIMU Lecturas9DOFIMU;

	uint16_t SensibilidadGiroscopo = 32768/IMU9250.Sensibilidad_Giroscopo;
	while(1){
		Semaphore_pend(SEMAPHORE_Calculo_AHRS, BIOS_WAIT_FOREVER);

		Leer_servidor_Lecturas_IMU_9DOF(&Lecturas9DOFIMU);
		LeerVariableAHRS_Orientacion_q15(&AHRS.OrientacionAHRS);

		Compensacion_Sensor_magnetico(AHRS.OrientacionAHRS.Roll, AHRS.OrientacionAHRS.Pitch, &Lecturas9DOFIMU.LecturasBrujula.x_mag);

		//Conversion a º/s
		VelocidadAngular[0] = (q16_4_t)(((int32_t)Lecturas9DOFIMU.LecturasIMU.x_vel * SensibilidadGiroscopo)>>12);
		VelocidadAngular[1] = (q16_4_t)(((int32_t)Lecturas9DOFIMU.LecturasIMU.y_vel * SensibilidadGiroscopo)>>12);
		VelocidadAngular[2] = (q16_4_t)(((int32_t)Lecturas9DOFIMU.LecturasIMU.z_vel * SensibilidadGiroscopo)>>12);

		Actualizar_Matriz_DCM_V2(&AHRS, VelocidadAngular);
		Normalizar_DCM(&AHRS);
		Correccion_deriva(&AHRS, (q15_t*)&Lecturas9DOFIMU.LecturasIMU.x_acel, OrientacionMagnetica);
		Angulos_Euler(&AHRS);

		EscribirVariableAHRS_Orientacion_q15(&AHRS.OrientacionAHRS);
	}
}

void CLK_Calculo_AHRS(){
	Semaphore_post(SEMAPHORE_Calculo_AHRS);
}

void ResetAHRS(){
	AHRS.OrientacionAHRS.Pitch = 0;
	AHRS.OrientacionAHRS.Roll = 0;
	AHRS.OrientacionAHRS.Yaw = 0;

	arm_fill_q15(0, (q15_t*)AHRS.OrientacionAHRS.DCM_matriz, 9);
	AHRS.OrientacionAHRS.DCM_matriz[0][0] = 32767;
	AHRS.OrientacionAHRS.DCM_matriz[1][1] = 32767;
	AHRS.OrientacionAHRS.DCM_matriz[2][2] = 32767;
}
void setAHRS(tpAHRS *AHRSset){
	memcpy(&AHRS, AHRSset, sizeof(tpAHRS));
}
