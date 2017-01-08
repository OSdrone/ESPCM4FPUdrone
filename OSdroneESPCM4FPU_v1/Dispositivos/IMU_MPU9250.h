/*
 * IMU.h
 *
 *  Created on: 13 de oct. de 2016
 *      Author: DTUser
 */

#ifndef DISPOSITIVOS_IMU_H_
#define DISPOSITIVOS_IMU_H_


#include "stdbool.h"
#include <ti/drivers/I2C.h>
#include "arm_math.h"
#include "math.h"
#include "../Constantes.h"

#define CONVERTIR_A_RADIANES(DEGS) PI/180.0*DEGS
#define CONVERTIR_A_GRADOS(RADS) 180.0/PI*RADS

//..........9DOF_IMU..............................//

//..............................IMU_9250........................................//
typedef enum {
	DLPF_CFG_NO = -1,
	DLPF_CFG_0,
	DLPF_CFG_1,
	DLPF_CFG_2,
	DLPF_CFG_3,
	DLPF_CFG_4,
	DLPF_CFG_5,
	DLPF_CFG_6,
	DLPF_CFG_7
} tpDLPF_CFG;

typedef enum {
	Gain_Gyro_250, Gain_Gyro_500, Gain_Gyro_1000, Gain_Gyro_2000
} tpGanancia_Gyro;
typedef enum {
	DLPF_CFG_GYRO_NO = -1,
	DLPF_CFG_GYRO_0,
	DLPF_CFG_GYRO_1,
	DLPF_CFG_GYRO_2,
	DLPF_CFG_GYRO_3,
	DLPF_CFG_GYRO_4,
	DLPF_CFG_GYRO_5,
	DLPF_CFG_GYRO_6,
	DLPF_CFG_GYRO_7
} tpDLPF_CFG_GYRO;
typedef enum {
	Gain_Acel_2G, Gain_Acel_4G, Gain_Acel_8G, Gain_Acel_16G
} tpGanancia_Acel;
typedef enum {
	DLPF_CFG_ACEL_NO = -1,
	DLPF_CFG_ACEL_0,
	DLPF_CFG_ACEL_1,
	DLPF_CFG_ACEL_2,
	DLPF_CFG_ACEL_3,
	DLPF_CFG_ACEL_4,
	DLPF_CFG_ACEL_5,
	DLPF_CFG_ACEL_6,
	DLPF_CFG_ACEL_7
} tpDLPF_CFG_ACEL;

#define Dir_0_IMU_MPU9250 0x68
#define Dir_1_IMU_MPU9250 0x69
#define Dir_MAG_MPU9250 0x0C

#define IMU_MPU9250_INT_PIN_CFG 0x37
#define IMU_MPU9250_SMPLRT_DIV 0x19
#define IMU_MPU9250_USER_CTRL 0x6A
#define IMU_MPU9250_PWR_MGMT_1 0x6B
#define IMU_MPU9250_MAG_CTL1 0x0A
#define IMU_MPU9250_MAG_HXL 0x03

#define IMU_MPU6050_ACCEL_XOUT_H 0x3B

typedef struct {
	uint8_t Direccion_IMU;
	uint8_t Direccion_MAG;
	uint8_t SMPLRT_DIV;
	float32_t Sensibilidad_Giroscopo;
	float32_t Sensibilidad_Acel;
	float32_t Sensibilidad_Brujula;

	tpDLPF_CFG_GYRO DLPF_CFG_GYRO;
	tpGanancia_Gyro Ganancia_Gyro;
	tpDLPF_CFG_ACEL DLPF_CFG_ACEL;
	tpGanancia_Acel Ganancia_Acel;
} tpIMU9250;


typedef struct {
	int16_t x_acel;
	int16_t y_acel;
	int16_t z_acel;

	int16_t temp;

	int16_t x_vel;
	int16_t y_vel;
	int16_t z_vel;
}tpLecturasIMU;

typedef struct{
	int16_t  x_mag;
	int16_t  y_mag;
	int16_t  z_mag;
}tpLecturasBrujula;

typedef struct{
	tpLecturasIMU LecturasIMU;
	tpLecturasBrujula LecturasBrujula;
}tpLecturas9DOFIMU;

typedef struct { //tpCalibracion_IMU
//	int16_t Rango_Acel_x[2]; // Min Max
	int16_t Media_Acel_x;
	float32_t Des_est_Acel_x;
//	int16_t Rango_Acel_y[2];
	int16_t Media_Acel_y;
	float32_t Des_est_Acel_y;
//	int16_t Rango_Acel_z[2];
	int16_t Media_Acel_z;
	float32_t Des_est_Acel_z;

	int16_t Media_Temp;
	float32_t Des_est_Temp;

//	int16_t Rango_Vel_x[2];
	int16_t Media_Vel_x;
	float32_t Des_est_Vel_x;
//	int16_t Rango_Vel_y[2];
	int16_t Media_Vel_y;
	float32_t Des_est_Vel_y;
//	int16_t Rango_Vel_z[2];
	int16_t Media_Vel_z;
	float32_t Des_est_Vel_z;

//	int16_t Rango_Mag_x[2];
	int16_t Media_Mag_x;
	float32_t Des_est_Mag_x;
//	int16_t Rango_Mag_y[2];
	int16_t Media_Mag_y;
	float32_t Des_est_Mag_y;
//	int16_t Rango_Mag_z[2];
	int16_t Media_Mag_z;
	float32_t Des_est_Mag_z;

	q15_t Correccion_Alineamiento_IMU_matriz[9];
	arm_matrix_instance_q15 Correccion_Alineamiento_IMU;
	q15_t Correccion_COMPASS_matriz[9];
	arm_matrix_instance_q15 Correccion_COMPASS;

} tpCalibracionIMU9DOF;

bool Iniciar_IMU_MPU9250(I2C_Handle I2C, tpIMU9250 IMU92520);
bool Leer_IMU_MPU9250(I2C_Handle I2C, tpIMU9250 IMU92520, tpLecturasIMU *LecturasIMU);

#endif
