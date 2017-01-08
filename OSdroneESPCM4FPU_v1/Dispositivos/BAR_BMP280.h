/*
 * BAR_BMP280.h
 *
 *  Created on: 30/12/2016
 *      Author: Ruben
 */

#ifndef DISPOSITIVOS_BAR_BMP280_H_
#define DISPOSITIVOS_BAR_BMP280_H_

/*...BMP280..*/
#define BMP_280_DIR_0 0x76
#define BMP_280_DIR_1 0x77

#define BMP_280_calibracion_T1
#define BMP_280_press_msb 0xF7
#define BMP_280_ctrl_meas 0xF4

typedef struct { //tpLecturasBarometro
	uint32_t UP;
	uint16_t UT;

	uint32_t Presion;
	float Temperatura;

	//Parametros de calibracion
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
} tpLecturasBarometro;

typedef struct { //tpLecturasBarometro_BMP280

	uint32_t Presion;
	uint32_t Temperatura;

	uint16_t dig_T1;
	uint16_t dig_T2;
	uint16_t dig_T3;

	uint16_t dig_P1;
	uint16_t dig_P2;
	uint16_t dig_P3;
	uint16_t dig_P4;
	uint16_t dig_P5;
	uint16_t dig_P6;
	uint16_t dig_P7;
	uint16_t dig_P8;
	uint16_t dig_P9;
} tpLecturasBarometro_BMP280;

typedef enum {
	x1, x2, x4, x8, x16
} OverSampling_BMP280;

typedef struct {
	uint8_t Direccion;
	enum {
		Mode_Sleep = 0, Mode_Forced, Mode_Normal
	} Modo;
	OverSampling_BMP280 Oversampling_Presion;
	OverSampling_BMP280 Oversampling_Temperatura;
	enum {
		Filtro_off = 0, Filtro_2, Filtro_4, Filtro_8, Filtro_16
	} Filtro_BMP280;
	enum {
		ms05 = 0, ms625, ms125, ms250, ms500, ms1000, ms2000, ms4000
	} t_sampling;
} tpBarometro_BMP280;

#endif /* DISPOSITIVOS_BAR_BMP280_H_ */
