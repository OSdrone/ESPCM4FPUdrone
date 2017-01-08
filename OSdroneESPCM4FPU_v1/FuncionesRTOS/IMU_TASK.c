/*
 * IMU.c
 *
 *  Created on: 8/12/2016
 *      Author: Ruben
 */


#include "../Dispositivos/IMU_MPU9250.h"
#include "TIRTOS_FCN.h"

extern I2C_Handle I2C_SENSORES;
extern tpIMU9250 IMU9250;
extern tpCalibracionIMU9DOF CalibracionIMU9DOF;

void Lectura_Datos_IMU(){
	tpLecturasIMU LecturasIMU;

	int16_t LecturaOrientada_matriz[3];
	arm_matrix_instance_q15 LecturaOrientada = {3, 1, LecturaOrientada_matriz};
	int16_t Lectura_matriz[3];
	arm_matrix_instance_q15 Lectura = {3, 1, Lectura_matriz};

	static q15_t CoeficientesFiltroAceleracion[5*NUM_ETAPAS_FILTRO_ACEL_ANG] =  COEF_FILTRO_ACEL_VALUES;
	static q15_t EstadoFiltroVelocidadAceleracion [4*NUM_ETAPAS_FILTRO_ACEL_ANG][3];
	static arm_biquad_casd_df1_inst_q15 FiltroAceleracion[3] = {
			{NUM_ETAPAS_FILTRO_VEL_ANG, &EstadoFiltroVelocidadAceleracion[0][0], CoeficientesFiltroAceleracion, SHIFT_FILTRO_ACEL},
			{NUM_ETAPAS_FILTRO_VEL_ANG, &EstadoFiltroVelocidadAceleracion[0][1], CoeficientesFiltroAceleracion, SHIFT_FILTRO_ACEL},
			{NUM_ETAPAS_FILTRO_VEL_ANG, &EstadoFiltroVelocidadAceleracion[0][2], CoeficientesFiltroAceleracion, SHIFT_FILTRO_ACEL}
	};

	static q15_t CoeficientesFiltroVelocidad[5*NUM_ETAPAS_FILTRO_VEL_ANG] =  COEF_FILTRO_VEL_VALUES;
	static q15_t EstadoFiltroVelocidad [4*NUM_ETAPAS_FILTRO_VEL_ANG][3];
	static arm_biquad_casd_df1_inst_q15 FiltroVelocidad[3] = {
			{NUM_ETAPAS_FILTRO_VEL_ANG, &EstadoFiltroVelocidad[0][0], CoeficientesFiltroVelocidad, SHIFT_FILTRO_VELOCIDAD},
			{NUM_ETAPAS_FILTRO_VEL_ANG, &EstadoFiltroVelocidad[0][1], CoeficientesFiltroVelocidad, SHIFT_FILTRO_VELOCIDAD},
			{NUM_ETAPAS_FILTRO_VEL_ANG, &EstadoFiltroVelocidad[0][2], CoeficientesFiltroVelocidad, SHIFT_FILTRO_VELOCIDAD}
	};

	//Arranque del IMU


	while(1){
		Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);

		Leer_IMU_MPU9250(I2C_SENSORES, IMU9250, &LecturasIMU);
//		Mailbox_pend(Buzon_Lecturas_IMU, NULL, BIOS_NO_WAIT);
//		Mailbox_post(Buzon_Lecturas_IMU, &Lecturas_9DOF_IMU, BIOS_NO_WAIT);

		// Acel
		//Filtrar
		arm_biquad_cascade_df1_q15(&FiltroAceleracion[0], &Lectura_matriz[0], &LecturasIMU.x_acel, 1);
		arm_biquad_cascade_df1_q15(&FiltroAceleracion[1], &Lectura_matriz[1], &LecturasIMU.y_acel, 1);
		arm_biquad_cascade_df1_q15(&FiltroAceleracion[2], &Lectura_matriz[2], &LecturasIMU.z_acel, 1);
		//Alinear
		arm_mat_mult_q15(&CalibracionIMU9DOF.Correccion_Alineamiento_IMU, &Lectura, &LecturaOrientada, NULL);
		LecturasIMU.x_acel = LecturaOrientada_matriz[0];
		LecturasIMU.y_acel = LecturaOrientada_matriz[1];
		LecturasIMU.z_acel = LecturaOrientada_matriz[2];

		// Vel
		//Ofsset
		LecturasIMU.x_vel -= CalibracionIMU9DOF.Media_Vel_x;
		LecturasIMU.y_vel -= CalibracionIMU9DOF.Media_Vel_y;
		LecturasIMU.z_vel -= CalibracionIMU9DOF.Media_Vel_z;
		//Filtrar
		arm_biquad_cascade_df1_q15(&FiltroVelocidad[0], &Lectura_matriz[0], &LecturasIMU.x_vel, 1);
		arm_biquad_cascade_df1_q15(&FiltroVelocidad[1], &Lectura_matriz[1], &LecturasIMU.y_vel, 1);
		arm_biquad_cascade_df1_q15(&FiltroVelocidad[2], &Lectura_matriz[2], &LecturasIMU.z_vel, 1);
		//Alinear
		arm_mat_mult_q15(&CalibracionIMU9DOF.Correccion_Alineamiento_IMU, &Lectura, &LecturaOrientada, NULL);
		LecturasIMU.x_vel = LecturaOrientada_matriz[0];
		LecturasIMU.y_vel = LecturaOrientada_matriz[1];
		LecturasIMU.z_vel = LecturaOrientada_matriz[2];

		//Temp

		Escribir_servidor_Lecturas_IMU(&LecturasIMU);
	}
}

void CLK_Lectura_Datos_IMU(){
	Semaphore_post(SEMAPHORE_Leer_IMU);
}
