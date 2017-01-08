/*
 * Control.c
 *
 *  Created on: 8/12/2016
 *      Author: Ruben
 */

#include "TIRTOS_FCN.h"

#include "../Funciones_Transferencia.h"

extern tpIMU9250 IMU9250;

void Control(){

	float32_t Roll, Pitch, Yaw;
	tpLecturas9DOFIMU Lecturas9DOFIMU;

	float32_t Ref_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Ref = {4, 1, Ref_matriz};

	float32_t Accion_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Accion = {4, 1, Accion_matriz};

	float32_t Variables_medidas_matriz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Variables_medidas = {10, 1, Variables_medidas_matriz};

	float32_t Variables_predichas_matriz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Variables_predichas = {10, 1, Variables_predichas_matriz};

	float32_t Variables_estimadas_matriz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Variables_estimadas = {10, 1, Variables_estimadas_matriz};

	float32_t Perturbaciones_estimadas_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Perturbaciones_estimadas = {4, 1, Perturbaciones_estimadas_matriz};

	float32_t Perturbaciones_calculadas_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Perturbaciones_calculadas = {4, 1, Perturbaciones_calculadas_matriz};

	float32_t Aux_Matriz[100];
	arm_matrix_instance_f32 Aux = {10, 10, Aux_Matriz};

	float32_t Aux2_Matriz[100];
	arm_matrix_instance_f32 Aux2 = {10, 10, Aux2_Matriz};

#ifdef Filtro_Perturbaciones

	float32_t Estado_filtro_Per_0[4*num_etapas_Filtro_Per];
	float32_t Estado_filtro_Per_1[4*num_etapas_Filtro_Per];
	float32_t Estado_filtro_Per_2[4*num_etapas_Filtro_Per];
	float32_t Estado_filtro_Per_3[4*num_etapas_Filtro_Per];

	arm_biquad_casd_df1_inst_f32 Filtro_Per_0 = {num_etapas_Filtro_Per, Estado_filtro_Per_0, (float32_t *)Coeficientes_Filtro_Pre_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Per_1 = {num_etapas_Filtro_Per, Estado_filtro_Per_1, (float32_t *)Coeficientes_Filtro_Pre_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Per_2 = {num_etapas_Filtro_Per, Estado_filtro_Per_2, (float32_t *)Coeficientes_Filtro_Pre_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Per_3 = {num_etapas_Filtro_Per, Estado_filtro_Per_3, (float32_t *)Coeficientes_Filtro_Pre_Valores};

	//Inicializamos el filtro
	arm_fill_f32(0.0, Estado_filtro_Per_0, 4*num_etapas_Filtro_Per);
	arm_fill_f32(0.0, Estado_filtro_Per_1, 4*num_etapas_Filtro_Per);
	arm_fill_f32(0.0, Estado_filtro_Per_2, 4*num_etapas_Filtro_Per);
	arm_fill_f32(0.0, Estado_filtro_Per_3, 4*num_etapas_Filtro_Per);
#endif

	while(1){
		Semaphore_pend(SEMAPHORE_Control, BIOS_WAIT_FOREVER);

		Leer_servidor_Referencia(Ref.pData, NULL);
		Leer_servidor_RPY(&Roll, &Pitch, &Yaw);
		Leer_servidor_Lecturas_IMU_9DOF(&Lecturas9DOFIMU);
		Leer_servidor_Variables_Estado_Estimadas(Variables_estimadas.pData);
		Leer_servidor_Perturbaciones_Estimadas(Perturbaciones_estimadas.pData);


//.....Sensado..Variables.....//
		//Ajuste posicion inicial//
		Variables_medidas.pData[1] = Pitch;
		Variables_medidas.pData[3] = Roll;
		Variables_medidas.pData[5] = Yaw;


		Variables_medidas.pData[0] = Lecturas9DOFIMU.LecturasIMU.x_vel / IMU9250.Sensibilidad_Giroscopo;
		Variables_medidas.pData[2] = Lecturas9DOFIMU.LecturasIMU.y_vel / IMU9250.Sensibilidad_Giroscopo;
		Variables_medidas.pData[4] = Lecturas9DOFIMU.LecturasIMU.z_vel / IMU9250.Sensibilidad_Giroscopo;


		if(Modo_Control == ANGULOS_3){
			switch(ModoPerturbaciones){
				case NO_CORREGIR_PERTURBACIONES:
				case CORREGIR_PERTURBACIONES:
				case INTEGRAR_PERTURBACIONES_ESTIMADAS:
					Variables_medidas.pData[9] = Variables_predichas.pData[9];
				break;
				case INTEGRAR_PERTURBACIONES:
					Variables_medidas.pData[9] = Ref.pData[3];
			}
		}
		Escribir_servidor_Variables_Estado_Medidas(Variables_medidas.pData);

//Estimar variables estado

		Aux.numRows = Variables_medidas.numRows;
		Aux.numCols = Variables_predichas.numCols;
		arm_mat_sub_f32(&Variables_medidas, &Variables_predichas, &Aux);

		Aux2.numRows = 10;
		Aux2.numCols = 1;
		arm_mat_mult_f32(&Lo_per, &Aux, &Aux2);

		arm_mat_add_f32(&Variables_predichas, &Aux2, &Variables_estimadas);
		Escribir_servidor_Variables_Estado_Estimadas(Variables_estimadas.pData);

//Estimar perturbacion
		switch(ModoPerturbaciones){
			case NO_CORREGIR_PERTURBACIONES:
				break;
			case CORREGIR_PERTURBACIONES:

				Aux.numRows = Variables_medidas.numRows;
				Aux.numCols = Variables_predichas.numCols;
				arm_mat_sub_f32(&Variables_medidas, &Variables_predichas, &Aux);

				Aux2.numRows = Aux2.numRows;
				Aux2.numCols = Aux2.numCols;
				arm_mat_mult_f32(&Lp, &Aux, &Aux2);

				Aux.numRows = Aux.numRows;
				Aux.numCols = Aux.numCols;
				arm_mat_add_f32(&Perturbaciones_estimadas, &Aux2, &Aux);

				//A 0 la estimacion de Empuje
//				Aux.pData[3] = 0;

#ifndef Filtro_Perturbaciones
				arm_copy_f32(Aux.pData, Perturbaciones_estimadas.pData, sizeof(Perturbaciones_estimadas_matriz)/sizeof(float32_t));
#else
				//		Filtar_Perturbacion
				arm_biquad_cascade_df1_f32(&Filtro_Per_0, &Aux.pData[0], &Perturbaciones_estimadas.pData[0], 1);
				arm_biquad_cascade_df1_f32(&Filtro_Per_1, &Aux.pData[1], &Perturbaciones_estimadas.pData[1], 1);
				arm_biquad_cascade_df1_f32(&Filtro_Per_2, &Aux.pData[2], &Perturbaciones_estimadas.pData[2], 1);
				arm_biquad_cascade_df1_f32(&Filtro_Per_3, &Aux.pData[3], &Perturbaciones_estimadas.pData[3], 1);
#endif

			break;
			case INTEGRAR_PERTURBACIONES:

				Perturbaciones_estimadas.pData[0] += Ki * (Ref.pData[0] - Variables_medidas.pData[1]);
				Perturbaciones_estimadas.pData[1] += Ki * (Ref.pData[1] - Variables_medidas.pData[3]);
				Perturbaciones_estimadas.pData[2] += Ki * (Ref.pData[2] - Variables_medidas.pData[5]);
				Perturbaciones_estimadas.pData[3] += Ki * (Ref.pData[3] - Variables_medidas.pData[9]);
			break;

			case INTEGRAR_PERTURBACIONES_ESTIMADAS:

				Perturbaciones_estimadas.pData[0] += Ki_EST * (Variables_predichas.pData[1] - Variables_medidas.pData[1]);
				Perturbaciones_estimadas.pData[1] += Ki_EST * (Variables_predichas.pData[3] - Variables_medidas.pData[3]);
				Perturbaciones_estimadas.pData[2] += Ki_EST * (Variables_predichas.pData[5] - Variables_medidas.pData[5]);
				Perturbaciones_estimadas.pData[3] += Ki_EST * (Variables_predichas.pData[9] - Variables_medidas.pData[9]);
			break;
		}

				//Prealimentar perturbaciones conocidas
		Perturbaciones_calculadas.pData[0] = Perturbaciones_estimadas.pData[0];// + Pert_Fuerza_Bateria*sin(Variables_estimadas.pData[1]*PI/180.0);
		Perturbaciones_calculadas.pData[1] = Perturbaciones_estimadas.pData[1];// + Pert_Fuerza_Bateria*sin(Variables_estimadas.pData[3]*PI/180.0);
		Perturbaciones_calculadas.pData[2] = Perturbaciones_estimadas.pData[2];
		Perturbaciones_calculadas.pData[3] = Perturbaciones_estimadas.pData[3];

		 Escribir_servidor_Perturbaciones_Estimadas(Perturbaciones_estimadas.pData);

//Accion
		switch(ModoPerturbaciones){
			case NO_CORREGIR_PERTURBACIONES:
				Aux.numRows = K_pre_4.numRows;
				Aux.numCols = Ref.numCols;

				Aux2.numRows = K_4.numRows;
				Aux2.numCols = Variables_estimadas.numCols;

				switch(Modo_Control){
					case ANGULOS_4:
						arm_mat_mult_f32(&K_pre_4, &Ref, &Aux);
						arm_mat_mult_f32(&K_4, &Variables_estimadas, &Aux2);
						arm_mat_sub_f32(&Aux, &Aux2, &Accion);
					break;

					case ANGULOS_3:
						arm_mat_mult_f32(&K_pre_3, &Ref, &Aux);
						arm_mat_mult_f32(&K_3, &Variables_estimadas, &Aux2);
						arm_mat_sub_f32(&Aux, &Aux2, &Accion);
					break;

					case EMPUJE:
						Accion_matriz[0] =  Ref.pData[3]; //
						Accion_matriz[1] =  Ref.pData[3]; //
						Accion_matriz[2] =  Ref.pData[3]; //
						Accion_matriz[3] =  Ref.pData[3]; //
					break;
				}
			break;

			case CORREGIR_PERTURBACIONES:
			case INTEGRAR_PERTURBACIONES:
			case INTEGRAR_PERTURBACIONES_ESTIMADAS:

				Aux.numRows = K_4.numRows;
				Aux.numCols = Variables_estimadas.numCols;

				Aux2.numRows = Accion.numRows;
				Aux2.numCols = Aux.numCols;

				switch(Modo_Control){
					case ANGULOS_4:
						arm_mat_mult_f32(&K_pre_4, &Ref, &Accion);
						arm_mat_mult_f32(&K_4, &Variables_estimadas, &Aux);

						arm_mat_sub_f32(&Accion, &Aux, &Aux2);

						Aux.numRows = La.numRows;
						Aux.numCols = Perturbaciones_calculadas.numCols;
						arm_mat_mult_f32(&La, &Perturbaciones_calculadas, &Aux);

						arm_mat_sub_f32(&Aux2, &Aux, &Accion);
					break;

					case ANGULOS_3:
						arm_mat_mult_f32(&K_pre_3, &Ref, &Accion);
						arm_mat_mult_f32(&K_3, &Variables_estimadas, &Aux);

						arm_mat_sub_f32(&Accion, &Aux, &Aux2);

						Aux.numRows = La.numRows;
						Aux.numCols = Perturbaciones_calculadas.numCols;
						arm_mat_mult_f32(&La, &Perturbaciones_calculadas, &Aux);

						arm_mat_sub_f32(&Aux2, &Aux, &Accion);
					break;
					case EMPUJE:
						Accion_matriz[0] =  Ref.pData[3]; //
						Accion_matriz[1] =  Ref.pData[3]; //
						Accion_matriz[2] =  Ref.pData[3]; //
						Accion_matriz[3] =  Ref.pData[3]; //
					break;
				}
			break;

		}

		if(Accion.pData[0] < Accion_Minima )
			Accion.pData[0] = Accion_Minima;
		if(Accion.pData[1] < Accion_Minima )
			Accion.pData[1] = Accion_Minima;
		if(Accion.pData[2] < Accion_Minima )
			Accion.pData[2] = Accion_Minima;
		if(Accion.pData[3] < Accion_Minima )
			Accion.pData[3] = Accion_Minima;

		if(Accion.pData[0] > Accion_Maxima )
			Accion.pData[0] = Accion_Maxima;
		if(Accion.pData[1] > Accion_Maxima )
			Accion.pData[1] = Accion_Maxima;
		if(Accion.pData[2] > Accion_Maxima )
			Accion.pData[2] = Accion_Maxima;
		if(Accion.pData[3] > Accion_Maxima )
			Accion.pData[3] = Accion_Maxima;

//Aplicar U
		PWM_setDuty(PWM0, (uint32_t)Accion.pData[0] + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM1, (uint32_t)Accion.pData[1] + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM2, (uint32_t)Accion.pData[2] + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM3, (uint32_t)Accion.pData[3] + Pulso_minimo_PWM_motor);

//Predecir estado
		arm_mat_mult_f32(&F, &Variables_estimadas, &Variables_predichas);

		Aux.numRows = G.numRows;
		Aux.numCols = Accion.numCols;
		arm_mat_mult_f32(&G, &Accion, &Aux);

		Aux2.numRows = Variables_predichas.numRows;
		Aux2.numCols = Aux.numCols;
		arm_mat_add_f32(&Variables_predichas, &Aux, &Aux2);

		Aux.numRows = Gp.numRows;
		Aux.numCols = Perturbaciones_calculadas.numCols;
		arm_mat_mult_f32(&Gp, &Perturbaciones_calculadas, &Aux);

		arm_mat_add_f32(&Aux2, &Aux, &Variables_predichas);

	}
}
void CLK_Control(){
	Semaphore_post(SEMAPHORE_Control);
}
