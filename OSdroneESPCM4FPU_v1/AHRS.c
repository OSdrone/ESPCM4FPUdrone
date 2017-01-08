/*
 * AHRS.c
 *
 *  Created on: 22/12/2015
 *      Author: Ruben
 */
#include "AHRS.h"


q31_t Compensacion_Sensor_magnetico(q15_t Roll, q15_t Pitch, int16_t VectorMagnetico[3]){
	q31_t SenRoll = 0;
	q31_t CosRoll = 0;
	q31_t SenPitch = 0;
	q31_t CosPitch = 0;

	q31_t MagX = 0;
	q31_t MagY = 0;

	arm_sin_cos_q31(Roll<<16, &SenRoll, &CosRoll);
	arm_sin_cos_q31(Pitch<<16, &SenPitch, &CosPitch);

	//Pasamos a q15
	SenRoll= SenRoll>>16;
	CosRoll= CosRoll>>16;
	SenPitch= SenPitch>>16;
	CosPitch= CosPitch>>16;

	MagY = ((CosRoll*VectorMagnetico[1] - SenRoll*VectorMagnetico[2])>>16);
	MagX = ((( (CosPitch*VectorMagnetico[0] + SenRoll*VectorMagnetico[1]) +
			 (CosRoll*VectorMagnetico[2]) )>>15) * SenPitch);

	return (q15_t)(2147483648 * atan2(-MagY, MagX));
}

void Actualizar_Matriz_DCM_V2(tpAHRS *AHRS, q16_4_t VelocidadAngular[3]){
	q16_4_t VelocidadAngularTotal[3] = { 0, 0, 0 };
	q15_t Vector_Rotacion[3] = { 0, 0, 0 };
	q16_3_t VelocidadAngularMagnitud = 0;
	q31_t Seno = 0;
	q31_t Coseno = 0;

	uint32_t Aux32 = 0;

	q15_t Rot_matriz[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	arm_matrix_instance_q15 Rotacion = { 3, 3, (q15_t *) Rot_matriz };
	q15_t Aux_matriz[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	arm_matrix_instance_q15 Aux = { 3, 3, (q15_t *) Aux_matriz };

	VelocidadAngularTotal[0] = VelocidadAngular[0]
			+ AHRS->ConfiguracionAHRS.Correccion_Proporcional[0]
			+ AHRS->ConfiguracionAHRS.Correccion_Integral[0];
	VelocidadAngularTotal[1] = VelocidadAngular[1]
			+ AHRS->ConfiguracionAHRS.Correccion_Proporcional[1]
			+ AHRS->ConfiguracionAHRS.Correccion_Integral[1];
	VelocidadAngularTotal[2] = VelocidadAngular[2]
			+ AHRS->ConfiguracionAHRS.Correccion_Proporcional[2]
			+ AHRS->ConfiguracionAHRS.Correccion_Integral[2];

	Aux32 = ((int32_t)VelocidadAngularTotal[0] * VelocidadAngularTotal[0]);
	Aux32 += ((int32_t)VelocidadAngularTotal[1] * VelocidadAngularTotal[1]);
	Aux32 += ((int32_t)VelocidadAngularTotal[2] * VelocidadAngularTotal[2]);
	//Velocidad en formato q32_8


	VelocidadAngularMagnitud = (q16_3_t)((2^3) * (q16_3_t)sqrtf(Aux32/256.0));
//	arm_sqrt_q31(Aux32, &Aux32); //TODO revisar coma fija al hacer sqtr, resultado de q32_6 q32_12?
//	VelocidadAngularMagnitud = (q16_3_t)(Aux32>>(12-3));

	//Normalizamos el vector de rotacion
	Vector_Rotacion[0] = (q15_t)((int32_t)(VelocidadAngularTotal[0]<<16) / VelocidadAngularMagnitud);
	Vector_Rotacion[1] = (q15_t)((int32_t)(VelocidadAngularTotal[1]<<16) / VelocidadAngularMagnitud);
	Vector_Rotacion[2] = (q15_t)((int32_t)(VelocidadAngularTotal[2]<<16) / VelocidadAngularMagnitud);

	//Usamos VelocidadAngularMagnitud como angulo
	Aux32 = ((q31_t)VelocidadAngularMagnitud * AHRS->ConfiguracionAHRS.TiempoMuestreo)<<12;//(31-(16+3))
	arm_sin_cos_q31(Aux32, &Seno, &Coseno);
	Seno>>=16;
	Coseno>>=16;


	Rot_matriz[0][0] = (q15_t)(Coseno + (((((int32_t)Vector_Rotacion[0]*Vector_Rotacion[0])>>15) * (32767 - Coseno))>>15));
	Rot_matriz[0][1] = (q15_t)((((int32_t)Vector_Rotacion[0]*Vector_Rotacion[1])>>15) * (32767 - Coseno) - (Seno * Vector_Rotacion[2]))>>15;
	Rot_matriz[0][2] = (q15_t)((((int32_t)Vector_Rotacion[0]*Vector_Rotacion[2])>>15) * (32767 - Coseno) + (Seno * Vector_Rotacion[1]))>>15;

	Rot_matriz[1][0] = (q15_t)((((int32_t)Vector_Rotacion[0]*Vector_Rotacion[1])>>15) * (32767 - Coseno) + (Seno * Vector_Rotacion[2]))>>15;
	Rot_matriz[1][1] = (q15_t)(Coseno + (((((int32_t)Vector_Rotacion[1]*Vector_Rotacion[1])>>15) * (32767 - Coseno))>>15));
	Rot_matriz[1][2] = (q15_t)((((int32_t)Vector_Rotacion[1]*Vector_Rotacion[2])>>15) * (32767 - Coseno) - (Seno * Vector_Rotacion[0]))>>15;

	Rot_matriz[2][0] = (q15_t)((((int32_t)Vector_Rotacion[0]*Vector_Rotacion[2])>>15) * (32767 - Coseno) - (Seno * Vector_Rotacion[1]))>>15;
	Rot_matriz[2][1] = (q15_t)((((int32_t)Vector_Rotacion[1]*Vector_Rotacion[2])>>15) * (32767 - Coseno) + (Seno * Vector_Rotacion[0]))>>15;
	Rot_matriz[2][2] = (q15_t)(Coseno + (((((int32_t)Vector_Rotacion[2]*Vector_Rotacion[2])>>15) * (32767 - Coseno))>>15));

	arm_mat_mult_q15(&AHRS->OrientacionAHRS.DCM, &Rotacion, &Aux, NULL);
	arm_copy_q15((q15_t *) Aux_matriz, (q15_t *) &AHRS->OrientacionAHRS.DCM_matriz, 9);
}

void Normalizar_DCM(tpAHRS *AHRS) {
	q63_t error = 0;

	q15_t Vector_Aux[3] = { 0, 0, 0 };
	q15_t Matriz_Ortogonal[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

	arm_dot_prod_q15(&AHRS->OrientacionAHRS.DCM_matriz[0][0], &AHRS->OrientacionAHRS.DCM_matriz[1][0], 3, &error);
	error = error >> 1;
	error = -error;

	error = error>>(63-15);
	arm_scale_q15(&AHRS->OrientacionAHRS.DCM_matriz[1][0], (q15_t)error, 0, Vector_Aux, 3);
	arm_add_q15(&AHRS->OrientacionAHRS.DCM_matriz[0][0], Vector_Aux, &Matriz_Ortogonal[0][0], 3); //Vector X ortogonal

	arm_scale_q15(&AHRS->OrientacionAHRS.DCM_matriz[0][0], (q15_t)error, 0, Vector_Aux, 3);
	arm_add_q15(&AHRS->OrientacionAHRS.DCM_matriz[1][0], Vector_Aux, &Matriz_Ortogonal[1][0], 3); //Vector Y ortogonal

	//Producto Cruz
	Matriz_Ortogonal[2][0] = (q15_t)( ((int32_t)Matriz_Ortogonal[0][1]*Matriz_Ortogonal[1][2]) -
			((int32_t)Matriz_Ortogonal[0][2] * Matriz_Ortogonal[1][1]) )>>15;
	Matriz_Ortogonal[2][1] = (q15_t)( ((int32_t)Matriz_Ortogonal[0][2]*Matriz_Ortogonal[1][0]) -
				((int32_t)Matriz_Ortogonal[0][0] * Matriz_Ortogonal[1][2]) )>>15;
	Matriz_Ortogonal[2][2] = (q15_t)( ((int32_t)Matriz_Ortogonal[0][0]*Matriz_Ortogonal[1][1]) -
				((int32_t)Matriz_Ortogonal[0][1] * Matriz_Ortogonal[1][0]) )>>15;


	arm_dot_prod_q15(&Matriz_Ortogonal[0][0], &Matriz_Ortogonal[0][0], 3, &error);
	error = error>>(63-15);
	//error = (q15_t) ((error >> 1) * (3 - error) >> 15);
	error = (q15_t) ((int32_t)(error>>1) * ((int32_t)(3*32767)/8 - error/8))>>(15-3);
	arm_scale_q15(&Matriz_Ortogonal[0][0], error, 0, &AHRS->OrientacionAHRS.DCM_matriz[0][0], 3);

	arm_dot_prod_q15(&Matriz_Ortogonal[1][0], &Matriz_Ortogonal[1][0], 3, &error);
	error = error>>(63-15);
	//error = (q15_t) ((error >> 1) * (3 - error) >> 15);
	error = (q15_t) ((int32_t)(error>>1) * ((int32_t)(3*32767)/8 - error/8))>>(15-3);
	arm_scale_q15(&Matriz_Ortogonal[0][0], error, 0, &AHRS->OrientacionAHRS.DCM_matriz[0][0], 3);

	arm_dot_prod_q15(&Matriz_Ortogonal[2][0], &Matriz_Ortogonal[2][0], 3, &error);
	error = error>>(63-15);
	//error = (q15_t) ((error >> 1) * (3 - error) >> 15);
	error = (q15_t) ((int32_t)(error>>1) * ((int32_t)(3*32767)/8 - error/8))>>(15-3);
	arm_scale_q15(&Matriz_Ortogonal[0][0], error, 0, &AHRS->OrientacionAHRS.DCM_matriz[0][0], 3);

}

void Correccion_deriva(tpAHRS *AHRS, q15_t AceleracionLineal[3], q31_t OrientacionMagentica) {
	q15_t error[3] = { 0, 0, 0 };
	q15_t Aux[3] = { 0, 0, 0 };

	q31_t Seno = 0;
	q31_t Coseno = 0;
	//ROLL PITCH
	//faltaria filtrar???

	//Producto cruz
	error[0] = (q15_t)((int32_t)AceleracionLineal[1] * AHRS->OrientacionAHRS.DCM_matriz[2][2] -
			           (int32_t)AceleracionLineal[2] * AHRS->OrientacionAHRS.DCM_matriz[2][1])>>15;
	error[1] = (q15_t)((int32_t)AceleracionLineal[2] * AHRS->OrientacionAHRS.DCM_matriz[2][0] -
				       (int32_t)AceleracionLineal[0] * AHRS->OrientacionAHRS.DCM_matriz[2][2])>>15;
	error[2] = (q15_t)((int32_t)AceleracionLineal[0] * AHRS->OrientacionAHRS.DCM_matriz[2][1] -
				       (int32_t)AceleracionLineal[1] * AHRS->OrientacionAHRS.DCM_matriz[2][0])>>15;

	arm_scale_q15(error, AHRS->ConfiguracionAHRS.Kp_Roll_Pitch, (int8_t)SHIFT_GANANCIA_AHRS, AHRS->ConfiguracionAHRS.Correccion_Proporcional, 3);
	arm_scale_q15(error, AHRS->ConfiguracionAHRS.Ki_Roll_Pitch, (int8_t)SHIFT_GANANCIA_AHRS, Aux, 3);
	arm_add_q15(Aux, AHRS->ConfiguracionAHRS.Correccion_Integral,AHRS->ConfiguracionAHRS.Correccion_Integral, 3);


//YAW
	arm_sin_cos_q31(OrientacionMagentica, &Seno, &Coseno);
	Aux[0] = (q15_t)((((Seno>>15) * AHRS->OrientacionAHRS.DCM_matriz[0][0]) - ((Coseno>>15) * AHRS->OrientacionAHRS.DCM_matriz[1][0]))>>16);
	arm_scale_q15(&AHRS->OrientacionAHRS.DCM_matriz[2][0], Aux[0], 0, error, 3);

	arm_scale_q15(error, AHRS->ConfiguracionAHRS.Kp_Yaw, 0, Aux, 3);
	arm_add_q15(Aux, AHRS->ConfiguracionAHRS.Correccion_Proporcional,AHRS->ConfiguracionAHRS.Correccion_Proporcional, 3);

	arm_scale_q15(error, AHRS->ConfiguracionAHRS.Ki_Yaw, 0, Aux, 3);
	arm_add_q15(Aux, AHRS->ConfiguracionAHRS.Correccion_Integral,AHRS->ConfiguracionAHRS.Correccion_Integral, 3);
}

void Angulos_Euler(tpAHRS *AHRS) {
	AHRS->OrientacionAHRS.Pitch = -asin(AHRS->OrientacionAHRS.DCM_matriz[2][0]) * 180.0 / PI;
	AHRS->OrientacionAHRS.Roll  = atan2(AHRS->OrientacionAHRS.DCM_matriz[2][1], AHRS->OrientacionAHRS.DCM_matriz[2][2]) * 180.0 / PI;
	AHRS->OrientacionAHRS.Yaw = atan2(AHRS->OrientacionAHRS.DCM_matriz[1][0], AHRS->OrientacionAHRS.DCM_matriz[0][0]) * 180.0 / PI;
}

