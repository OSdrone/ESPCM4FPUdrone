/*
 * TIRTOS_FCN.h
 *
 *  Created on: 8/12/2016
 *      Author: Ruben
 */

#ifndef FUNCIONESRTOS_TIRTOS_FCN_H_
#define FUNCIONESRTOS_TIRTOS_FCN_H_

#include "arm_math.h"
#include "../Parametros.h"
#include "../Servidores.h"
#include "../Quad_board.h"
//#include "../Funciones_Transferencia.h"
#include "../Dispositivos/Receiver_FlySky8.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/Knl/Task.h>
#include <ti/sysbios/Knl/Clock.h>
#include <ti/sysbios/Knl/Semaphore.h>
#include <ti/sysbios/Knl/Mailbox.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/GPIO.h>

//Variables Globales
extern tpEstado_Sistema Estado_Sistema;
extern tpModoTelemetria ModoTelemetria;
extern tpInfoTelemetria InfoTelemetria;

extern float32_t Posicion_inicial;

//...Modos..Vuelo.......//
extern tpModo_Control Modo_Control;
extern tpModoPerturbaciones ModoPerturbaciones;

//.......PWM............//
extern PWM_Handle PWM0;
extern PWM_Handle PWM1;
extern PWM_Handle PWM2;
extern PWM_Handle PWM3;

//....Coordinador......//
extern Task_Handle TASK_Coordinador;
extern Semaphore_Handle SEMAPHORE_Coordinador;
extern Clock_Handle CLOCK_Coordinador;

void Coordinador();
void CLK_Coordinador();

void AjustarRangoEntrada_Q15(tpLectura_Radio Lectura_Radio, q15_t Canal[]);

//....Calculo_AHRS...//
extern Task_Handle TASK_Calculo_AHRS;
extern Semaphore_Handle SEMAPHORE_Calculo_AHRS;
extern Clock_Handle CLOCK_Calculo_AHRS;

void Calculo_AHRS();
void CLK_Calculo_AHRS();

//....Leer_IMU...//
extern Task_Handle TASK_Leer_IMU;
extern Semaphore_Handle SEMAPHORE_Leer_IMU;
extern Clock_Handle CLOCK_Leer_IMU;

void Lectura_Datos_IMU();
void CLK_Lectura_Datos_IMU();

void ResetAHRS();
void setAHRS(tpAHRS *AHRSset);

//...Control.....//
extern Task_Handle TASK_Control;
extern Semaphore_Handle SEMAPHORE_Control;
extern Clock_Handle CLOCK_Control;

void Control();
void CLK_Control();

#endif /* FUNCIONESRTOS_TIRTOS_FCN_H_ */
