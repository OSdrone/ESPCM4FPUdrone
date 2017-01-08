/*
 * Coordinador_TASK.c
 *
 *  Created on: 7/1/2017
 *      Author: Ruben
 */

#include "TIRTOS_FCN.h"

extern I2C_Handle I2C_DISPOSITIVOS;
extern tpCalibracion_Receptor Calibracion_Receptor;
extern tpCalibracionIMU9DOF CalibracionIMU9DOF;

void Coordinador(){
//..Ref.............................
	tpLectura_Radio Lectura_Radio;
	I2C_Transaction I2C_Transmision;

	I2C_Transmision.slaveAddress = Dir_AUX;
	I2C_Transmision.writeBuf = NULL;
	I2C_Transmision.writeCount = 0;
	I2C_Transmision.readBuf = &Lectura_Radio;
	I2C_Transmision.readCount = sizeof(tpLectura_Radio);

	bool estado_Transmision = false;

	static uint8_t Temporizador_Ticks = ticks_arranque_vuelo; //3seg

	q15_t Canal[8];
	float32_t Referencia[4] = {0, 0, 0, 0};

	Semaphore_pend(SEMAPHORE_Coordinador, BIOS_WAIT_FOREVER);

//	WatchDog_0 = Watchdog_open(QUAD_BOARD_WATCHDOG0 , &PARAMS_WatchDog_0);
//	Watchdog_clear(WatchDog_0);

	Clock_start(CLOCK_Coordinador);

	estado_Transmision = I2C_transfer(I2C_DISPOSITIVOS, &I2C_Transmision);


	while(!(estado_Transmision && Lectura_Radio.Canal_PWM[2] < 1100)){
		Semaphore_pend(SEMAPHORE_Coordinador, BIOS_WAIT_FOREVER);
		estado_Transmision = I2C_transfer(I2C_DISPOSITIVOS, &I2C_Transmision);
  		if(Lectura_Radio.Error_conexion != 0){
//			Watchdog_clear(WatchDog_0);
		}
	}

	while(1){
		Semaphore_pend(SEMAPHORE_Coordinador, BIOS_WAIT_FOREVER);

		estado_Transmision = I2C_transfer(I2C_DISPOSITIVOS, &I2C_Transmision);

		if(estado_Transmision && Lectura_Radio.Error_conexion != 0){
//			Watchdog_clear(WatchDog_0);

//Ajustamos Rangos entrada
			AjustarRangoEntrada_Q15(Lectura_Radio, Canal);

			//......PULSADORES...................................//
			if(!GPIO_read(QUAD_BOARD_SW2)){
				while(!GPIO_read(QUAD_BOARD_SW2));

				switch(Estado_Sistema){
					case ESPERA:

					break;
					case DEBUG:
						Estado_Sistema = CALIBRACION;

					    GPIO_write(QUAD_BOARD_LED_RED, 0);
						GPIO_write(QUAD_BOARD_LED_GREEN, 1);
						GPIO_write(QUAD_BOARD_LED_BLUE, 1);

						Calibracion_Receptor[0].Rango_Entrada[0] = 1500; Calibracion_Receptor[0].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[1].Rango_Entrada[0] = 1500; Calibracion_Receptor[1].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[2].Rango_Entrada[0] = 1500; Calibracion_Receptor[2].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[3].Rango_Entrada[0] = 1500; Calibracion_Receptor[3].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[4].Rango_Entrada[0] = 1500; Calibracion_Receptor[4].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[5].Rango_Entrada[0] = 1500; Calibracion_Receptor[5].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[6].Rango_Entrada[0] = 1500; Calibracion_Receptor[6].Rango_Entrada[1] = 1500;
//						Calibracion_Receptor[7].Rango_Entrada[0] = 1500; Calibracion_Receptor[7].Rango_Entrada[1] = 1500;

					break;
					case CALIBRACION:
						Estado_Sistema = ESPERA;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 1);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

					break;
				}

			//...........STICK_IZQUIERDA NEGATIVO.............................................................//
			}else if((Canal[0] <= (int16_t)(-0.9 * 32767)) && (Canal[2] < (int16_t)(0.05 * 32767))){
				if (Temporizador_Ticks-- == 0){

					switch(Estado_Sistema){
						case DEBUG:
							//resetear variables
						break;
						case ESPERA:
							Estado_Sistema = DEBUG;

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 1);
						break;
						case VUELO:
							Estado_Sistema = ESPERA;

							Clock_stop(CLOCK_Control);
							Semaphore_reset(SEMAPHORE_Control, 0);

							PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
							PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
							PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
							PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);
						break;
					}
				}
			//...........STICK_IZQUIERDA POSITIVO.............................................................//
			}else if((Canal[0] >= (int16_t)(0.9 * 32767)) && (Canal[2]) < (int16_t)(0.05 * 32767)) {
				if (--Temporizador_Ticks == 0){

					switch(Estado_Sistema){
						case DEBUG:
							Estado_Sistema = ESPERA;

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);

							Leer_servidor_RPY(NULL, NULL, &Posicion_inicial);
							Posicion_inicial = CONVERTIR_A_GRADOS(Posicion_inicial);

						break;
						case ESPERA:
							Estado_Sistema = VUELO;

							GPIO_write(QUAD_BOARD_LED_RED, 0);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);

							Clock_start(CLOCK_Control);
							Resetear_servidor_Perturbaciones_Estimadas();

						break;
					}
				}
			//...........STICK DERECHA NEGATIVO.............................................................//
			}else if((Canal[3] <= (int16_t)(-0.9 * 32767)) && (Canal[2]) < (int16_t)(0.05 * 32767)) {
				if (--Temporizador_Ticks == 0){

					switch(Estado_Sistema){
						case DEBUG:
						break;
						case ESPERA:
							//RESET
/*
							Leer_servidor_RPY(&Angulos[1], &Angulos[0], &Angulos[2]);

							//Iniciamos la matriz de correccion
							Calibracion_IMU.Giro[1] -= CONVERTIR_A_GRADOS(Angulos[0]);
							Calibracion_IMU.Giro[2] -= CONVERTIR_A_GRADOS(Angulos[1]);

						    Rotacion_ZXY(&Calibracion_IMU.Correccion_Alineamiento, Calibracion_IMU.Giro, false);

							GPIO_write(QUAD_BOARD_LED_RED, 0);
							GPIO_write(QUAD_BOARD_LED_GREEN, 0);
							GPIO_write(QUAD_BOARD_LED_BLUE, 1);

							Task_sleep(250);

							ResetDCM();

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);
*/
						break;
					}
					Temporizador_Ticks = ticks_arranque_vuelo;
				}
			//...........STICK DERECHA POSITIVO.............................................................//
			}
			/*
			else if((Canal[3] >= 0.9) && (Canal[2]) < 0.05) {
				if (--Temporizador_Ticks == 0){

					switch(Estado_Sistema){
						case DEBUG:
							Estado_Sistema = IDENTIFICACION;

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 0);
							GPIO_write(QUAD_BOARD_LED_BLUE, 1);

							i = 0;

							do{
								Orden = START;
								UART_write(UART_BT_TELEMETRIA, &Orden, 1);
								Orden = IDENTIFICAR;
								UART_write(UART_BT_TELEMETRIA, &Orden, 1);
								Orden = FINAL;
								UART_write(UART_BT_TELEMETRIA, &Orden, 1);

								UART_read(UART_BT_TELEMETRIA, &Orden, 1);

								Watchdog_clear(WatchDog_0);
							}while(Orden != IDENTIFICAR && ++i < Num_intentos_conexion_Identificacion);

							if(Orden == IDENTIFICAR){
								UART_read(UART_BT_TELEMETRIA, &nDatos_Identifiacion, 4);
								UART_read(UART_BT_TELEMETRIA, &PuntoTrabajo_motor ,4);

								Datos = Memory_alloc(NULL, nDatos_Identifiacion*2, 0, &eb);

								for(i=0; i<nDatos_Identifiacion; i++){
									UART_read(UART_BT_TELEMETRIA, (int16_t *)Datos + i, 2);
									Watchdog_clear(WatchDog_0);
								}

								UART_read(UART_BT_TELEMETRIA, &Orden, 1);

								if(Orden == FINAL){

									nDatos_leidos = 0;
									Clock_start(CLOCK_Identificacion);
								}else{
									Estado_Sistema = ESPERA;

									GPIO_write(QUAD_BOARD_LED_RED, 1);
									GPIO_write(QUAD_BOARD_LED_GREEN, 1);
									GPIO_write(QUAD_BOARD_LED_BLUE, 0);
								}
							}else{

								Estado_Sistema = ESPERA;

								GPIO_write(QUAD_BOARD_LED_RED, 1);
								GPIO_write(QUAD_BOARD_LED_GREEN, 1);
								GPIO_write(QUAD_BOARD_LED_BLUE, 0);
							}
					}
				}
			}
			*/
			else{
				Temporizador_Ticks = ticks_arranque_vuelo;
			}

			//.............Accion...............................//
			switch(Estado_Sistema){
				case VUELO:
					if(Canal[7] < -83){	//...ERROR....//
		//				Estado_Sistema_Anterior = Estado_Sistema;
						Estado_Sistema = ERROR;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 0);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

						//Stop a todos los clocks y tareas
						Clock_stop(CLOCK_Control);
						Semaphore_reset(SEMAPHORE_Control, 0);
//						Clock_stop(CLOCK_Identificacion);
//						Semaphore_reset(SEMAPHORE_Identificacion, 0);

						//Parada_motores
						PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);
					}

					//Modos de Vuelo
					else if((Canal[7] > -83) && (Canal[7] <= -50)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_1; }
					else if((Canal[7] > -50) && (Canal[7] <= -16)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = INTEGRAR_PERTURBACIONES_ESTIMADAS; InfoTelemetria = TELE_2;}
					else if((Canal[7] > -16) && (Canal[7] <=  16)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = INTEGRAR_PERTURBACIONES; InfoTelemetria = TELE_3;}
					else if((Canal[7] >  16) && (Canal[7] <=  50)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}
					else if((Canal[7] >  50) && (Canal[7] <=  83)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}
					else if((Canal[7] >  83))						  { Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}


					switch(Modo_Control){
						case ANGULOS_3:
							Referencia[0] = (float32_t)Canal[1]/32768 * Angulo_Maximo;
							Referencia[1] = (float32_t)Canal[3]/32768 * Angulo_Maximo;
							Referencia[2] = (float32_t)Canal[0]/32768 * Angulo_Maximo;
							Referencia[3] = (float32_t)Canal[2]/32768 * Valor_Empuje_Maximo;
						break;
						case ANGULOS_4:
							Referencia[0] = (float32_t)Canal[1]/32768 * Angulo_Maximo;
							Referencia[1] = (float32_t)Canal[3]/32768 * Angulo_Maximo;
							Referencia[2] = (float32_t)Canal[0]/32768 * Angulo_Maximo;
							Referencia[3] = (float32_t)Canal[2]/32768 * Valor_Fuerza_Maximo;
						break;
						case EMPUJE:
							Referencia[0] = (float32_t)Canal[1]/32768 * Angulo_Maximo;
							Referencia[1] = (float32_t)Canal[3]/32768 * Angulo_Maximo;
							Referencia[2] = (float32_t)Canal[0]/32768 * Angulo_Maximo;
							Referencia[3] = (float32_t)Canal[2]/32768 * Valor_Empuje_Maximo;
						break;
					}
/*
					if (Canal[4] > 0){
						K_pre_3_matriz[0] = 30.0/2 * Canal[4];
						K_pre_3_matriz[8] = -30.0/2 * Canal[4];

						K_3_matriz[0] = 30.0/2 * Canal[5];
						K_3_matriz[1] = 30.0/2 * Canal[4];
						K_3_matriz[20] = -30.0/2 * Canal[5];
						K_3_matriz[21] = -30.0/2 * Canal[4];
					}else{
						K_pre_3_matriz[0] = 30.0/2 * Kp1;
						K_pre_3_matriz[8] = -30.0/2 * Kp1;

						K_3_matriz[0] = 30.0/2 * Kv1;
						K_3_matriz[1] = 30.0/2 * Kp1;
						K_3_matriz[20] = -30.0/2 * Kv1;
						K_3_matriz[21] = -30.0/2 * Kp1;
					}
*/
					Escribir_servidor_Referencia(Referencia, NULL);
				break;
				case CALIBRACION:

					Calibracion_Receptor[0].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[0] < Calibracion_Receptor[0].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[0] :  Calibracion_Receptor[0].Rango_Entrada[0];
					Calibracion_Receptor[0].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[0] > Calibracion_Receptor[0].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[0] :  Calibracion_Receptor[0].Rango_Entrada[1];
					Calibracion_Receptor[1].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[1] < Calibracion_Receptor[1].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[1] :  Calibracion_Receptor[1].Rango_Entrada[0];
					Calibracion_Receptor[1].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[1] > Calibracion_Receptor[1].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[1] :  Calibracion_Receptor[1].Rango_Entrada[1];
					Calibracion_Receptor[2].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[2] < Calibracion_Receptor[2].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[2] :  Calibracion_Receptor[2].Rango_Entrada[0];
					Calibracion_Receptor[2].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[2] > Calibracion_Receptor[2].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[2] :  Calibracion_Receptor[2].Rango_Entrada[1];
					Calibracion_Receptor[3].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[3] < Calibracion_Receptor[3].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[3] :  Calibracion_Receptor[3].Rango_Entrada[0];
					Calibracion_Receptor[3].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[3] > Calibracion_Receptor[3].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[3] :  Calibracion_Receptor[3].Rango_Entrada[1];
					Calibracion_Receptor[4].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[4] < Calibracion_Receptor[4].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[4] :  Calibracion_Receptor[4].Rango_Entrada[0];
					Calibracion_Receptor[4].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[4] > Calibracion_Receptor[4].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[4] :  Calibracion_Receptor[4].Rango_Entrada[1];
					Calibracion_Receptor[5].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[5] < Calibracion_Receptor[5].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[5] :  Calibracion_Receptor[5].Rango_Entrada[0];
					Calibracion_Receptor[5].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[5] > Calibracion_Receptor[5].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[5] :  Calibracion_Receptor[5].Rango_Entrada[1];
					Calibracion_Receptor[6].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[6] < Calibracion_Receptor[6].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[6] :  Calibracion_Receptor[6].Rango_Entrada[0];
					Calibracion_Receptor[6].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[6] > Calibracion_Receptor[6].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[6] :  Calibracion_Receptor[6].Rango_Entrada[1];
//					Calibracion_Receptor[7].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[7] < Calibracion_Receptor[7].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[7] :  Calibracion_Receptor[7].Rango_Entrada[0];
//					Calibracion_Receptor[7].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[7] > Calibracion_Receptor[7].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[7] :  Calibracion_Receptor[7].Rango_Entrada[1];
				break;
				case ERROR:
					if (Canal[7] > -83){ //..FIN_ERROR..//
						Estado_Sistema = ESPERA;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 1);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

					}
					break;
				case ESPERA:
				case DEBUG:
					if(Canal[7] < -83){	//...ERROR....//
						Estado_Sistema = ERROR;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 0);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

						//Stop a todos los clocks y tareas
						Clock_stop(CLOCK_Control);
						Semaphore_reset(SEMAPHORE_Control, 0);
//						Clock_stop(CLOCK_Identificacion);
//						Semaphore_reset(SEMAPHORE_Identificacion, 0);

						//Parada_motores
						PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);
					}
				break;
			}
		}
	}
}

void CLK_Coordinador(){
	Semaphore_post(SEMAPHORE_Coordinador);
}

void AjustarRangoEntrada_Q15(tpLectura_Radio Lectura_Radio, q15_t Canal[]){
	Canal[0] = ( (int32_t)( Lectura_Radio.Canal_PWM[0] - Calibracion_Receptor[0].Rango_Entrada[0] ) * ( Calibracion_Receptor[0].Rango_Salida[1] - Calibracion_Receptor[0].Rango_Salida[0] ) / ( Calibracion_Receptor[0].Rango_Entrada[1] - Calibracion_Receptor[0].Rango_Entrada[0] ) + Calibracion_Receptor[0].Rango_Salida[0] );
	Canal[1] = ( (int32_t)( Lectura_Radio.Canal_PWM[1] - Calibracion_Receptor[1].Rango_Entrada[0] ) * ( Calibracion_Receptor[1].Rango_Salida[1] - Calibracion_Receptor[1].Rango_Salida[0] ) / ( Calibracion_Receptor[1].Rango_Entrada[1] - Calibracion_Receptor[1].Rango_Entrada[0] ) + Calibracion_Receptor[1].Rango_Salida[0] );
	Canal[2] = ( (int32_t)( Lectura_Radio.Canal_PWM[2] - Calibracion_Receptor[2].Rango_Entrada[0] ) * ( Calibracion_Receptor[2].Rango_Salida[1] - Calibracion_Receptor[2].Rango_Salida[0] ) / ( Calibracion_Receptor[2].Rango_Entrada[1] - Calibracion_Receptor[2].Rango_Entrada[0] ) + Calibracion_Receptor[2].Rango_Salida[0] );
	Canal[3] = ( (int32_t)( Lectura_Radio.Canal_PWM[3] - Calibracion_Receptor[3].Rango_Entrada[0] ) * ( Calibracion_Receptor[3].Rango_Salida[1] - Calibracion_Receptor[3].Rango_Salida[0] ) / ( Calibracion_Receptor[3].Rango_Entrada[1] - Calibracion_Receptor[3].Rango_Entrada[0] ) + Calibracion_Receptor[3].Rango_Salida[0] );
	Canal[4] = ( (int32_t)( Lectura_Radio.Canal_PWM[4] - Calibracion_Receptor[4].Rango_Entrada[0] ) * ( Calibracion_Receptor[4].Rango_Salida[1] - Calibracion_Receptor[4].Rango_Salida[0] ) / ( Calibracion_Receptor[4].Rango_Entrada[1] - Calibracion_Receptor[4].Rango_Entrada[0] ) + Calibracion_Receptor[4].Rango_Salida[0] );
	Canal[5] = ( (int32_t)( Lectura_Radio.Canal_PWM[5] - Calibracion_Receptor[5].Rango_Entrada[0] ) * ( Calibracion_Receptor[5].Rango_Salida[1] - Calibracion_Receptor[5].Rango_Salida[0] ) / ( Calibracion_Receptor[5].Rango_Entrada[1] - Calibracion_Receptor[5].Rango_Entrada[0] ) + Calibracion_Receptor[5].Rango_Salida[0] );
	Canal[6] = ( (int32_t)( Lectura_Radio.Canal_PWM[6] - Calibracion_Receptor[6].Rango_Entrada[0] ) * ( Calibracion_Receptor[6].Rango_Salida[1] - Calibracion_Receptor[6].Rango_Salida[0] ) / ( Calibracion_Receptor[6].Rango_Entrada[1] - Calibracion_Receptor[6].Rango_Entrada[0] ) + Calibracion_Receptor[6].Rango_Salida[0] );
	Canal[7] = ( (int32_t)( Lectura_Radio.Canal_PWM[7] - Calibracion_Receptor[7].Rango_Entrada[0] ) * ( Calibracion_Receptor[7].Rango_Salida[1] - Calibracion_Receptor[7].Rango_Salida[0] ) / ( Calibracion_Receptor[7].Rango_Entrada[1] - Calibracion_Receptor[7].Rango_Entrada[0] ) + Calibracion_Receptor[7].Rango_Salida[0] );

}
