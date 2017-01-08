#ifndef PARAMETROS_DEF
#define PARAMETROS_DEF

#include "arm_math.h"

#define Frecuencia_CPU (uint32_t)80000000UL

#define BAUD_RATE_9600 (uint32_t)9600UL
#define BAUD_RATE_115200 (uint32_t)115200UL
#define BAUD_RATE_128000 (uint32_t)128000UL
#define BAUD_RATE_460800 (uint32_t)460800UL
#define BITRATE_SPI (uint32_t)1000000UL

//Integrado AUX
#define Dir_AUX 0x01

//Parametros Tareas
#define PRIORIDAD_Leer_IMU 10
#define PERIODO_Leer_IMU 1
#define Timeout_Clk_Leer_IMU 1

#define PRIORIDAD_Calculo_AHRS 9
#define PERIODO_Calculo_AHRS 2
#define Timeout_Clk_Calculo_AHRS 250

#define PRIORIDAD_Control 8
#define PERIODO_Control 5
#define Timeout_Clk_Control 5

#define PRIORIDAD_Identificacion 8
#define PERIODO_Identificacion 5
#define Timeout_Clk_Identificacion 5

#define PRIORIDAD_Coordinador 7
#define PERIODO_Coordinador 25
#define Timeout_Clk_Coordinador 25

#define PRIORIDAD_Calculo_Altura 6
#define PERIODO_Calculo_Altura 60
#define Timeout_Clk_Calculo_Altura 60

//Parametros Funcionamiento
#define numCanales 8
#define ticks_arranque_vuelo 3*1000/PERIODO_Coordinador
#define Numero_Muetras_calibracion_IMU 1000
#define Numero_Muetras_calibracion_Brujula 100
#define Longitud_buffer 9000
#define Num_intentos_conexion_Identificacion 5

typedef enum {VUELO, ESPERA, ERROR, IDENTIFICACION, CALIBRACION, DEBUG, ERROR_CONEXION} tpEstado_Sistema;
//typedef enum {IDENTIFICACION_EJE, IDENTIFICACION_MOTOR, IDENTIFIACION_NULA}tpEstado_Identificacion;
typedef enum {NO_TELEMETRIA, TELEMETRIA_CONTROL, TELEMETRIA_YPR, TELEMETRIA_IMU, TELEMETRIA_BRUJULA}tpModoTelemetria;
typedef enum {CORREGIR_PERTURBACIONES, NO_CORREGIR_PERTURBACIONES, INTEGRAR_PERTURBACIONES, INTEGRAR_PERTURBACIONES_ESTIMADAS}tpModoPerturbaciones;
typedef enum {ANGULOS_4, ANGULOS_3, ANGULOS_1, EMPUJE}tpModo_Control;
typedef enum {PARADA_EMER, ESTABILIZACION_EMER, ATERIZAJE_EMER, NO_WATCHDOG}tpModoWatchdog;
typedef enum {CALIBRACION_COMPLETA_IMU, CALIBRACION_GIROSCOPO, CALIBRACION_ACELEROMETRO, NO_CALIBRAR_IMU}tpModoCalibracionIMU;
typedef enum {START = '#', FINAL = '*', IDENTIFICAR ='I', CALIBRAR = 'C', TELEMETRIA = 'T', DATO_ANTERIOR = 0xFF } tpOrden;
typedef enum {TELE_0, TELE_1, TELE_2, TELE_3, TELE_4} tpInfoTelemetria;
// TELE_0 Telemetria Control, Sin perturcaciones, control 3 Angulos
// TELE_1 Telemetria Control, perturcaciones, control 3 Angulos
// TELE_2 Telemetria Control, perturcaciones integradas estimadas, control 3 Angulos
// TELE_3 Telemetria Control, perturcaciones integradas, control 3 Angulos
// TELE_4 Telemetria Control, EMPUJE


typedef struct{
	uint8_t Inicio;
	uint8_t InfoTelemetria;
	int16_t Referencia[4];
	uint16_t Accion[4];
	int16_t Variables_Estado[10];
	int16_t Perturbaciones[4];
	uint16_t Altura_Barometrica;
	uint16_t Altura_US;
	int16_t Acel[3];
	int16_t Gyro[3];
	int16_t Magnetics[3];
	uint8_t Final;
}tpTelemetria_Control;

typedef struct{
	uint8_t Inicio;
	int16_t Yaw;
	int16_t Pitch;
	int16_t Roll;
	uint8_t Final;
}tpTelemetria_YPR;

typedef struct{
	tpOrden Inicio;
	uint16_t Acel[3];
	uint16_t Gyro[3];
	uint16_t Mag[3];
	tpOrden Final;
}tpTelemetria_IMU;

typedef struct{
	float32_t Yaw_offset;
	float32_t Pitch_offset;
	float32_t Roll_offset;
}tpPuntoInicial;

//VALORES MAXIMOS
#define Angulo_Maximo 30.0//º
#define Velocidad_Angular_Maxima 10.0 //º/s
#define Valor_Empuje_Maximo 800
#define Valor_Fuerza_Maximo 2.0
#define Valor_perturbacion_Motor_MAX 5.0
#define Valor_perturbacion_Motor_MIN 0.0

//VALORES CONVERSION Q_16
#define Angulo_Max_Q16 180.0
#define Velocidad_Max_Q16 1000.0
#define F_Max_Q16 100.0

//TIMEOUTS
#define Timeout_UART_BLUETOOTH_Lectura 10

//TAREAS
#define nTokensIniciales_0 0
#define nTokensIniciales_1 1

//Parametros Ultrasonido US
#define Pulso_arranque_us 10 //us
#define Distancia_MAX_us 4 //m
#define Velocidad_Sonido 340 // m/s
#define Max_pulso_us 25000 // (4.25 metros)
#define Min_pulso_us 120 // 2 cm

typedef enum{INT_TRIGGER, INT_LLEGADA}tpIntUS;

//Parametros Motor
#define Offset_rpm 1174
#define Accion_Maxima 1000
#define Accion_Minima 100

#define Pulso_minimo_PWM_motor 1000U
#define Pulso_maximo_PWM_motor 2000U

//Parametros Chasis
#define MasaQuadrotor 0.9 //Kg
#define Pert_Fuerza_Bateria 0.3 //N

#endif
