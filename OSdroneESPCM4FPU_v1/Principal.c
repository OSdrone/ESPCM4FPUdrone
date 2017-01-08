/*
AHRS como tarea
Todos Sensores IMU en una tarea.
Telemetria sensores sin filtrar. paso de dato con buzon
Añadida correcccion de posicion mediante rotacion de la medidas, con calculo de matriz al inicio del ciclo




Sincronizacion de arranaque cambiada, las tareas inician su propio clock excepto el IMU
IMU->Se calibra, Guarda la primera lectura, Inicio AHRS , Tarea ciclico.
											*Reset de los parametros, Guarda la primera lectura, Inicio Coordinador , Tarea ciclico.
																								 Arranca WD, SYNCRO, Tarea ciclico.


*/

/* PARAMETROS CONFIGURACION */
//#define Filtro_Perturbaciones
#define Filtrado_Vel_IMU


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
//#include <xdc/runtime/IHeap.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/Knl/Task.h>
#include <ti/sysbios/Knl/Clock.h>
#include <ti/sysbios/Knl/Semaphore.h>
#include <ti/sysbios/Knl/Mailbox.h>
#include <ti/sysbios/Knl/Swi.h>
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Watchdog.h>

/* Periphals libraries */
#include <driverlib/eeprom.h>

/* Board Header file */

#include "Quad_Board.h"
#include "FuncionesRTOS/TIRTOS_FCN.h"
#include "arm_math.h"

#include "Parametros.h"

#include "Dispositivos/BAR_BMP280.h"
#include "Dispositivos/IMU_MPU9250.h"
#include "Dispositivos/Receiver_FlySky8.h"

#include "AHRS.h"
#include "Servidores.h"

//................Variables.....................................................//
//...Sistema....//
Ptr Datos;

tpModoCalibracionIMU ModoCalibracionIMU = CALIBRACION_COMPLETA_IMU; //CALIBRACION_COMPLETA_IMU;

tpEstado_Sistema Estado_Sistema  = ESPERA;
tpEstado_Sistema Estado_Sistema_Anterior = ESPERA;

tpModoTelemetria ModoTelemetria = TELEMETRIA_CONTROL;
tpInfoTelemetria InfoTelemetria = TELE_0;
const tpOrden START_FRAME = START;
const tpOrden FINAL_FRAME = FINAL;

tpModo_Control Modo_Control = ANGULOS_3;
tpModo_Control Modo_Control_Anterior = ANGULOS_3;
tpModoPerturbaciones ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; //NO_CORREGIR_PERTURBACIONES;

tpModoWatchdog ModoWatchdog = PARADA_EMER;

//....SENSORES....//
tpIMU9250 IMU9250 = {
	.Direccion_IMU = Dir_0_IMU_MPU9250,
	.Direccion_MAG = Dir_MAG_MPU9250,
	.SMPLRT_DIV = 0,
	.Ganancia_Acel = Gain_Acel_16G,
	.Sensibilidad_Acel = 32768.0/16.0,
	.Ganancia_Gyro = Gain_Gyro_500,
	.Sensibilidad_Giroscopo = 32768.0 / 500.0,
	.Sensibilidad_Brujula = 32768.0/ 4800.0,

	.DLPF_CFG_ACEL = DLPF_CFG_ACEL_NO,
	.DLPF_CFG_GYRO = DLPF_CFG_GYRO_NO
};

tpCalibracionIMU9DOF CalibracionIMU9DOF = {
		.Correccion_Alineamiento_IMU_matriz = {32767, 0, 0, 0, 32767, 0, 0, 0, 32767},
		.Correccion_Alineamiento_IMU = {3, 1, CalibracionIMU9DOF.Correccion_Alineamiento_IMU_matriz},
		.Correccion_COMPASS_matriz = {32767, 0, 0, 0, 32767, 0, 0, 0, 32767},
		.Correccion_COMPASS = {3, 1, CalibracionIMU9DOF.Correccion_COMPASS_matriz},
};

tpBarometro_BMP280 Barometro_BMP280 = {
	.Direccion = BMP_280_DIR_0,
	.Modo = Mode_Normal,
	.Oversampling_Presion = x16,
	.Oversampling_Temperatura = x16,
	.t_sampling = ms05
};

//...RECEPTOR...//

tpCalibracion_Receptor Calibracion_Receptor = { //POENR PROTEGIDO???
		{.Rango_Salida = {-32768, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-32768, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {0, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-32768, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-32768, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-32768, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-32768, 32767},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-100, 100},
		 .Rango_Entrada = {966, 1973}
		}
};

//..Variables del sistema...//
float32_t Posicion_inicial = 0;

uint16_t Altura_US_mm = 0;
uint16_t Altura_Presion_mm = 0;

float32_t Gravedad;

//......Identificacion..........//
uint32_t nDatos_Identifiacion = 0;
uint32_t PuntoTrabajo_motor = 0;
uint16_t nDatos_leidos = 0;
//uint16_t Ticks_por_RPS = 0;

//................Instancias....................................................//
//.......PWM............//
PWM_Handle PWM0;
PWM_Handle PWM1;
PWM_Handle PWM2;
PWM_Handle PWM3;

PWM_Params PARAMS_PWM0;
PWM_Params PARAMS_PWM1;
PWM_Params PARAMS_PWM2;
PWM_Params PARAMS_PWM3;

//.......I2C............//
I2C_Handle I2C_SENSORES;
I2C_Handle I2C_DISPOSITIVOS;

I2C_Params PARAMS_I2C;

//I2C_Params PARAMS_I2C_AUX;

//.......UART............//
UART_Handle UART_USB;
//UART_Handle UART_BT_MANDO;
UART_Handle UART_BT_TELEMETRIA;
UART_Handle UART_AUX;

UART_Params PARAMS_UART_USB;
UART_Params PARAMS_UART_BT_TELEMETRIA;
UART_Params PARAMS_UART_AUX;

//......SPI...nRF24L01......//
//tp_nRF24L01 nRF24L01;
//SPI_Params PARAMS_SPI_0;

//.....Timer..............//
Timer_Handle US_Timer;
Timer_Params PARAMS_US_Timer;

//....WATCHDOG...........//
Watchdog_Handle WatchDog_0;
Watchdog_Params PARAMS_WatchDog_0;

//...BUZON............//
Mailbox_Handle Buzon_Lecturas_IMU;
Mailbox_Params PARAMS_Buzon;


//.....................TASK.......SEMAPHORES.............CLOCK........................//

Task_Params Parametros_Tarea;
Semaphore_Params Parametros_Semaforo;
Clock_Params Parametos_Clock;

//....CONTROL....//
Task_Handle TASK_Control;
Semaphore_Handle SEMAPHORE_Control;
Clock_Handle CLOCK_Control;

//....ALTURA_US..//
Task_Handle TASK_Calculo_Altura;
Semaphore_Handle SEMAPHORE_Calculo_Altura;
Clock_Handle CLOCK_Calculo_Altura;

//....Leer_IMU...//
Task_Handle TASK_Leer_IMU;
Semaphore_Handle SEMAPHORE_Leer_IMU;
Clock_Handle CLOCK_Leer_IMU;

//...Calculo_AHRS....//
Task_Handle TASK_Calculo_AHRS;
Semaphore_Handle SEMAPHORE_Calculo_AHRS;
Clock_Handle CLOCK_Calculo_AHRS;

//....Identificacion......//
Task_Handle TASK_Identificacion;
Semaphore_Handle SEMAPHORE_Identificacion;
Clock_Handle CLOCK_Identificacion;

//....Coordinador......//
Task_Handle TASK_Coordinador;
Semaphore_Handle SEMAPHORE_Coordinador;
Clock_Handle CLOCK_Coordinador;

//............ERROR..........................//
Error_Block eb;

//...................FUNCIONES......................................................//
void Rotacion_X(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes);
void Rotacion_Y(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes);
void Rotacion_Z(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes);
void Rotacion_ZYZp(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes);
void Rotacion_ZXY(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes);
//void Rotacion_XYZ(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes);

/*
//.........WATCHDOG.................................//
void FuncionWatchDog();
void Reestablecer_Conexion();
//...Control.....//
void Control(UArg arg0, UArg arg1);
void CLK_Control();
//...US..........//
void ISR_Timer_US();
void ISR_GPIO_US();
void Calculo_Altura(UArg arg0, UArg arg1);
void CLK_Calculo_Altura();
//....Leer_IMU...//
void Lectura_Datos_IMU(UArg arg0, UArg arg1);
void CLK_Lectura_Datos_IMU();
//....Calculo_AHRS...//
void Calculo_AHRS(UArg arg0, UArg arg1);
void CLK_Calculo_AHRS();
//....Identificacion......//
void Identificacion(UArg arg0, UArg arg1);
void CLK_Identificacion();

#ifdef Sensor_RPM
	void ISR_GPIO_RPM();
#endif

//....Coordinador......//
void Coordinador(UArg arg0, UArg arg1);
void CLK_Coordinador();
*/

/*
 *  ======== main ========
 */
int main(void){
    Error_init(&eb);
//...........................INICIALIZACION...DRIVERS...................................................//
    /* Call board init functions */
	QUAD_BOARD_initGeneral();
    QUAD_BOARD_initGPIO();
    QUAD_BOARD_initPWM();
    QUAD_BOARD_initI2C();
    QUAD_BOARD_initSPI();
    QUAD_BOARD_initUART();
    QUAD_BOARD_initWatchdog();

//......SERVIDORES..........//
    Iniciar_Servidores();

    GPIO_write(QUAD_BOARD_LED_RED, 1);
    GPIO_write(QUAD_BOARD_LED_GREEN, 0);
    GPIO_write(QUAD_BOARD_LED_BLUE, 0);

//......PWM........//
    PWM_Params_init(&PARAMS_PWM0);
    PARAMS_PWM0.period = 5000;
    PARAMS_PWM0.dutyMode = PWM_DUTY_TIME;
    PWM_Params_init(&PARAMS_PWM1);
    PARAMS_PWM1.period = 5000;
    PARAMS_PWM1.dutyMode = PWM_DUTY_TIME;
    PWM_Params_init(&PARAMS_PWM2);
    PARAMS_PWM2.period = 5000;
    PARAMS_PWM2.dutyMode = PWM_DUTY_TIME;
    PWM_Params_init(&PARAMS_PWM3);
    PARAMS_PWM3.period = 5000;
    PARAMS_PWM3.dutyMode = PWM_DUTY_TIME;

    PWM0 = PWM_open(QUAD_BOARD_PWM0, &PARAMS_PWM0);
    PWM1 = PWM_open(QUAD_BOARD_PWM1, &PARAMS_PWM1);
    PWM2 = PWM_open(QUAD_BOARD_PWM2, &PARAMS_PWM2);
    PWM3 = PWM_open(QUAD_BOARD_PWM3, &PARAMS_PWM3);
/*
    PWM_setDuty(PWM0, Pulso_maximo_PWM_motor);
    PWM_setDuty(PWM1, Pulso_maximo_PWM_motor);
    PWM_setDuty(PWM2, Pulso_maximo_PWM_motor);
    PWM_setDuty(PWM3, Pulso_maximo_PWM_motor);
*/
    PWM_setDuty(PWM0, 0);
    PWM_setDuty(PWM1, 0);
    PWM_setDuty(PWM2, 0);
    PWM_setDuty(PWM3, 0);
//.......I2C............//
    I2C_Params_init(&PARAMS_I2C);
    PARAMS_I2C.bitRate = I2C_400kHz;
    PARAMS_I2C.transferMode = I2C_MODE_BLOCKING;
    PARAMS_I2C.transferCallbackFxn = NULL;

    I2C_SENSORES = I2C_open(QUAD_BOARD_I2C0, &PARAMS_I2C);

//    I2C_Params_init(&PARAMS_I2C_AUX);
//    PARAMS_I2C.bitRate = I2C_400kHz;
//    PARAMS_I2C.transferMode = I2C_MODE_BLOCKING;
//    PARAMS_I2C.transferCallbackFxn = NULL;

    I2C_DISPOSITIVOS = I2C_open(QUAD_BOARD_I2C2, &PARAMS_I2C);

//........UART..........//
    UART_Params_init(&PARAMS_UART_BT_TELEMETRIA);
    PARAMS_UART_BT_TELEMETRIA.baudRate = BAUD_RATE_460800;
    PARAMS_UART_BT_TELEMETRIA.dataLength = UART_LEN_8;
    PARAMS_UART_BT_TELEMETRIA.parityType = UART_PAR_NONE;
    PARAMS_UART_BT_TELEMETRIA.stopBits = UART_STOP_ONE;
    PARAMS_UART_BT_TELEMETRIA.readEcho = UART_ECHO_OFF;

    PARAMS_UART_BT_TELEMETRIA.readReturnMode = UART_RETURN_FULL;
    PARAMS_UART_BT_TELEMETRIA.writeDataMode = UART_DATA_BINARY;
    PARAMS_UART_BT_TELEMETRIA.readDataMode = UART_DATA_BINARY;
    PARAMS_UART_BT_TELEMETRIA.readMode = UART_MODE_BLOCKING;
    PARAMS_UART_BT_TELEMETRIA.writeMode = UART_MODE_BLOCKING;
    PARAMS_UART_BT_TELEMETRIA.writeTimeout = BIOS_WAIT_FOREVER;
    //PARAMS_UART_BT_TELEMETRIA.readTimeout = BIOS_NO_WAIT;
    PARAMS_UART_BT_TELEMETRIA.readTimeout = 180;

    UART_BT_TELEMETRIA = UART_open(QUAD_BOARD_UART5_BT_TELEMETRIA, &PARAMS_UART_BT_TELEMETRIA);

//.....WATCHDOG.....................//

/*
	Watchdog_Params_init(&PARAMS_WatchDog_0);
	PARAMS_WatchDog_0.callbackFxn = FuncionWatchDog;
	PARAMS_WatchDog_0.debugStallMode = Watchdog_DEBUG_STALL_ON;
	PARAMS_WatchDog_0.resetMode = Watchdog_RESET_OFF;
*/

//.....BUZON...............//
/*
	Mailbox_Params_init(&PARAMS_Buzon_Calibracion);
	Buzon_Calibracion_IMU = Mailbox_create(sizeof(tpLecturas_IMU), 1, &PARAMS_Buzon_Calibracion, &eb);
	Buzon_Calibracion_Brujula = Mailbox_create(sizeof(tpLecturas_Brujula), 1, &PARAMS_Buzon_Calibracion, &eb);
*/
//.........TIMER...............//
/*
	Timer_Params_init(&PARAMS_US_Timer);
	PARAMS_US_Timer.period = Pulso_arranque_us;  //Para una distancia MAX de 4 metros (aprox)
   	PARAMS_US_Timer.periodType = Timer_PeriodType_MICROSECS;
    PARAMS_US_Timer.runMode = Timer_RunMode_ONESHOT;
    PARAMS_US_Timer.startMode = Timer_StartMode_USER;

    US_Timer = Timer_create(5, ISR_Timer_US, &PARAMS_US_Timer, &eb );
*/
//................INTERRUPCIONES.....................................//

/*
    GPIO_setCallback(QUAD_BOARD_ECHO, ISR_GPIO_US);
    GPIO_enableInt(QUAD_BOARD_ECHO);
#ifdef Sensor_RPM
    GPIO_setCallback(QUAD_BOARD_RPM, ISR_GPIO_RPM);
    GPIO_enableInt(QUAD_BOARD_RPM);
#endif
*/

//...........................INICIALIZACION...TAREAS...................................................//
//....Control....//
	Task_Params_init(&Parametros_Tarea);
	Parametros_Tarea.priority = PRIORIDAD_Control;
	Parametros_Tarea.stackSize = 2304;
	TASK_Control = Task_create(Control, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
	SEMAPHORE_Control = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

	Clock_Params_init(&Parametos_Clock);
	Parametos_Clock.period = PERIODO_Control;
	Parametos_Clock.startFlag = false;
    CLOCK_Control = Clock_create(CLK_Control, Timeout_Clk_Control, &Parametos_Clock, &eb);

/*
//....Altura..//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Calculo_Altura;
    Parametros_Tarea.stackSize = 768;
	TASK_Calculo_Altura = Task_create(Calculo_Altura, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Calculo_Altura = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Calculo_Altura;
    Parametos_Clock.startFlag = false;
    CLOCK_Calculo_Altura = Clock_create(CLK_Calculo_Altura, Timeout_Clk_Calculo_Altura, &Parametos_Clock, &eb);
*/
//....Leer_IMU...//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Leer_IMU;
    Parametros_Tarea.stackSize = 1256;
	TASK_Leer_IMU = Task_create(Lectura_Datos_IMU, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Leer_IMU = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Leer_IMU;
    Parametos_Clock.startFlag = true;
    CLOCK_Leer_IMU = Clock_create(CLK_Lectura_Datos_IMU, Timeout_Clk_Leer_IMU, &Parametos_Clock, &eb);

//...Calculo AHRS....//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Calculo_AHRS;
	TASK_Calculo_AHRS = Task_create(Calculo_AHRS, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Calculo_AHRS = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Calculo_AHRS;
    Parametos_Clock.startFlag = false;
    CLOCK_Calculo_AHRS = Clock_create(CLK_Calculo_AHRS, Timeout_Clk_Calculo_AHRS, &Parametos_Clock, &eb);
/*
//....Identificacion......//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Identificacion;
    Parametros_Tarea.stackSize = 512;
    TASK_Identificacion = Task_create(Identificacion, &Parametros_Tarea, &eb);

    Semaphore_Params_init(&Parametros_Semaforo);
    Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Identificacion = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Identificacion;
    Parametos_Clock.startFlag = false;
    CLOCK_Identificacion = Clock_create(CLK_Identificacion, Timeout_Clk_Identificacion, &Parametos_Clock, &eb);
*/
//......Coordinador.......//

    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Coordinador;
    TASK_Coordinador = Task_create(Coordinador, &Parametros_Tarea, &eb);

    Semaphore_Params_init(&Parametros_Semaforo);
    Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Coordinador = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Coordinador;
    Parametos_Clock.startFlag = false;
    CLOCK_Coordinador = Clock_create(CLK_Coordinador, Timeout_Clk_Coordinador, &Parametos_Clock, &eb);


/*
	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor + Accion_Maxima);
	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor + Accion_Maxima);
	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor + Accion_Maxima);
	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor + Accion_Maxima);
    while(GPIO_read(QUAD_BOARD_SW2));

	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor + Accion_Minima);
	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor + Accion_Minima);
	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor + Accion_Minima);
	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor + Accion_Minima);
    while(!GPIO_read(QUAD_BOARD_SW2));
*/

/*
    	PWM_setDuty(PWM0, Accion_Maxima + Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM1, Accion_Maxima + Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM2, Accion_Maxima + Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM3, Accion_Maxima + Pulso_minimo_PWM_motor);
    while(GPIO_read(QUAD_BOARD_SW2));
    	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);
    while(!GPIO_read(QUAD_BOARD_SW2));
*/

    //Iniciamos la matriz de correccion
//    Rotacion_ZXY(&CalibracionIMU9DOF.Correccion_Alineamiento, CalibracionIMU9DOF.Giro, false);

    while(GPIO_read(QUAD_BOARD_SW2));

   /* Start BIOS */
    BIOS_start();

    return (0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void FuncionWatchDog(){
	float32_t Referencia[4] = {0, 0, 0, 0};
	UInt Key, Key2, Key3;

	Key = Task_disable();
	Key2 = Hwi_disable();
	Key3 = Swi_disable();

	Watchdog_clear(WatchDog_0);

	switch(Estado_Sistema){
		case VUELO:
		case ESPERA:
		default:
			Estado_Sistema_Anterior = Estado_Sistema;
			Modo_Control_Anterior = Modo_Control;

			Estado_Sistema = ERROR_CONEXION;

			GPIO_write(QUAD_BOARD_LED_RED, 0);
			GPIO_write(QUAD_BOARD_LED_GREEN, 0);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);
		break;
	}

	switch(ModoWatchdog){
		case PARADA_EMER:
			GPIO_write(QUAD_BOARD_LED_RED, 1);
			GPIO_write(QUAD_BOARD_LED_GREEN, 0);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);

			//Stop a todos los clocks y tareas
			Clock_stop(CLOCK_Control);
			Semaphore_reset(SEMAPHORE_Control, 0);
			Clock_stop(CLOCK_Identificacion);
			Semaphore_reset(SEMAPHORE_Identificacion, 0);

			PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);

			while(1);
			//System_exit(0);
		//break;
		case ESTABILIZACION_EMER:
			GPIO_toggle(QUAD_BOARD_LED_RED);
			GPIO_toggle(QUAD_BOARD_LED_GREEN);
			GPIO_toggle(QUAD_BOARD_LED_BLUE);

			Modo_Control = ANGULOS_4;
			memcpy(Direccion_servidor_Referencia(), Referencia, sizeof(Referencia));
		break;
	}

	Swi_restore(Key3);
	Hwi_restore(Key2);
	Task_restore(Key);
}

void Reestablecer_Conexion(){

	Modo_Control = Modo_Control_Anterior;
	Estado_Sistema = Estado_Sistema_Anterior;

	switch(Estado_Sistema){
		case VUELO:
			GPIO_write(QUAD_BOARD_LED_RED, 0);
			GPIO_write(QUAD_BOARD_LED_GREEN, 1);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);
			break;
		case ESPERA:
			GPIO_write(QUAD_BOARD_LED_RED, 1);
			GPIO_write(QUAD_BOARD_LED_GREEN, 1);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);
			break;
	}
}
void Rotacion_X(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes){
	float32_t Seno = 0;
	float32_t Cos = 0;

	float32_t Aux_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};

	float32_t Matriz_Rotacion_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Matriz_Rotacion = {3, 3, Matriz_Rotacion_Matriz};

	if(Radianes){ Giro = Giro * 180.0/PI; }
	arm_sin_cos_f32(Giro, &Seno, &Cos);
	Matriz_Rotacion.pData[0] = 1;
	Matriz_Rotacion.pData[4] = Cos;
	Matriz_Rotacion.pData[5] = -Seno;
	Matriz_Rotacion.pData[7] = Seno;
	Matriz_Rotacion.pData[8] = Cos;

	arm_copy_f32(Matriz->pData, Aux.pData, 3*Matriz->numCols);
	Aux.numCols = Matriz->numCols;
	Aux.numRows = Matriz->numRows;

	arm_mat_mult_f32(&Aux, &Matriz_Rotacion, Matriz);

}
void Rotacion_Y(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes){
	float32_t Seno = 0;
	float32_t Cos = 0;

	float32_t Aux_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};

	float32_t Matriz_Rotacion_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Matriz_Rotacion = {3, 3, Matriz_Rotacion_Matriz};

	if(Radianes){ Giro = Giro * 180.0/PI; }
	arm_sin_cos_f32(Giro, &Seno, &Cos);
	Matriz_Rotacion.pData[0] = Cos;
	Matriz_Rotacion.pData[2] = Seno;
	Matriz_Rotacion.pData[4] = 1;
	Matriz_Rotacion.pData[6] = -Seno;
	Matriz_Rotacion.pData[8] = Cos;

	arm_copy_f32(Matriz->pData, Aux.pData, 3*Matriz->numCols);
	Aux.numCols = Matriz->numCols;
	Aux.numRows = Matriz->numRows;

	arm_mat_mult_f32(&Aux, &Matriz_Rotacion, Matriz);
}
void Rotacion_Z(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes){
	float32_t Seno = 0;
	float32_t Cos = 0;

	float32_t Aux_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};

	float32_t Matriz_Rotacion_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Matriz_Rotacion = {3, 3, Matriz_Rotacion_Matriz};

	if(Radianes){ Giro = Giro * 180.0/PI; }
	arm_sin_cos_f32(Giro, &Seno, &Cos);
	Matriz_Rotacion.pData[0] = Cos;
	Matriz_Rotacion.pData[1] = -Seno;
	Matriz_Rotacion.pData[3] = Seno;
	Matriz_Rotacion.pData[4] = Cos;
	Matriz_Rotacion.pData[8] = 1;

	arm_copy_f32(Matriz->pData, Aux.pData, 3*Matriz->numCols);
	Aux.numCols = Matriz->numCols;
	Aux.numRows = Matriz->numRows;

	arm_mat_mult_f32(&Aux, &Matriz_Rotacion, Matriz);
}
/*
void Rotacion_ZYZp(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes){
	float32_t Aux_Matriz[9] = {1,0,0,0,1,0,0,0,1};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};
	float32_t Aux_Matriz2[9] = {1,0,0,0,1,0,0,0,1};
	arm_matrix_instance_f32 Aux2 = {Matriz->numRows, Matriz->numCols, Aux_Matriz2};
	float32_t Aux_Matriz3[9] = {1,0,0,0,1,0,0,0,1};
	arm_matrix_instance_f32 Aux3 = {Matriz->numRows, Matriz->numCols, Aux_Matriz3};

	Rotacion_Z(&Aux, Giro[0], Radianes);
	Rotacion_Y(&Aux2, Giro[1], Radianes);
	arm_mat_mult_f32(&Aux, &Aux2, &Aux3);
	Rotacion_Z(&Aux, Giro[2], Radianes);
	arm_mat_mult_f32(&Aux3, &Aux, Matriz);
}
*/
void Rotacion_ZXY(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes){

	Matriz->pData[0] = 1;
	Matriz->pData[1] = 0;
	Matriz->pData[2] = 0;
	Matriz->pData[3] = 0;
	Matriz->pData[4] = 1;
	Matriz->pData[5] = 0;
	Matriz->pData[6] = 0;
	Matriz->pData[7] = 0;
	Matriz->pData[8] = 1;

	Rotacion_Z(Matriz, Giro[0], Radianes);
	Rotacion_X(Matriz, Giro[1], Radianes);
	Rotacion_Y(Matriz, Giro[2], Radianes);
}

void Rotacion_ZYZp(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes){

	Matriz->pData[0] = 1;
	Matriz->pData[1] = 0;
	Matriz->pData[2] = 0;
	Matriz->pData[3] = 0;
	Matriz->pData[4] = 1;
	Matriz->pData[5] = 0;
	Matriz->pData[6] = 0;
	Matriz->pData[7] = 0;
	Matriz->pData[8] = 1;

	Rotacion_Z(Matriz, Giro[0], Radianes);
	Rotacion_Y(Matriz, Giro[1], Radianes);
	Rotacion_Z(Matriz, Giro[2], Radianes);
}



