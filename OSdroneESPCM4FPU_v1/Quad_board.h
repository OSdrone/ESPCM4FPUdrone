/** ============================================================================
 *  @file       QUAD_BOARD.h
 *
 *  @brief      QUAD_BOARD Board Specific APIs
 *
 *  The QUAD_BOARD header file should be included in an application as
 *  follows:
 *  @code
 *  #include <QUAD_BOARD.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef QUAD_BOARD_H
#define QUAD_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on QUAD_BOARD are active high. */
#define QUAD_BOARD_LED_OFF (0)
#define QUAD_BOARD_LED_ON  (1)

/*!
 *  @def    QUAD_BOARD_GPIOName
 *  @brief  Enum of GPIO names on the QUAD_BOARD dev board
 */
typedef enum QUAD_BOARD_GPIOName {
	QUAD_BOARD_SW1 = 0,
	QUAD_BOARD_SW2,
	QUAD_BOARD_ECHO,
	QUAD_BOARD_RPM,

	QUAD_BOARD_TRIGG,
	QUAD_BOARD_LED_RED,
	QUAD_BOARD_LED_BLUE,
	QUAD_BOARD_LED_GREEN,
	QUAD_BOARD_POWER_33,
	QUAD_BOARD_SPI_CE,
	QUAD_BOARD_SPI_CSN,

	QUAD_BOARD_GPIOCOUNT
} QUAD_BOARD_GPIOName;

/*!
 *  @def    QUAD_BOARD_I2CName
 *  @brief  Enum of I2C names on the QUAD_BOARD dev board
 */
typedef enum QUAD_BOARD_I2CName {
    QUAD_BOARD_I2C0 = 0,
	QUAD_BOARD_I2C2,
    QUAD_BOARD_I2CCOUNT
} QUAD_BOARD_I2CName;

/*!
 *  @def    QUAD_BOARD_PWMName
 *  @brief  Enum of PWM names on the QUAD_BOARD dev board
 */
typedef enum QUAD_BOARD_PWMName {
    QUAD_BOARD_PWM0 = 0,
    QUAD_BOARD_PWM1,
    QUAD_BOARD_PWM2,
    QUAD_BOARD_PWM3,

    QUAD_BOARD_PWMCOUNT
} QUAD_BOARD_PWMName;

/*!
 *  @def    QUAD_BOARD_UARTName
 *  @brief  Enum of UARTs on the QUAD_BOARD dev board
 */
typedef enum QUAD_BOARD_UARTName {
    QUAD_BOARD_UART0_USB = 0,
	QUAD_BOARD_UART5_BT_TELEMETRIA,
	QUAD_BOARD_UART1_BT_MANDO,
	QUAD_BOARD_UART7_AUX,

    QUAD_BOARD_UARTCOUNT
} QUAD_BOARD_UARTName;

/*!
 *  @def    QUAD_BOARD_WatchdogName
 *  @brief  Enum of Watchdogs on the QUAD_BOARD dev board
 */
typedef enum QUAD_BOARD_WatchdogName {
    QUAD_BOARD_WATCHDOG0 = 0,

    QUAD_BOARD_WATCHDOGCOUNT
} QUAD_BOARD_WatchdogName;

/*!
 *  @brief  Initialize board specific DMA settings
 *
 *  This function creates a hwi in case the DMA controller creates an error
 *  interrrupt, enables the DMA and supplies it with a uDMA control table.
 */

typedef enum QUAD_BOARD_SPIName {
	QUAD_BOARD_SPI0 = 0,

	QUAD_BOARD_SPICOUNT
} QUAD_BOARD_SPIName;


extern void QUAD_BOARD_initDMA(void);

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 *  This includes:
 *     - Flash wait states based on the process
 *     - Disable clock source to watchdog module
 *     - Enable clock sources for peripherals
 */
extern void QUAD_BOARD_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_PinConfig
 *  variable.
 */
extern void QUAD_BOARD_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void QUAD_BOARD_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void QUAD_BOARD_initPWM(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void QUAD_BOARD_initUART(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void QUAD_BOARD_initWatchdog(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void QUAD_BOARD_initSPI(void);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_BOARD_H */
