/*
 *  ======== QUAD_BOARD.c ========
 *  This file is responsible for setting up the board specific items for the
 *  QUAD_BOARD board.
 */

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <driverlib/pin_map.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "QUAD_board.h"


#ifndef TI_DRIVERS_UART_DMA
#define TI_DRIVERS_UART_DMA 0
#endif

/*
 *  =============================== DMA ===============================
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[32];
static bool dmaInitialized = false;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct hwiStruct;

/*
 *  ======== dmaErrorHwi ========
 */
static Void dmaErrorHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== QUAD_BOARD_initDMA ========
 */
void QUAD_BOARD_initDMA(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if (!dmaInitialized) {

        Error_init(&eb);

        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(hwiStruct), INT_UDMAERR, dmaErrorHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't create DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(dmaControlTable);

        dmaInitialized = true;
    }
}

/*
 *  =============================== General ===============================
 */
/*
 *  ======== QUAD_BOARD_initGeneral ========
 */
void QUAD_BOARD_initGeneral(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}

/*
 *  =============================== GPIO ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOTiva_config, ".const:GPIOTiva_config")
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in QUAD_BOARD.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input pins */
    /* QUAD_BOARD_GPIO_SW1 */
    GPIOTiva_PF_4 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* QUAD_BOARD_GPIO_SW2 */
    GPIOTiva_PF_0 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
	/* QUAD_BOARD_ECHO */
	GPIOTiva_PD_6 | GPIO_CFG_INPUT | GPIO_CFG_IN_INT_BOTH_EDGES,
	/* QUAD_BOARD_RPM */
	GPIOTiva_PA_5 | GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING,

    /* Output pins */
	/* QUAD_BOARD_TRIGG */
	GPIOTiva_PD_7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW  | GPIO_CFG_OUT_LOW,
    /* QUAD_BOARD_LED_RED */
    GPIOTiva_PF_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    /* QUAD_BOARD_LED_BLUE */
    GPIOTiva_PF_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    /* QUAD_BOARD_LED_GREEN */
    GPIOTiva_PF_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
	/* QUAD_BOARD_POWER_33 */
	GPIOTiva_PD_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
	/*QUAD_BOARD_SPI_CE */
	GPIOTiva_PA_6 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
	/*QUAD_BOARD_SPI_CSN */
	GPIOTiva_PA_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,

};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in QUAD_BOARD.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /* QUAD_BOARD_GPIO_SW1 */
    NULL,  /* QUAD_BOARD_GPIO_SW2 */
	NULL,  /* QUAD_BOARD_ECHO */
	NULL,  /* QUAD_BOARD_RPM */
};

/* The device-specific GPIO_config structure */
const GPIOTiva_Config GPIOTiva_config = {
    .pinConfigs = (GPIO_PinConfig *) gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *) gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  ======== QUAD_BOARD_initGPIO ========
 */
void QUAD_BOARD_initGPIO(void)
{
    /* QUAD_BOARD_GPIO_SW2 - PF0 requires unlocking before configuration */
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_6;
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    /* Initialize peripheral and pins */
    GPIO_init();
}

/*
 *  =============================== I2C ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cTivaHWAttrs, ".const:i2cTivaHWAttrs")
#endif

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

/* I2C objects */
I2CTiva_Object i2cTivaObjects[QUAD_BOARD_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const I2CTiva_HWAttrs i2cTivaHWAttrs[QUAD_BOARD_I2CCOUNT] = {
    {I2C0_BASE, INT_I2C0, ~0 /* Interrupt priority */},
    {I2C2_BASE, INT_I2C2, ~0 /* Interrupt priority */}
};

const I2C_Config I2C_config[] = {
    {&I2CTiva_fxnTable, &i2cTivaObjects[0], &i2cTivaHWAttrs[0]},
	{&I2CTiva_fxnTable, &i2cTivaObjects[1], &i2cTivaHWAttrs[1]},
    {NULL, NULL, NULL}
};

/*
 *  ======== QUAD_BOARD_initI2C ========
 */
void QUAD_BOARD_initI2C(void)
{
    /* I2C1 Init */
    /* Enable the peripheral */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);


    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);

    I2C_init();
}

/*
 *  =============================== PWM ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PWM_config, ".const:PWM_config")
#pragma DATA_SECTION(pwmTivaHWAttrs, ".const:pwmTivaHWAttrs")
#endif

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTiva.h>
#include <driverlib/pwm.h>

PWMTiva_Object pwmTivaObjects[QUAD_BOARD_PWMCOUNT];

/* PWM configuration structure */
const PWMTiva_HWAttrs pwmTivaHWAttrs[QUAD_BOARD_PWMCOUNT] = {
	    {
	        PWM0_BASE,
	        PWM_OUT_0,
	        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
	    },
	    {
	        PWM0_BASE,
	        PWM_OUT_1,
	        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
	    },
	    {
	        PWM0_BASE,
	        PWM_OUT_2,
	        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
	    },
	    {
	        PWM0_BASE,
	        PWM_OUT_3,
	        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
	    }
};

const PWM_Config PWM_config[] = {
    {&PWMTiva_fxnTable, &pwmTivaObjects[0], &pwmTivaHWAttrs[0]},
    {&PWMTiva_fxnTable, &pwmTivaObjects[1], &pwmTivaHWAttrs[1]},
    {&PWMTiva_fxnTable, &pwmTivaObjects[2], &pwmTivaHWAttrs[2]},
    {&PWMTiva_fxnTable, &pwmTivaObjects[3], &pwmTivaHWAttrs[3]},
    {NULL, NULL, NULL}
};

/*
 *  ======== QUAD_BOARD_initPWM ========
 */
void QUAD_BOARD_initPWM(void)
{
    /* Enable PWM peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    /*
     * Enable PWM output on GPIO pins.  Board_LED1 and Board_LED2 are now
     * controlled by PWM peripheral - Do not use GPIO APIs.
     */
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    PWM_init();
}

/*
 *  =============================== UART ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartTivaHWAttrs, ".const:uartTivaHWAttrs")
#endif

#include <ti/drivers/UART.h>
#if TI_DRIVERS_UART_DMA
#include <ti/drivers/uart/UARTTivaDMA.h>

/* UART objects */
UARTTivaDMA_Object uartTivaObjects[QUAD_BOARD_UARTCOUNT];

/* UART configuration structure */
const UARTTivaDMA_HWAttrs uartTivaHWAttrs[QUAD_BOARD_UARTCOUNT] = {
	    {/* QUAD_BOARD_UART0 */
	        UART0_BASE,
	        INT_UART0,
	        ~0,        /* Interrupt priority */
	        UDMA_CH8_UART0RX,
	        UDMA_CH9_UART0TX,
	    },
	    {/* QUAD_BOARD_UART1 */
	        UART1_BASE,
	        INT_UART1,
	        ~0,        /* Interrupt priority */
			UDMA_CH22_UART1RX,
			UDMA_CH23_UART1TX,
	    },
	    {/* QUAD_BOARD_UART4 */
	        UART5_BASE,
	        INT_UART5,
	        ~0,        /* Interrupt priority */
			UDMA_CH6_UART5RX,
			UDMA_CH7_UART5TX,
	    },
	    {/* QUAD_BOARD_UART7 */
	        UART7_BASE,
	        INT_UART7,
	        ~0,        /* Interrupt priority */
			UDMA_CH20_UART7RX,
			UDMA_CH21_UART7TX,
	    }
};

const UART_Config UART_config[] = {
	    {
	        &UARTTivaDMA_fxnTable,
	        &uartTivaObjects[0],
	        &uartTivaHWAttrs[0]
	    },
	    {
	        &UARTTivaDMA_fxnTable,
	        &uartTivaObjects[1],
	        &uartTivaHWAttrs[1]
	    },
	    {
	        &UARTTivaDMA_fxnTable,
	        &uartTivaObjects[2],
	        &uartTivaHWAttrs[2]
	    },
	    {
	        &UARTTivaDMA_fxnTable,
	        &uartTivaObjects[3],
	        &uartTivaHWAttrs[3]
	    },
    {NULL, NULL, NULL}
};

#else
#include <ti/drivers/uart/UARTTiva.h>

/* UART objects */
UARTTiva_Object uartTivaObjects[QUAD_BOARD_UARTCOUNT];
unsigned char uartTivaRingBuffer[32];
unsigned char uartTivaRingBuffer1[32];
unsigned char uartTivaRingBuffer5[32];
unsigned char uartTivaRingBuffer7[32];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[QUAD_BOARD_UARTCOUNT] = {
    {/* QUAD_BOARD_UART0 */
        .baseAddr = UART0_BASE,
        .intNum = INT_UART0,
        .intPriority = ~0,
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer,
        .ringBufSize = sizeof(uartTivaRingBuffer)
    },
    {/* QUAD_BOARD_UART1 */
        .baseAddr = UART1_BASE,
        .intNum = INT_UART1,
        .intPriority = ~0,
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer1,
        .ringBufSize = sizeof(uartTivaRingBuffer1)
    },
    {/* QUAD_BOARD_UART4 */
        .baseAddr = UART5_BASE,
        .intNum = INT_UART5,
        .intPriority = ~0,
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer5,
        .ringBufSize = sizeof(uartTivaRingBuffer5)
    },
    {/* QUAD_BOARD_UART7 */
        .baseAddr = UART7_BASE,
        .intNum = INT_UART7,
        .intPriority = ~0,
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer7,
        .ringBufSize = sizeof(uartTivaRingBuffer7)
    }
};

const UART_Config UART_config[] = {
	    {
	        &UARTTiva_fxnTable,
	        &uartTivaObjects[0],
	        &uartTivaHWAttrs[0]
	    },
	    {
	        &UARTTiva_fxnTable,
	        &uartTivaObjects[1],
	        &uartTivaHWAttrs[1]
	    },
	    {
	        &UARTTiva_fxnTable,
	        &uartTivaObjects[2],
	        &uartTivaHWAttrs[2]
	    },
	    {
	        &UARTTiva_fxnTable,
	        &uartTivaObjects[3],
	        &uartTivaHWAttrs[3]
	    },
    {NULL, NULL, NULL}
};
#endif /* TI_DRIVERS_UART_DMA */

/*
 *  ======== QUAD_BOARD_initUART ========
 */
void QUAD_BOARD_initUART(void)
{
    /* Enable and configure the peripherals used by the uart. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
/*
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    GPIOPinConfigure(GPIO_PE1_U7TX);
    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    /* Initialize the UART driver */

#if TI_DRIVERS_UART_DMA
    QUAD_BOARD_initDMA();
#endif
    UART_init();
}

/*
 *  =============================== Watchdog ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Watchdog_config, ".const:Watchdog_config")
#pragma DATA_SECTION(watchdogTivaHWAttrs, ".const:watchdogTivaHWAttrs")
#endif

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

/* Watchdog objects */
WatchdogTiva_Object watchdogTivaObjects[QUAD_BOARD_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[QUAD_BOARD_WATCHDOGCOUNT] = {
    /* QUAD_BOARD_WATCHDOG0 with 1 sec period at default CPU clock freq */
    {WATCHDOG0_BASE, INT_WATCHDOG, ~0 /* Interrupt priority */, 80000000},
};

const Watchdog_Config Watchdog_config[] = {
    {&WatchdogTiva_fxnTable, &watchdogTivaObjects[0], &watchdogTivaHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== QUAD_BOARD_initWatchdog ========
 *
 * NOTE: To use the other watchdog timer with base address WATCHDOG1_BASE,
 *       an additional function call may need be made to enable PIOSC. Enabling
 *       WDOG1 does not do this. Enabling another peripheral that uses PIOSC
 *       such as ADC0 or SSI0, however, will do so. Example:
 *
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
 *
 *       See the following forum post for more information:
 *       http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/176487/654390.aspx#654390
 */
void QUAD_BOARD_initWatchdog(void)
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}

/*
 *  =============================== SPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiTivaDMAHWAttrs, ".const:spiTivaDMAHWAttrs")
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

/* SPI objects */
SPITivaDMA_Object spiTivaDMAObjects[QUAD_BOARD_SPICOUNT];
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiTivaDMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
uint32_t spiTivaDMAscratchBuf[QUAD_BOARD_SPICOUNT];

/* SPI configuration structure */
const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[QUAD_BOARD_SPICOUNT] = {
    {
        SSI0_BASE,
        INT_SSI0,
        ~0,         /* Interrupt priority */
        &spiTivaDMAscratchBuf[0],
        0,
        UDMA_CHANNEL_SSI0RX,
        UDMA_CHANNEL_SSI0TX,
        uDMAChannelAssign,
        UDMA_CH10_SSI0RX,
        UDMA_CH11_SSI0TX
    }
};

const SPI_Config SPI_config[] = {
    {&SPITivaDMA_fxnTable, &spiTivaDMAObjects[0], &spiTivaDMAHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== QUAD_BOARD_initSPI ========
 */
void QUAD_BOARD_initSPI(void)
{
    /* SPI0 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    /* Need to unlock PF0 */
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
//    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | /* GPIO_PIN_3 | */
                                    GPIO_PIN_4 | GPIO_PIN_5);

    QUAD_BOARD_initDMA();
    SPI_init();
}
