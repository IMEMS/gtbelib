/******************************************************************************
 *
 * tw_extension.h - Extension for the Tivaware Driver Library for TI Tiva C
 *  microcontrollers.
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  rev1 March 2014
 *
 *  Originally written for the MRIG gyroscope project
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/

#ifndef tw_extension_H_
#define tw_extension_H_

// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
#ifdef __cplusplus
extern "C"
{
#endif

/************
 * TypeDefs *
 ************/

/**
 * Data type for saving a float to flash memory
 **/
typedef struct {
	union {
		float fn[1];
		uint32_t uintn[1];
	} data;
	uint32_t address;
} floatFlash;

/*
typedef union {

float fn[1];

uint32_t uintn[1];

} float_32t_mask;
*/



/*******************
 * Port Assignment *
 *******************/
// Assigns port,pin and peripheral definitions depending on the MCU in use

#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
	// Processing Indicator
	#define TWE_PROCESSING_GPIO_PERIPH 		SYSCTL_PERIPH_GPIOC
	#define TWE_PROCESSING_GPIO_BASE		GPIO_PORTC_BASE
	#define TWE_PROCESSING_PIN				GPIO_PIN_7
	// RGB LED assignments defined in RGB driver header (RGB.h)
	// UART
	#define TWE_UART_COMM_PERIPH			SYSCTL_PERIPH_UART0
	#define TWE_UART_COMM_BASE				UART0_BASE
	#define TWE_UART_COMM_GPIO_PERIPH		SYSCTL_PERIPH_GPIOA
	#define TWE_UART_COMM_GPIO_BASE		GPIO_PORTA_BASE
	#define TWE_UART_COMM_RX_PIN_CONFIG	GPIO_PA0_U0RX
	#define TWE_UART_COMM_TX_PIN_CONFIG	GPIO_PA1_U0TX
	#define TWE_UART_COMM_RX_PIN			GPIO_PIN_0
	#define TWE_UART_COMM_TX_PIN			GPIO_PIN_1
	#define TWE_UART_COMM_UDMA_CHANNEL 	UDMA_CHANNEL_UART0TX

#endif

#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
	// Processing Indicator
	#define TWE_PROCESSING_GPIO_PERIPH 			SYSCTL_PERIPH_GPIOP
	#define TWE_PROCESSING_GPIO_BASE			GPIO_PORTP_BASE
	#define TWE_PROCESSING_PIN					GPIO_PIN_3
	// Error LED
	#define TWE_LED_PERIPH						SYSCTL_PERIPH_GPION
	#define TWE_LED_GPIO_BASE					GPIO_PORTN_BASE
	#define TWE_LED_D1_PIN						GPIO_PIN_1	// D1 on EK-TM4C1294XL
	#define TWE_LED_D2_PIN						GPIO_PIN_0	// D2 on EK-TM4C1294XL
	// UART Communication
	#define TWE_UART_COMM_PERIPH				SYSCTL_PERIPH_UART6
	#define TWE_UART_COMM_BASE					UART6_BASE
	#define TWE_UART_COMM_GPIO_PERIPH			SYSCTL_PERIPH_GPIOP
	#define TWE_UART_COMM_GPIO_BASE				GPIO_PORTP_BASE
	#define TWE_UART_COMM_RX_PIN_CONFIG			GPIO_PP0_U6RX
	#define TWE_UART_COMM_TX_PIN_CONFIG			GPIO_PP1_U6TX
	#define TWE_UART_COMM_RX_PIN				GPIO_PIN_0
	#define TWE_UART_COMM_TX_PIN				GPIO_PIN_1
	#define TWE_UART_COMM_UDMA_CHANNEL 			11
	#define TWE_UART_COMM_UDMA_CHANNEL_ASSIGN 	UDMA_CH11_UART6TX
	//QSSI Communication
	#define twe_QSSI_COMM_BAUD					10000000	// MAX BAUD = 10MHz for slave mode
	#define twe_QSSI_COMM_PERIPH				SYSCTL_PERIPH_SSI1
	#define twe_QSSI_COMM_BASE					SSI1_BASE
	#define twe_QSSI_COMM_CLK_FSS_GPIO_PERIPH	SYSCTL_PERIPH_GPIOB
	#define twe_QSSI_COMM_CLK_FSS_GPIO_BASE		GPIO_PORTB_AHB_BASE
	#define twe_QSSI_COMM_XDAT01_GPIO_PERIPH	SYSCTL_PERIPH_GPIOE
	#define twe_QSSI_COMM_XDAT01_GPIO_BASE		GPIO_PORTE_AHB_BASE
	#define twe_QSSI_COMM_XDAT23_GPIO_PERIPH	SYSCTL_PERIPH_GPIOD
	#define twe_QSSI_COMM_XDAT23_GPIO_BASE		GPIO_PORTD_AHB_BASE
	#define twe_QSSI_COMM_CLK_PIN_CONFIG		GPIO_PB5_SSI1CLK
	#define twe_QSSI_COMM_CLK_PIN				GPIO_PIN_5
	#define twe_QSSI_COMM_FSS_PIN_CONFIG		GPIO_PB4_SSI1FSS
	#define twe_QSSI_COMM_FSS_PIN				GPIO_PIN_4
	#define twe_QSSI_COMM_DAT0_PIN_CONFIG		GPIO_PE4_SSI1XDAT0
	#define twe_QSSI_COMM_DAT0_PIN				GPIO_PIN_4
	#define twe_QSSI_COMM_DAT1_PIN_CONFIG		GPIO_PE5_SSI1XDAT1
	#define twe_QSSI_COMM_DAT1_PIN				GPIO_PIN_5
	#define twe_QSSI_COMM_DAT2_PIN_CONFIG		GPIO_PD4_SSI1XDAT2
	#define twe_QSSI_COMM_DAT2_PIN				GPIO_PIN_4
	#define twe_QSSI_COMM_DAT3_PIN_CONFIG		GPIO_PD5_SSI1XDAT3
	#define twe_QSSI_COMM_DAT3_PIN				GPIO_PIN_5
	//QSSI Communication uDMA
	#define twe_QSSI_COMM_RX_UDMA_CHANNEL_ASSIGN	UDMA_CH24_SSI1RX
	#define twe_QSSI_COMM_RX_UDMA_CHANNEL			24
	#define twe_QSSI_COMM_TX_UDMA_CHANNEL_ASSIGN	UDMA_CH25_SSI1TX
	#define twe_QSSI_COMM_TX_UDMA_CHANNEL			25
#endif

/**************
 * Parameters *
 **************/
// UART Communication
#define TWE_UART_TX_BUFFER_LENGTH  4
#define TWE_UART_TX_BUFFER_PRI     true
#define TWE_UART_TX_BUFFER_ALT     false

// QSSI Communication
#define TWE_QSSI_TX_BUFFER_LENGTH  4


/********************
 * Global Variables *
 ********************/
// uDMA
static uint32_t g_DMAErrCount = 0;
// UART Communication
extern volatile uint32_t twe_g_UARTtxBufferIdx;
extern volatile unsigned char twe_g_UARTtxBufferPRI[TWE_UART_TX_BUFFER_LENGTH];
extern volatile unsigned char twe_g_UARTtxBufferALT[TWE_UART_TX_BUFFER_LENGTH];
extern volatile bool twe_g_UARTtxBufferSelectFlag;
//extern volatile int32_t twe_g_UARTCommand; // UART Command Handling

// QSSI Communication
extern volatile unsigned char twe_g_QSSItxBuffer[TWE_QSSI_TX_BUFFER_LENGTH];


/**************
 * Prototypes *
 **************/
// Flash
extern int32_t twe_initFlash(uint32_t codeStart, uint32_t codeReserveLength,
		                     uint32_t dataStart, uint32_t dataEraseLength);
extern int32_t twe_eraseFlashRange(uint32_t startAddress, uint32_t eraseLength);
extern int32_t twe_protectFlashRange(uint32_t startAddress, uint32_t protectLength, tFlashProtection eProtect);
extern void twe_FLASH_badWriteISR(void);
// FPU
extern void twe_initFPU(void);
extern void twe_initFPUlazy(void);
// UART
extern void twe_initUART(uint32_t SysClkFreq, uint32_t baudRate);
extern void twe_initUARTtxUDMA(void);
// SysCtl
extern void twe_initSystem80MHz(void);
// uDMA
extern void twe_initUDMAcontroller(void);
extern void twe_uDMAErrorHandler(void);
// GPIO
extern void twe_initProcessingIndicator(void);
#ifdef PART_TM4C1294NCPDT
extern void twe_initErrorLED(void);
extern void twe_setErrorLED(void);
#endif
// RGB Extension
extern void twe_RGBInitSolid(uint32_t ui32Enable);
extern void twe_RGBInitSetGreen(void);
// SSI
extern void twe_SSIIntEnableEOT(uint32_t ui32Base);
extern void twe_initQSSI(uint32_t SysClkFreq, bool RXmode);
#ifdef QSSI_OUT_MODE
extern void twe_initQSSIuDMArx(void);
extern void twe_initQSSIuDMAtx(void);
#endif
// I2C
extern void twe_I2CMasterVerify(uint32_t ui32Base, bool burst, bool receive);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* tw_extension_H_ */
