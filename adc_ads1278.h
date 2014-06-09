/******************************************************************************
 *
 * adc_ads1278.h - headers for the TI ads1278 ADC.
 *  Interfaces the TI tiva C Launchpad with the TI ads1278 analog to digital
 *  converter
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  February 2014
 *
 *  Originally written for the MRIG gyroscope project
 *
 *  GT BE Peripherals:
 *   ADC: ADS1278
 *    INPUTS
 *     TEST0		-> GPIO PE4: PE4
 *     TEST1		-> GPIO PE4: PE4
 *     CLKDIV		-> GPIO PE5: PE5
 *     ~SYNC		-> GPIO PE0: PE0
 *     CLK			-> M1PWM5  : PF1 (option: hardware jumper)
 *     CLK			-> SSI2_CLK: PB4 (option: hardware jumper)
 *     MODE0		-> GPIO PA6: PA6
 *	   MODE1		-> GPIO PA7: PA7
 *	   FORMAT0		-> GPIO PC4: PC4
 *	   FORMAT1		-> GPIO PC5: PC5
 *	   FORMAT2		-> GPIO PC6: PC6
 *	   SERIAL
 *	   SCLK  		-> SSI2_CLK: PB4
 *	   ~DRDY/ FSYNC -> GPIO PB5: PB5 (CS) (SPI Format: ~DRDY)
 *	   DOUT1        -> SSI2_RX:  PB6 (MISO)
 *	   DOUT2		-> GPIO PB0: PB2
 *	   DOUT3		-> GPIO PB1: PB1
 *	   DOUT4		-> GPIO PB2: PB2
 *	   DOUT5		-> GPIO PB3: PB3
 *	   DOUT6		-> GPIO PB7: PB7
 *	   DOUT7		-> GPIO PD6: PD6
 *	   DOUT8		-> GPIO PD7: PD7
 *
 *    OUTPUTS
 *	   NONE
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/

#ifndef ADC_ADS1278_H_
#define ADC_ADS1278_H_

// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
#ifdef __cplusplus
extern "C"
{
#endif

// Set the ADC SCLK rate
// note: SCLK may be run as fast as the CLK frequency.
// note: For best performance, limit fSCLK/fCLK to ratios of 1, 1/2, 1/4, 1/8, etc.
//#define ADC_FREQ_SSI_DATA 10000000 // Can be Read by logic 8
#define ADC_FREQ_SSI_DATA 20000000


/************
 * TypeDefs *
 ************/

/**
 * Data type for saving incoming 24bit data
 **/
typedef union {
		int32_t intn[1];
		uint32_t uintn[1];
		} ints24;

/*******************
 * Port Assignment *
 *******************/
// Assigns port,pin and peripheral definitions depending on the MCU in use

#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
	// Control 1: CLKDIV, TEST0, TEST1, ~SYNC
	#define ADC_CTL1_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOE
	#define ADC_CTL1_GPIO_BASE 		GPIO_PORTE_AHB_BASE
	#define ADC_CTL1_CLKDIV_PIN 	GPIO_PIN_5 // CLKDIV
	#define ADC_CTL1_TEST_PIN 		GPIO_PIN_4 // TEST0, TEST1
	#define ADC_CTL1_NSYNC_PIN 		GPIO_PIN_0 // ~SYNC
	// Control 2: MODE0, MODE1
	#define ADC_CTL2_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOA
	#define ADC_CTL2_GPIO_BASE 		GPIO_PORTA_AHB_BASE
	#define ADC_CTL2_MODE1_PIN 		GPIO_PIN_7 // MODE1
	#define ADC_CTL2_MODE0_PIN 		GPIO_PIN_6 // MODE0
	// Control 3: FORMAT2, FORMAT1, FORMAT0
	#define ADC_CTL3_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOC
	#define ADC_CTL3_GPIO_BASE 		GPIO_PORTE_AHB_BASE
	#define ADC_CTL2_FORMAT2_PIN 	GPIO_PIN_6 // MODE1
	#define ADC_CTL2_FORMAT1_PIN 	GPIO_PIN_5 // MODE0
	#define ADC_CTL2_FORMAT0_PIN 	GPIO_PIN_4 // MODE0
	// ~DRDY
	#define ADC_NDRDY_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOB
	#define ADC_NDRDY_GPIO_BASE	   	GPIO_PORTB_AHB_BASE
	#define ADC_NDRDY_PIN	   		GPIO_PIN_5
	#define ADC_NDRDY_INT_PIN	   	GPIO_INT_PIN_5
	#define ADC_NDRDY_INT			INT_GPIOB
	// ~SYNC
	#define ADC_NSYNC_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOE
	#define ADC_NSYNC_GPIO_BASE	   	GPIO_PORTE_AHB_BASE
	#define ADC_NSYNC_PIN	   		GPIO_PIN_0
	// Serial Communication
	#define ADC_SSI_PERIPH 			SYSCTL_PERIPH_SSI2
	#define ADC_SSI_BASE 			SSI2_BASE
	#define ADC_SSI_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOB
	#define ADC_SSI_GPIO_BASE	   	GPIO_PORTB_BASE
	#define ADC_SSI_CLK_PIN_CONFIG 	GPIO_PB4_SSI2CLK
	#define ADC_SSI_CLK_PIN 		GPIO_PIN_4
	#define ADC_SSI_RX_PIN_CONFIG 	GPIO_PB6_SSI2RX
	#define ADC_SSI_RX_PIN		 	GPIO_PIN_6
	#define ADC_SSI_INT		 		INT_SSI2
	//#define ADC_SSI_TX_PIN_CONFIG 	GPIO_PB7_SSI2TX
	//#define ADC_SSI_TX_PIN	   		GPIO_PIN_7
	// PWM Clock
	#define ADC_PWM_PERIPH 			SYSCTL_PERIPH_PWM1
	#define ADC_PWM_BASE 			PWM1_BASE
	#define ADC_PWM_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOF
	#define ADC_PWM_GPIO_BASE	   	GPIO_PORTF_BASE
	#define ADC_PWM_PIN_CONFIG 		GPIO_PF1_M1PWM5
	#define ADC_PWM_PIN		 		GPIO_PIN_1
	#define ADC_PWM_GEN		 		PWM_GEN_2
	#define ADC_PWM_OUT		 		PWM_OUT_5
	#define ADC_PWM_OUT_BIT	 		PWM_OUT_5_BIT
	// SSI uDMA
	#define ADC_SSI_RX_UDMA_CHANNEL 	   	UDMA_CHANNEL_SSI2RX
	#define ADC_SSI_RX_UDMA_CHANNEL_ASSIGN 	UDMA_CH12_SSI2RX
#endif

#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
		// Control 1: CLKDIV, TEST0, TEST1, ~SYNC
		#define ADC_CTL1_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOC
		#define ADC_CTL1_GPIO_BASE 		GPIO_PORTC_AHB_BASE
		#define ADC_CTL1_CLKDIV_PIN 	GPIO_PIN_7 // CLKDIV
		#define ADC_CTL1_TEST_PIN 		GPIO_PIN_6 // TEST0, TEST1

		// Control 2: MODE0, MODE1
		#define ADC_CTL2_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOH
		#define ADC_CTL2_GPIO_BASE 		GPIO_PORTH_AHB_BASE
		#define ADC_CTL2_MODE1_PIN 		GPIO_PIN_3 // MODE1
		#define ADC_CTL2_MODE0_PIN 		GPIO_PIN_2 // MODE0
		// Control 3: FORMAT2, FORMAT1, FORMAT0
		#define ADC_CTL3_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOF
		#define ADC_CTL3_GPIO_BASE 		GPIO_PORTF_AHB_BASE
		#define ADC_CTL3_FORMAT2_PIN 	GPIO_PIN_3 // MODE1
		#define ADC_CTL3_FORMAT1_PIN 	GPIO_PIN_2 // MODE0
		#define ADC_CTL3_FORMAT0_PIN 	GPIO_PIN_1 // MODE0
		// ~DRDY
		#define ADC_NDRDY_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOD
		#define ADC_NDRDY_GPIO_BASE	   	GPIO_PORTD_AHB_BASE
		#define ADC_NDRDY_PIN	   		GPIO_PIN_1
		#define ADC_NDRDY_INT_PIN	   	GPIO_INT_PIN_1
		#define ADC_NDRDY_INT			INT_GPIOD
		// ~SYNC
		#define ADC_NSYNC_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOM
		#define ADC_NSYNC_GPIO_BASE	   	GPIO_PORTM_BASE
		#define ADC_NSYNC_PIN	   		GPIO_PIN_3
		// Serial Communication
		#define ADC_SSI_PERIPH 			SYSCTL_PERIPH_SSI2
		#define ADC_SSI_BASE 			SSI2_BASE
		#define ADC_SSI_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOD
		#define ADC_SSI_GPIO_BASE	   	GPIO_PORTD_AHB_BASE
		#define ADC_SSI_CLK_PIN_CONFIG 	GPIO_PD3_SSI2CLK
		#define ADC_SSI_CLK_PIN 		GPIO_PIN_3
		#define ADC_SSI_RX_PIN_CONFIG 	GPIO_PD0_SSI2XDAT1
		#define ADC_SSI_RX_PIN		 	GPIO_PIN_0
		#define ADC_SSI_INT		 		INT_SSI2
		// PWM Clock
		#define ADC_PWM_PERIPH 			SYSCTL_PERIPH_PWM0
		#define ADC_PWM_BASE 			PWM0_BASE
		#define ADC_PWM_GPIO_PERIPH 	SYSCTL_PERIPH_GPIOG
		#define ADC_PWM_GPIO_BASE	   	GPIO_PORTG_BASE
		#define ADC_PWM_PIN_CONFIG 		GPIO_PG0_M0PWM4
		#define ADC_PWM_PIN		 		GPIO_PIN_0
		#define ADC_PWM_GEN		 		PWM_GEN_2
		#define ADC_PWM_OUT		 		PWM_OUT_4
		#define ADC_PWM_OUT_BIT	 		PWM_OUT_4_BIT
		// SSI uDMA
		#define ADC_SSI_RX_UDMA_CHANNEL 	   		UDMA_CHANNEL_SSI2RX
		#define ADC_SSI_RX_UDMA_CHANNEL_ASSIGN 		UDMA_CH12_SSI2RX
#endif

/*********************
 * Hardware Settings *
 *********************/
// Clock Input Selection (hardware jumper)
#define ADC_CLK_M1PWM5   0x00000000 // use M1PWM5 as the source of the clock
#define ADC_CLK_SSI2_CLK 0x00000001 // use SSI2_CLK as the source of the clock
// DAC Power Mode Control - Indicates which
#define ADC_PWUP1 		0x00000001
#define ADC_PWUP2 		0x00000002
#define ADC_PWUP3 		0x00000004
#define ADC_PWUP4 		0x00000008
#define ADC_PWUP5 		0x00000010
#define ADC_PWUP6 		0x00000020
#define ADC_PWUP7 		0x00000040
#define ADC_PWUP8		0x00000080
#define ADC_PWUP_ALL 	0x000000FF

/******************
 * ADC Parameters *
 ******************/
// Mode Selection
#define ADC_MODE_HIGH_SPEED  0x00000000 // 00 | MAX Fdata = 144,531
#define ADC_MODE_HIGH_RESOL  0x00000040 // 01 | MAX Fdata =  52,734
#define ADC_MODE_LOW_POWER   0x00000080 // 10 | MAX Fdata =  52,734
#define ADC_MODE_LOW_SPEED   0x000000C0 // 11 | MAX Fdata =  10,547
// CLK Frequency Settings
#define ADC_FREQ_37MHZ_HS	  0x00000000 // Fclk = 37MHz,   Mode = high speed mode
#define ADC_FREQ_27MHZ_HR	  0x00000001 // Fclk = 27MHz,   Mode = high resolution mode
#define ADC_FREQ_27MHZ_LP	  0x00000002 // Fclk = 27MHz,   Mode = low power mode
#define ADC_FREQ_13p5MHZ_LP	  0x00000004 // Fclk = 13.5MHz, Mode = low power mode
#define ADC_FREQ_27MHZ_LS	  0x00000008 // Fclk = 27MHz,   Mode = low speed mode
#define ADC_FREQ_5p4MHZ_LS	  0x00000010 // Fclk = 13.5MHz, Mode = low speed mode
// Test Mode
#define ADC_TEST_MODE_TEST	  0x00000010 // 11 | Continuity test of the digital I/O pins
#define ADC_TEST_MODE_NORMAL 0x00000000 // 00 | Normal operation
// Clock Input Divider
#define ADC_CLKDIV_HIGHF 	  0x00000020 // 37MHz (High-Speed mode)/otherwise 27MHz
#define ADC_CLKDIV_LOWF 	  0x00000000 // 13.5MHz (low-power)/5.4MHz (low-speed)
// PWM frequency divider
#define ADC_PWM_DIV2		  0x00000002 // PWM frequency = system clock /2
#define ADC_PWM_DIV3		  0x00000003 // PWM frequency = system clock /3

 											 //	Interface  |   Dout   | Data
 											 //	Protocol   |   Mode   | Position
// Serial Input Format						 //----------------------------------
#define ADC_FORMAT_SPI_TDM_DYN    0x00000000 //    SPI     |    TDM   | Dynamic
#define ADC_FORMAT_SPI_TDM_FIX    0x00000001 //    SPI     |    TDM   | Fixed
#define ADC_FORMAT_SPI_DISCRETE   0x00000002 //    SPI     | Discrete |
#define ADC_FORMAT_FSYNC_TDM_DYN  0x00000003 // Frame Sync |    TDM   | Dynamic
#define ADC_FORMAT_FSYNC_TDM_FIX  0x00000004 // Frame Sync |    TDM   | Fixed
#define ADC_FORMAT_FSYNC_DISCRETE 0x00000005 // Frame Sync | Discrete |
#define ADC_FORMAT_MODULATOR	   0x00000006 // Modulator Mode
// uDMA Functions
#define UDMA_CHANNEL_SSI2RX        12 // uDMA channel number for SSI2 RX channel
#define UDMA_DATA_BUFFER_BYTE2	   0  // uDMA RX data buffer byte 2
#define UDMA_DATA_BUFFER_BYTE1	   1
#define UDMA_DATA_BUFFER_BYTE0	   2

/********************
 * Global Variables *
 ********************/
extern volatile char ADC_g_dataBufferBytes[3];
extern volatile bool DAC_g_dataReady;

/**************
 * Prototypes *
 **************/
extern void ADC_initADC(uint32_t SysClkFreq);
extern void ADC_initControls(void);
extern void ADC_initDRDYint(void);
extern void ADC_initDRDY(void);
extern void ADC_initNSYNC(void);
extern void ADC_initSSI(uint32_t SysClkFreq);
extern void ADC_setTestMode(uint32_t testModeValue);
extern void ADC_setCLKfreq(uint32_t clkFreqValue);
extern void ADC_setMode(uint32_t modeValue);
extern void ADC_setCLKdiv(uint32_t CLKdivValue);
extern void ADC_initClkPWM(uint32_t sysClkFreqDiv);
extern void ADC_setSerialFormat(uint32_t serialFormatValue);
// uDMA
extern void ADC_initUDMAssiRX(void);
extern void ADC_SSIRXuDMA_ISR(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* ADC_ADS1278_H_ */


