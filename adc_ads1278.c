/******************************************************************************
 *
 * adc_ads1278.c - Drivers for the TI ads1278 ADC.
 *  Interfaces the TI tiva C Launchpad with the TI ads1278 analog to digital
 *  converter.
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  rev1 Feb 2014
 *
 *  Originally written for the MRIG gyroscope project
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_udma.h"
#include "inc/hw_gpio.h"

// Tivaware
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"

// GTBE Lib
#include "adc_ads1278.h"
#include "dac_ad5754.h"
//#include "tw_extension.h"

/*****************
 * ADC Functions *
 *****************/

/**
 * Initializes the ADC
 *
 *  Gives clk freq = 25 MHz assuming a system clk freq = 50MHz
 *
 *  /param modeValue Sets the mode of the ADC
 *  values: ADC_MODE_HIGH_SPEED, ADC_MODE_HIGH_RESOL, ADC_MODE_LOW_POWER,
 *   ADC_MODE_LOW_SPEED
 *
 **/
 void ADC_initADC(uint32_t SysClkFreq) {
	 ADC_initControls();
	 ADC_setTestMode(ADC_TEST_MODE_NORMAL);
	 ADC_initNSYNC();
	 ADC_setSerialFormat(ADC_FORMAT_SPI_TDM_DYN);
	 ADC_setCLKfreq(ADC_FREQ_37MHZ_HS);  // 37 MHz is max frequency
	 ADC_initClkPWM(ADC_PWM_DIV2); // Sets actual device frequency
	 ADC_initSSI(SysClkFreq);
 }

/**
 * Initializes the control signals of the ADC
 **/
 void ADC_initControls(void) {
	 // TEST0, TEST1, CLKDIV
	 MAP_SysCtlPeripheralEnable(ADC_CTL1_GPIO_PERIPH);
	 MAP_GPIOPinTypeGPIOOutput(ADC_CTL1_GPIO_BASE, ADC_CTL1_CLKDIV_PIN|ADC_CTL1_TEST_PIN);
	 // MODE0, MODE1
	 MAP_SysCtlPeripheralEnable(ADC_CTL2_GPIO_PERIPH);
	 MAP_GPIOPinTypeGPIOOutput(ADC_CTL2_GPIO_BASE, ADC_CTL2_MODE1_PIN|ADC_CTL2_MODE0_PIN);
	 // FORMAT2, FORMAT1, FORMAT0
	 MAP_SysCtlPeripheralEnable(ADC_CTL3_GPIO_PERIPH);
	 MAP_GPIOPinTypeGPIOOutput(ADC_CTL3_GPIO_BASE, ADC_CTL3_FORMAT2_PIN|ADC_CTL3_FORMAT1_PIN|ADC_CTL3_FORMAT0_PIN);
 }

/**
 * Initialize ~DRDY pin as an input interrupt to trigger the read.
 * \note need to add the interrupt prototype to the NVIC
 **/
 void ADC_initDRDYint(void) {
	// ~DRDY
  	MAP_SysCtlPeripheralEnable(ADC_NDRDY_GPIO_PERIPH);
  	while(!SysCtlPeripheralReady(ADC_NDRDY_GPIO_PERIPH));
  	MAP_GPIOPinTypeGPIOInput(ADC_NDRDY_GPIO_BASE, ADC_NDRDY_PIN);
  	while(!SysCtlPeripheralReady(ADC_NDRDY_GPIO_PERIPH));
  	MAP_GPIOIntTypeSet(ADC_NDRDY_GPIO_BASE, ADC_NDRDY_PIN, GPIO_FALLING_EDGE);
  	while(!SysCtlPeripheralReady(ADC_NDRDY_GPIO_PERIPH));
  	GPIOIntEnable(ADC_NDRDY_GPIO_BASE, ADC_NDRDY_INT_PIN);
  	while(!SysCtlPeripheralReady(ADC_NDRDY_GPIO_PERIPH));
  	IntEnable(ADC_NDRDY_INT);
  	while(!SysCtlPeripheralReady(ADC_NDRDY_GPIO_PERIPH));
 }

 /**
  * Initialize ~DRDY pin as an input
  **/
  void ADC_initDRDY(void) {
	// ~DRDY
   	MAP_SysCtlPeripheralEnable(ADC_NDRDY_GPIO_PERIPH);
   	MAP_GPIOPinTypeGPIOInput(ADC_NDRDY_GPIO_BASE, ADC_NDRDY_PIN);
  }

/**
 * Initializes ~SYNC
 **/
  void ADC_initNSYNC(void) {
	 MAP_SysCtlPeripheralEnable(ADC_NSYNC_GPIO_PERIPH);
	 MAP_GPIOPinTypeGPIOOutput(ADC_NSYNC_GPIO_BASE, ADC_NSYNC_PIN);
	 MAP_GPIOPinWrite(ADC_NSYNC_GPIO_BASE, ADC_NSYNC_PIN, ADC_NSYNC_PIN);
  }

/**
 * initializes SSI for use with the ADC
 **/
 void ADC_initSSI(uint32_t SysClkFreq) {
 	uint32_t trashBin[1] = {0};

 	// Enable Peripherals
 	MAP_SysCtlPeripheralEnable(ADC_SSI_PERIPH);
 	MAP_SysCtlPeripheralEnable(ADC_SSI_GPIO_PERIPH);

 	// Set the pin muxing
 	MAP_GPIOPinConfigure(ADC_SSI_CLK_PIN_CONFIG);
 	MAP_GPIOPinConfigure(ADC_SSI_RX_PIN_CONFIG);
 	//MAP_GPIOPinConfigure(GPIO_PB7_SSI2TX);
 	//MAP_GPIOPinConfigure(GPIO_PB5_SSI2FSS);

 	MAP_GPIOPinTypeSSI(ADC_SSI_GPIO_BASE, ADC_SSI_RX_PIN|ADC_SSI_CLK_PIN);

 	// SPI Mode1
 	MAP_SSIConfigSetExpClk(ADC_SSI_BASE, SysClkFreq, SSI_FRF_MOTO_MODE_1,
 	 					   SSI_MODE_MASTER, ADC_FREQ_SSI_DATA, 8);
 	// Enable SSI
 	MAP_SSIEnable(ADC_SSI_BASE);
 	//SSIDMAEnable(ADC_SSI_BASE, SSI_DMA_RX); // Enable SSI uDMA
 	while (MAP_SSIDataGetNonBlocking(ADC_SSI_BASE, &trashBin[0])) {
 	     }
 	//MAP_IntEnable(INT_SSI2);
 }

/**
 * Sets the test mode of the ADC that tests the digital I/O
 *
 *  /param modeValue Sets the test mode of the ADC
 *  values: ADC_TEST_MODE_NORMAL, ADC_TEST_MODE_TEST
 **/
 void ADC_setTestMode(uint32_t testModeValue) {
	 MAP_GPIOPinWrite(ADC_CTL1_GPIO_BASE, ADC_CTL1_TEST_PIN, testModeValue);
 }

 /**
  * Sets the clock divider setting of the ADC
  *
  *  /param clkDivValue Sets the clock divider setting
  *  values: ADC_CLKDIV_HIGHF, ADC_CLKDIV_LOWF
  **/
 void ADC_setCLKfreq(uint32_t clkFreqValue) {
 	 switch(clkFreqValue) {
 	 case ADC_FREQ_37MHZ_HS :
 		ADC_setMode(ADC_MODE_HIGH_SPEED);
 		ADC_setCLKdiv(ADC_CLKDIV_HIGHF);
 		break;
 	 case ADC_FREQ_27MHZ_HR :
 		ADC_setMode(ADC_MODE_HIGH_RESOL);
 		ADC_setCLKdiv(ADC_CLKDIV_HIGHF);
 		break;
 	 case ADC_FREQ_27MHZ_LP :
 		ADC_setMode(ADC_MODE_LOW_POWER);
 		ADC_setCLKdiv(ADC_CLKDIV_HIGHF);
 		break;
 	 case ADC_FREQ_13p5MHZ_LP :
 		ADC_setMode(ADC_MODE_LOW_POWER);
 		ADC_setCLKdiv(ADC_CLKDIV_LOWF);
 		break;
 	 case ADC_FREQ_27MHZ_LS :
 		ADC_setMode(ADC_MODE_LOW_SPEED);
 		ADC_setCLKdiv(ADC_CLKDIV_HIGHF);
 		break;
 	 case ADC_FREQ_5p4MHZ_LS :
 		ADC_setMode(ADC_MODE_LOW_SPEED);
 		ADC_setCLKdiv(ADC_CLKDIV_LOWF);
 		break;
 	 default :
  		ADC_setMode(ADC_MODE_HIGH_SPEED);
  		ADC_setCLKdiv(ADC_CLKDIV_HIGHF);
 	 }
 }

/**
 * Sets the mode of the ADC
 *
 * 		Mode		   | MAX Fdata |
 * ---------------------------------
 * ADC_MODE_HIGH_SPEED |  144,531  |
 * ADC_MODE_HIGH_RESOL |   52,734  |
 * ADC_MODE_LOW_POWER  |   52,734  |
 * ADC_MODE_LOW_SPEED  |   10,547  |
 *
 *  /param modeValue Sets the mode of the ADC
 *  values: ADC_MODE_HIGH_SPEED, ADC_MODE_HIGH_RESOL, ADC_MODE_LOW_POWER,
 *   ADC_MODE_LOW_SPEED
 **/
 void ADC_setMode(uint32_t modeValue) {
	 MAP_GPIOPinWrite(ADC_CTL2_GPIO_BASE, ADC_CTL2_MODE1_PIN|ADC_CTL2_MODE0_PIN, modeValue);
 }

/**
 * Sets the clock divider setting of the ADC
 *
 *  /param clkDivValue Sets the clock divider setting
 *  values: ADC_CLKDIV_HIGHF, ADC_CLKDIV_LOWF
 **/
 void ADC_setCLKdiv(uint32_t CLKdivValue) {
	 MAP_GPIOPinWrite(ADC_CTL1_GPIO_BASE, ADC_CTL1_CLKDIV_PIN, CLKdivValue);
 }

/**
 * Initializes the M1PWM5 to generate the clock for the ADC
 *  Utilizes PWM module 1 output 5 (PF1)
 *
 *  Clock Setup Options
 *  *25MHz    - 50 MHz system clock,    sysClkFreqDiv = ADC_PWM_DIV2
 *  *26.67MHz - 80 MHz system clock,    sysClkFreqDiv = ADC_PWM_DIV3
 *  *33.33MHz - 66.67 MHz system clock, sysClkFreqDiv = ADC_PWM_DIV2
 *	 (Need 2.0v < DVDD < 2.2v)
 *  /param sysClkFreqDiv - frequency divider of the PWM
 *  values: ADC_PWM_DIV2, ADC_PWM_DIV3
 **/
 void ADC_initClkPWM(uint32_t sysClkFreqDiv) {
	// Setup GPIO config
	MAP_SysCtlPeripheralEnable(ADC_PWM_PERIPH);
	MAP_SysCtlPeripheralEnable(ADC_PWM_GPIO_PERIPH);

	MAP_GPIOPinTypePWM(ADC_PWM_GPIO_BASE, ADC_PWM_PIN);
	MAP_GPIOPinConfigure(ADC_PWM_PIN_CONFIG);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	// Configure the PWM generator for count down mode with immediate updates
	// to the parameters.
	MAP_PWMGenConfigure(ADC_PWM_BASE, ADC_PWM_GEN,
			            PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_STOP);

	#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
		// Set the period in number of clock cycles.
		MAP_PWMGenPeriodSet(ADC_PWM_BASE, ADC_PWM_GEN, 4);  //works freq = 20MHz
		//PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 8);  //works freq = 10MHz
		// Set the pulse width.
		MAP_PWMPulseWidthSet(ADC_PWM_BASE, ADC_PWM_OUT, 2); //works freq = 20MHz
		//PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 4); //works freq = 10MHz
	#endif
	#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
		// Set the period in number of clock cycles.
		MAP_PWMGenPeriodSet(ADC_PWM_BASE, ADC_PWM_GEN, 6);  //works freq = 20MHz
		// Set the pulse width.
		MAP_PWMPulseWidthSet(ADC_PWM_BASE, ADC_PWM_OUT, 3); //works freq = 20MHz
	#endif

	// Start the timer.
	MAP_PWMGenEnable(ADC_PWM_BASE, ADC_PWM_GEN);
	// Enable the output.
	MAP_PWMOutputState(ADC_PWM_BASE,  ADC_PWM_OUT_BIT, true);
 }

/**
 * Sets the serial format for the ADC output
 *  Options:
 *   SPI or frame sync format
 *   Dynamic or fixed output positions
 *   Parallel or serial (TDM) output
 *
 * 	\note Controls FORMAT2, FORMAT1, FORMAT0
 **/
void ADC_setSerialFormat(uint32_t serialFormatValue) {
	MAP_GPIOPinWrite(ADC_CTL3_GPIO_BASE, ADC_CTL3_FORMAT2_PIN | ADC_CTL3_FORMAT1_PIN | ADC_CTL3_FORMAT0_PIN, serialFormatValue);
}

/**
 * Initializes the SSI RX uDMA to transfer 3 bytes from the fifo to the data buffer
 **/
void ADC_initUDMAssiRX(void) {
	SSIDMAEnable(ADC_SSI_BASE, SSI_DMA_RX);

	uDMAChannelAssign(ADC_SSI_RX_UDMA_CHANNEL_ASSIGN);

	// Place the uDMA channel attributes in a known state. These should already be disabled by default.
	uDMAChannelAttributeDisable(ADC_SSI_RX_UDMA_CHANNEL,
	                            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
	                            (UDMA_ATTR_HIGH_PRIORITY |
	                            UDMA_ATTR_REQMASK));
	// Configure the control parameters for the SSI2 RX channel.  The channel
	// will be used to transfer the ADC measurements to memory.
	uDMAChannelControlSet(ADC_SSI_RX_UDMA_CHANNEL | UDMA_PRI_SELECT,
						  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
						  UDMA_ARB_4);
	// Set up the transfer parameters for the SSI2 Rx channel.  This will
	// configure the transfer buffers and the transfer size.
	uDMAChannelTransferSet(ADC_SSI_RX_UDMA_CHANNEL | UDMA_PRI_SELECT,
						   UDMA_MODE_BASIC,
						   (void *)(ADC_SSI_BASE + SSI_O_DR), (void *)(ADC_g_dataBufferBytes),
						   3);
	uDMAChannelAttributeEnable(ADC_SSI_RX_UDMA_CHANNEL, UDMA_ATTR_HIGH_PRIORITY);
	SSIIntEnable(ADC_SSI_BASE, SSI_DMARX);
	IntEnable(ADC_SSI_INT);

	uDMAChannelEnable(ADC_SSI_RX_UDMA_CHANNEL);
}

void ADC_SSIRXuDMA_ISR(void) {

	// Speed optimized implementation using direct register access

	//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x80);
	//HWREG(0x4005A000 +(0x80 <<2)) = 0x80; // Write high to GPIO PC7 (Need to turn on the bus)
	//HWREG(GPIO_PORTC_BASE + GPIO_O_DATA + (GPIO_PIN_7 <<2)) = GPIO_PIN_7; // Write high to GPIO PC7

	// clear interrupt
	// SSIIntClear(ADC_SSI_BASE, SSI_DMARX);
	HWREG(ADC_SSI_BASE + SSI_O_ICR) = SSI_DMARX;

	// uDMAChannelTransferSet(UDMA_CHANNEL_SSI2RX | UDMA_PRI_SELECT,
	// 						  UDMA_MODE_BASIC,
	//						  (void *)(SSI2_BASE + SSI_O_DR), (void *)ADC_g_dataBufferBytes,
	//						  3);
	// uDMAChannelTransferSet -> updates XFERSIZE and XFERMODE of the UDMA_CHANNEL_SSI2RX control word
	HWREG(HWREG(UDMA_CTLBASE) + (ADC_SSI_RX_UDMA_CHANNEL << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_BASIC | (((3-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));
	//HWREG(UDMA_CTLBASE_R + (UDMA_CHANNEL_SSI2RX << 4) + UDMA_CHCTL) |= (UDMA_CHCTL_XFERMODE_BASIC | (((3-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));

	//MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI2RX);
	HWREG(UDMA_ENASET) = 1 << (ADC_SSI_RX_UDMA_CHANNEL); // uDMAChannelEnable(UDMA_CHANNEL_SSI2RX);

	DAC_g_dataReady = true;

	// GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00);
	//HWREG(GPIO_PORTC_BASE + GPIO_O_DATA + (GPIO_PIN_7 <<2)) = 0x00; // Write low to GPIO PC7
}
