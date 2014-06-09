/********************************************************************************
 *
 * dac_ad5754.h - headers for the ADI ad5754 DAC.
 *  Interfaces the TI tiva C Launchpad with the ADI dac5754 digital to analog
 *  converter
 *
 *  Uses SSI0 on the TI Tiva C Series TM4C123G LaunchPad
 *  Uses SPI protocol for the serial interface to the DAC
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  February 2014
 *
 *  Originally written for the MRIG uHRG gyroscope project
 *
 *    INPUTS
 *     ~LDAC_Forcer
 *     ~LDAC_Quad
 *     ~CLR
 *	   SERIAL (SPI)
 *     SCLK
 *     ~SYNC
 *     SDO
 *     SDIN
 *    OUTPUTS
 *     All 4 outputs of each DAC may be used
 *
 * Transfers over SSI to the DAC can be handled by the CPU or by the uDMA.
 *
 * uDMA Mode: In order to use the uDMA mode, DAC_UDMA_MODE must be defined in the
 *  main program.  In TI CCS this can be done by setting a predefined symbol in
 *  the project properties menu.  The current implementation only handles software
 *  initiated transfers in uDMA auto mode.  Two buffers can still be used in a
 *  ping-pong configuration (not ping-pong mode).
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ********************************************************************************/

#ifndef DAC_AD5754_H_
#define DAC_AD5754_H_

// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
#ifdef __cplusplus
extern "C"
{
#endif




/*******************
 * Port Assignment *
 *******************/
// Assigns port,pin and peripheral definitions depending on the MCU in use

#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
	// SSI
	#define DAC_SSI_PERIPH 				SYSCTL_PERIPH_SSI0
	#define DAC_SSI_BASE 				SSI0_BASE
	#define DAC_SSI_GPIO_PERIPH 		SYSCTL_PERIPH_GPIOA
	#define DAC_SSI_GPIO_BASE 			GPIO_PORTA_BASE
	#define DAC_SSI_CLK_PIN_CONFIG 		GPIO_PA2_SSI0CLK
	#define DAC_SSI_CLK_PIN		 		GPIO_PIN_2
	#define DAC_SSI_FSS_PIN_CONFIG 		GPIO_PA3_SSI0FSS
	#define DAC_SSI_FSS_PIN		 		GPIO_PIN_3
	#define DAC_SSI_RX_PIN_CONFIG 		GPIO_PA4_SSI0RX
	#define DAC_SSI_RX_PIN		 		GPIO_PIN_4
	#define DAC_SSI_TX_PIN_CONFIG 		GPIO_PA5_SSI0TX
	#define DAC_SSI_TX_PIN		 		GPIO_PIN_5
	#define DAC_SSI_INT					INT_SSI0
	#define DAC_SSI_INT_TYPE			SSI_TXFF
	#define DAC_SSI_TX_UDMA_CHANNEL 	UDMA_CHANNEL_SSI0TX

	// ~LDAC_FORCER and ~LDAC_Quad
	#define DAC_LDAC_GPIO_PERIPH 		SYSCTL_PERIPH_GPIOE
	#define DAC_LDAC_GPIO_BASE 			GPIO_PORTE_AHB_BASE
	#define DAC_LDAC_FORCER_PIN 		GPIO_PIN_1
	#define DAC_LDAC_QUAD_PIN 			GPIO_PIN_2
	// LDAC Timer Not available on this MCU
	#define DAC_LDAC_TIMER_BASE			TIMER2_BASE // Not available on this MCU
	#define DAC_LDAC_FORCER_TIMER		TIMER_A

	//

	// ~CLR
	#define DAC_CLR_GPIO_PERIPH 		SYSCTL_PERIPH_AHB_GPIOE
	#define DAC_CLR_GPIO_BASE 			GPIO_PORTE_BASE
	#define DAC_CLR_PIN 				GPIO_PIN_3

#endif

#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
	// SSI
	#define DAC_SSI_PERIPH 				SYSCTL_PERIPH_SSI3
	#define DAC_SSI_BASE 				SSI3_BASE
	#define DAC_SSI_GPIO_PERIPH 		SYSCTL_PERIPH_GPIOQ
	#define DAC_SSI_GPIO_BASE 			GPIO_PORTQ_BASE
	#define DAC_SSI_CLK_PIN_CONFIG 		GPIO_PQ0_SSI3CLK
	#define DAC_SSI_CLK_PIN		 		GPIO_PIN_0
	#define DAC_SSI_FSS_PIN_CONFIG 		GPIO_PQ1_SSI3FSS
	#define DAC_SSI_FSS_PIN		 		GPIO_PIN_1
	#define DAC_SSI_RX_PIN_CONFIG 		GPIO_PQ3_SSI3XDAT1
	#define DAC_SSI_RX_PIN		 		GPIO_PIN_3
	#define DAC_SSI_TX_PIN_CONFIG 		GPIO_PQ2_SSI3XDAT0
	#define DAC_SSI_TX_PIN		 		GPIO_PIN_2
	#define DAC_SSI_INT					INT_SSI3
	#define DAC_SSI_INT_TYPE			SSI_TXEOT
	#define DAC_SSI_TX_UDMA_CHANNEL 	15
	#define DAC_SSI_TX_UDMA_CHANNEL_SELECTION	UDMA_CH15_SSI3TX

	// ~LDAC_FORCER and ~LDAC_Quad
	#define DAC_LDAC_GPIO_PERIPH 			SYSCTL_PERIPH_GPIOM
	#define DAC_LDAC_GPIO_BASE 				GPIO_PORTM_BASE
	#define DAC_LDAC_FORCER_PIN 			GPIO_PIN_1
	#define DAC_LDAC_FORCER_PIN_CONFIG		GPIO_PM1_T2CCP1
	#define DAC_LDAC_FORCER_TIMER_PERIPH	SYSCTL_PERIPH_TIMER2
	#define DAC_LDAC_FORCER_TIMER_BASE		TIMER2_BASE
	#define DAC_LDAC_FORCER_TIMER			TIMER_B
	#define DAC_LDAC_QUAD_PIN 				GPIO_PIN_2
	#define DAC_LDAC_QUAD_PIN_CONFIG		GPIO_PM2_T3CCP0
	#define DAC_LDAC_QUAD_TIMER_PERIPH		SYSCTL_PERIPH_TIMER3
	#define DAC_LDAC_QUAD_TIMER_BASE		TIMER3_BASE
	#define DAC_LDAC_QUAD_TIMER				TIMER_A
	// ~CLR
	#define DAC_CLR_GPIO_PERIPH 			SYSCTL_PERIPH_GPIOM
	#define DAC_CLR_GPIO_BASE 				GPIO_PORTM_BASE
	#define DAC_CLR_PIN 					GPIO_PIN_0

#endif

/*********************
 * Hardware Settings *
 *********************/
// Binary Format for Bipolar Voltage Ouput Ranges
#define OFFSET_BINARY     true
#define TWOS_COMPLEMENT   false

/************************
 * Serial Port Settings *
 ************************/
//#define DAC_FREQ_SSI_DATA 10000000 // Can be Read by logic 8
#define DAC_FREQ_SSI_DATA 30000000
// Delay required after each DAC write before the next can be written.
// Must be sufficient to ensure that the ~SYNC goes high between writes.
#define DAC_WRITE_DELAY 5000*(100000/FREQ_SSI0_DATA)

/********************
 * Global Variables *
 ********************/
// The uDMA is used to transfer the DAC output data from memory to SSI0.
// These variables are declared but must also be defined in the main program file.
// (without extern)
#ifdef DAC_UDMA_MODE
	extern volatile uint8_t DAC_g_bufferPRI[3];
	extern volatile uint8_t DAC_g_bufferALT[3];
	extern volatile uint8_t DAC_g_bufferSel;
#endif

/******************
 * DAC Parameters *
 ******************/
// DAC Addresses
#define DAC_ADDR_A        0x00000000 // DAC A Address
#define DAC_ADDR_B        0x00000001 // DAC B Address
#define DAC_ADDR_C        0x00000002 // DAC C Address
#define DAC_ADDR_D        0x00000003 // DAC D Address
#define DAC_ADDR_ALL_AD   0x00000004 // Address all 4 DACs A through D
#define DAC_ADDR_NONE_AD  0x00000010 // Address all 4 DACs E through H
// DAC Registers
#define DAC_REG_DATA	  0x00000000 // DAC data register
#define DAC_REG_RANGE	  0x00000001 // DAC output voltage range register
#define DAC_REG_PWR		  0x00000002 // DAC power control register
#define DAC_REG_CTL		  0x00000003 // DAC control register
// DAC output voltage range values
#define DAC_RANGE_P5V     0x00000000 // 0v to +5v
#define DAC_RANGE_P10V    0x00000001 // 0v to +10v   (Not Supported by GT BE)
#define DAC_RANGE_P10P8V  0x00000002 // 0v to +10.8v (Not Supported by GT BE)
#define DAC_RANGE_PM5V    0x00000003 // -5v to +5v
#define DAC_RANGE_PM10V   0x00000004 // -10v to +10v (Not Supported by GT BE)
#define DAC_RANGE_PM10P8V 0x00000005 // -10.8v to +10.8v (Not Supported by GT BE)
// DAC Power Settings
#define DAC_PWR_PUA			  0x00000001 // DAC A power up bit (clear to put in powerdown mode)
#define DAC_PWR_PUB			  0x00000002 // DAC B power up bit
#define DAC_PWR_PUC			  0x00000004 // DAC C power up bit
#define DAC_PWR_PUD			  0x00000008 // DAC D power up bit
#define DAC_PWR_ALL_AD		  0x0000000F // DAC A-D power up bit
#define DAC_PWR_TSD			  0x00000020 // Thermal shutdown alert (read only)
#define DAC_PWR_OCA			  0x00000080 // DAC A overcurrent alert (read only)
#define DAC_PWR_OCB			  0x00000100 // DAC B overcurrent alert (read only)
#define DAC_PWR_OCC			  0x00000200 // DAC C overcurrent alert (read only)
#define DAC_PWR_OCD			  0x00000400 // DAC D overcurrent alert (read only)
// DAC Control Commands
#define DAC_CTL_NOP		  0x00000000 // No operation: Does nothing
#define DAC_CTL_SETTINGS  0x00000001 // Clears the DAC outputs
#define DAC_CTL_CLR_DATA  0x00000004 // Clears the DAC outputs
#define DAC_CTL_LOAD_DATA 0x00000005 // Loads the DAC outputs with the data in the data registers
// DAC Control Settings
#define DAC_CTL_TSD		  0x00000008 // Thermal Shutdown enable
#define DAC_CTL_CLAMP	  0x00000004 // Current clamp enable, clamps current to 20mA
#define DAC_CTL_CLR_SEL	  0x00000002 // Clear setting selection: selects the voltage that a cleared output is set to
									 // CLR_SEL | Unipolar Output Range | Bipolar Output Range
									 //    0	|         0v		    |          0v
									 //    1	|	   Midscale			|	Negative full scale
#define DAC_CTL_SDO_DIS	  0x00000001 // SDO disable: disables the serial output (Clear to enable SDO)
// uDMA operation
#define DAC_BUFFER_SEL_PRI 0x00000000 // Primary DAC buffer selected
#define DAC_BUFFER_SEL_ALT 0x00000001 // Primary DAC buffer selected

/*****************************************
 * DAC Parameters: Daisy Chain Operation *
 *****************************************/
// DAC Addresses
#define DAC_ADDR_E         0x00000000 // DAC E Address
#define DAC_ADDR_F         0x00000100 // DAC F Address
#define DAC_ADDR_G         0x00000200 // DAC G Address
#define DAC_ADDR_H         0x00000300 // DAC H Address
#define DAC_ADDR_ALL_EH    0x00000400 // Address all 4 DACs E through H
#define DAC_ADDR_ALL_AH    0x00000404 // Address all 4 DACs E through H
#define DAC_ADDR_NONE_EH   0x00001000 // Address all 4 DACs E through H

// DAC Control Settings
#define DAC_CTL_TSD_EH	   0x00000800 // Thermal Shutdown enable
#define DAC_CTL_CLAMP_EH   0x00000400 // Current clamp enable, clamps current to 20mA
#define DAC_CTL_CLR_SEL_EH 0x00000200 // Clear setting selection: selects the voltage that a cleared output is set to
									  // CLR_SEL | Unipolar Output Range | Bipolar Output Range
									  //    0	|         0v		    |          0v
									  //    1	|	   Midscale			|	Negative full scale
#define DAC_CTL_SDO_DIS_EH 0x00000100 // SDO disable: disables the serial output (Clear to enable SDO)
#define DAC_CTL_SKIP_AD    0x00000010 // All settings remain unchanged for DACs A-D
#define DAC_CTL_SKIP_EH    0x00001000 // All settings remain unchanged for DACs E-H
// DAC output voltage range values
#define DAC_RANGE_P5V_EH     0x00000000 // 0v to +5v
#define DAC_RANGE_P10V_EH    0x00000100 // 0v to +10v   (Not Supported by GT BE)
#define DAC_RANGE_P10P8V_EH  0x00000200 // 0v to +10.8v (Not Supported by GT BE)
#define DAC_RANGE_PM5V_EH    0x00000300 // -5v to +5v
#define DAC_RANGE_PM10V_EH   0x00000400 // -10v to +10v (Not Supported by GT BE)
#define DAC_RANGE_PM10P8V_EH 0x00000500 // -10.8v to +10.8v (Not Supported by GT BE)
#define DAC_RANGE_SKIP_AD    0x00000010 // All settings remain unchanged for DACs A-D
#define DAC_RANGE_SKIP_EH    0x00001000 // All settings remain unchanged for DACs E-H
// DAC Power Settings
#define DAC_PWR_PUE			  0x00010000 // DAC A power up bit (clear to put in powerdown mode)
#define DAC_PWR_PUF			  0x00020000 // DAC B power up bit
#define DAC_PWR_PUG			  0x00040000 // DAC C power up bit
#define DAC_PWR_PUH			  0x00080000 // DAC D power up bit
#define DAC_PWR_PUALL_EH	  0x000F0000 // DAC E-H power up bit
#define DAC_PWR_PUALL_AH	  0x000F000F // DAC A-H power up bit
#define DAC_PWR_TSD_EH		  0x00200000 // Thermal shutdown alert (read only)
#define DAC_PWR_OCE			  0x00800000 // DAC A overcurrent alert (read only)
#define DAC_PWR_OCF			  0x01000000 // DAC B overcurrent alert (read only)
#define DAC_PWR_OCG			  0x02000000 // DAC C overcurrent alert (read only)
#define DAC_PWR_OCH			  0x04000000 // DAC D overcurrent alert (read only)
#define DAC_PWR_SKIP_AD		  0x00001000 // All settings remain unchanged for DACs A-D
#define DAC_PWR_SKIP_EH		  0x10000000 // All settings remain unchanged for DACs E-H

/**************
 * Prototypes *
 **************/
// Stand alone Operation
extern void DAC_initDAC(uint32_t rangeValue, uint32_t pwrSettingsValue, uint32_t SysClkFreq);
extern void DAC_initSSI(uint32_t SysClkFreq);
extern void DAC_initSSIint(void);
extern void DAC_SSIIntEnableEOT(uint32_t ui32Base);
extern void DAC_intHandlerSSI(void);
extern void DAC_intHandlerSSItimer(void);
extern uint32_t DAC_getSSInSettingsCR1(uint32_t ui32Base);
extern void DAC_initCtlLDAC(void);
extern void DAC_initCtlCLR(void);
extern void DAC_setRange(uint32_t dacAddress, uint32_t rangeValue);
extern void DAC_setPowerCtl(uint32_t pwrSettingsValue);
extern void DAC_setSettingsCtl(uint32_t ctlSettingsValue);
extern void DAC_clearDACs(void);
extern void DAC_clearDACsPin(void);
extern void DAC_loadDACs(void);
extern void DAC_loadDACsPin(void);
extern void DAC_loadDACsPinTimer(void);
extern void DAC_initTimersLDAC(bool enForcer, bool enQuad, uint32_t pulseWidth);
extern void DAC_writeNop(void);
extern void DAC_updateDataVolt(uint32_t dacAddress, uint32_t rangeValue, _Bool BIN, float voltage);
extern void DAC_updateDataDig(uint32_t dacAddress, uint32_t data);
#ifdef DAC_UDMA_MODE
extern void DAC_initDACuDMA(uint32_t rangeValue, uint32_t pwrSettingsValue, uint32_t SysClkFreq);
extern void DAC_initSSIuDMA(uint32_t SysClkFreq);
extern void DAC_inituDMAautoSSI(void);
#endif
// Daisy Chain Operation
extern void DACd_initDAC(uint32_t rangeValue, uint32_t pwrSettingsValue, uint32_t SysClkFreq);
extern void DACd_setSettingsCtl(uint32_t ctlSettingsValue);
extern void DACd_setRange(uint32_t dacAddress, uint32_t rangeValue);
extern void DACd_setPowerCtl(uint32_t pwrSettingsValue);
extern void DACd_clearDACs_AH(void);
extern void DACd_clearDACs_EH(void);
extern void DACd_loadDACs_EH(void);
extern void DACd_loadDACs_AH(void);
extern void DACd_loadDACsPin_EH(void);
extern void DACd_updateDataVolt(uint32_t dacAddress, uint32_t rangeValue,
		 	 	 	 	 	 	_Bool bin_AD, _Bool bin_EH,
		 	 	 	 	 	 	float voltage_AD, float voltage_EH);
extern void DACd_updateDataDig(uint32_t dacAddress, uint32_t data_AD, uint32_t data_EH);
// DAC output timing timer
// void initTimer(void);


#ifdef __cplusplus
}
#endif
#endif /* DAC_AD5754_H_ */
