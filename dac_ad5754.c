/********************************************************************************
 *
 * dac_ad5754.c - Drivers for the ADI ad5754 DAC.
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
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ********************************************************************************/

//*****************************************************************************
//
//! \defgroup DAC_ad5754 ADC
//! \brief Drivers for the ADI ad5754 DAC.
//! Interfaces the TI tiva C Launchpad with the ADI dac5754 digital to analog
//! converter.
//! Files:
//!  dac_ad5754.c
//!  dac_ad5754.h
//!
//! \author Curtis Mayberry
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup DAC_ad5754
//! @{
//
//*****************************************************************************
 
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"

// Tivaware
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"
#include "driverlib/udma.h"

// GTBE Lib
#include "dac_ad5754.h"

//*****************************************************************************
//
//! \internal
//! Checks an SSI base address.
//!
//! \param ui32Base specifies the SSI module base address.
//!
//! This function determines if a SSI module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static bool
_SSIBaseValid(uint32_t ui32Base)
{
    return((ui32Base == SSI0_BASE) || (ui32Base == SSI1_BASE) ||
           (ui32Base == SSI2_BASE) || (ui32Base == SSI3_BASE));
}
#endif


// #define Elements_in(arrayname) (sizeof arrayname/sizeof *arrayname)

/***************************************
 * DAC Functions: Standalone Operation *
 ***************************************/

/**
 * Initializes the DAC
 *  Sets the output voltage range, turns on the selected DACs and sets the control
 *  settings.
 *  NOTE: Make sure DAC_WRITE_DELAY is defined properly
 *
 *  Sets all DAC voltage output ranges to -5v to +5v
 *  turns on DAC output A and B
 *  Turns on thermal shutdown, output current clamp, and turns off SDO
 **/
 void DAC_initDAC(uint32_t rangeValue, uint32_t pwrSettingsValue, uint32_t SysClkFreq) {
	 DAC_initSSI(SysClkFreq);
	 DAC_initCtlLDAC();
	 DAC_initCtlCLR();
	 // Initialize active low controls to output high
	 MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_FORCER_PIN|DAC_LDAC_QUAD_PIN|DAC_CLR_PIN, DAC_LDAC_FORCER_PIN|DAC_LDAC_QUAD_PIN|DAC_CLR_PIN);
	 DAC_setSettingsCtl(DAC_CTL_TSD | DAC_CTL_CLAMP | DAC_CTL_SDO_DIS);
	 MAP_SysCtlDelay(4000);
	 DAC_setRange(DAC_ADDR_ALL_AD, rangeValue);
	 MAP_SysCtlDelay(4000);
	 DAC_setPowerCtl(pwrSettingsValue);
	 MAP_SysCtlDelay(4000);
 }

 /**
  * initializes SSI0 for use with the DAC
  **/
 void DAC_initSSI(uint32_t SysClkFreq) {
 	uint32_t trashBin[1] = {0};

 	// Enable Peripherals
 	MAP_SysCtlPeripheralEnable(DAC_SSI_PERIPH);
 	MAP_SysCtlPeripheralEnable(DAC_SSI_GPIO_PERIPH);

 	// Set the pin muxing to SSI0 on port A
 	MAP_GPIOPinConfigure(DAC_SSI_CLK_PIN_CONFIG);
 	MAP_GPIOPinConfigure(DAC_SSI_FSS_PIN_CONFIG);
 	MAP_GPIOPinConfigure(DAC_SSI_RX_PIN_CONFIG);
 	MAP_GPIOPinConfigure(DAC_SSI_TX_PIN_CONFIG);
 	MAP_GPIOPinTypeSSI(DAC_SSI_GPIO_BASE,DAC_SSI_TX_PIN|DAC_SSI_RX_PIN|DAC_SSI_FSS_PIN|DAC_SSI_CLK_PIN);

 	// SPI Mode1
 	MAP_SSIConfigSetExpClk(DAC_SSI_BASE, SysClkFreq, SSI_FRF_MOTO_MODE_1,
 						   SSI_MODE_MASTER, DAC_FREQ_SSI_DATA, 8);

 	// Enable SSI uDMA
 	MAP_SSIEnable(DAC_SSI_BASE);
 	//SSIDMAEnable(DAC_SSI_BASE, SSI_DMA_TX);
 	while (MAP_SSIDataGetNonBlocking(DAC_SSI_BASE, &trashBin[0])) {
 	     }
 	//MAP_IntEnable(INT_SSI0);
 }

 /**
  * Initializes the SSI interrupt to load the DAC
  *  Uses the SSI TX end of transmission (EOT) interrupt to signal when to
  *  trigger the loading of the DAC.
  **/
  void DAC_initSSIint(void) {
	  //DAC_SSIIntEnableEOT(DAC_SSI_BASE); // Enables the TX end of transmission (EOT) feature
	  MAP_SSIIntEnable(DAC_SSI_BASE, DAC_SSI_INT_TYPE); // TM4C123: EOT just changes how TXFF behaves
	  													// TM4C1294: EOT is a seperate interrupt
	  MAP_IntEnable(DAC_SSI_INT);
  }

 /**
   * Enables the TX end of transmission (EOT) feature that allows the TXFF interrupt
   *  to trigger immediately when a transmission completes
   *  The EOT interrupt triggers the  TXFF interrupt
   *  This enable is in the CR1 register rather than the interrupt mask (IM) register
   *
   *  \note Used for EK-TM4C123GXL
   *
   **/
 void DAC_SSIIntEnableEOT(uint32_t ui32Base) {

	// Check the argument.
	ASSERT(_SSIBaseValid(ui32Base));

	// Write to the control register
	HWREG(ui32Base + SSI_O_CR1) |= 0x00000010;
 }

  /**
   * Interrupt handler for loading the DACs after updating them
   * \note Need to add the ISR to the NVIC table in the row labeled "SSIX Rx and Tx"
   **/
  void DAC_intHandlerSSI(void) {
	  //SSIIntClear(DAC_SSI_BASE, DAC_SSI_INT_TYPE)
	  HWREG(DAC_SSI_BASE + SSI_O_ICR) = DAC_SSI_INT_TYPE;
	  //MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);
	  HWREG(DAC_LDAC_GPIO_BASE + GPIO_O_DATA + (DAC_LDAC_FORCER_PIN <<2)) = 0x00; // Write low to GPIO PC7
	  MAP_SysCtlDelay(1);
	  //MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x02);
	  HWREG(DAC_LDAC_GPIO_BASE + GPIO_O_DATA + (DAC_LDAC_FORCER_PIN <<2)) = DAC_LDAC_FORCER_PIN; // Write high to GPIO PC7
  }

  /**
   * Interrupt handler for loading the DACs with the timer after updating them
   * \note Need to add the ISR to the NVIC table in the row labeled "SSIX Rx and Tx"
   **/
  void DAC_intHandlerSSItimer(void) {
	  //SSIIntClear(DAC_SSI_BASE, DAC_SSI_INT_TYPE)
	  HWREG(DAC_SSI_BASE + SSI_O_ICR) = DAC_SSI_INT_TYPE;
	  MAP_TimerLoadSet(DAC_LDAC_FORCER_TIMER_BASE, DAC_LDAC_FORCER_TIMER, 50);
	  //TimerEnable(DAC_LDAC_FORCER_TIMER_BASE, DAC_LDAC_FORCER_TIMER);
	  HWREG(DAC_LDAC_FORCER_TIMER_BASE + TIMER_O_CTL) |= DAC_LDAC_FORCER_TIMER & (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
  }


/**
 * Reads the SSI control register 1
 *
 * \param ui32Base - The base address of the SSIn peripheral
 *
 * \return SSI CR1 register value
 **/
 uint32_t DAC_getSSInSettingsCR1(uint32_t ui32Base) {
	 return(HWREG(ui32Base + SSI_O_CR1));
 }

/**
 * Initialize LDAC Controls
 *  initializes: ~LDAC_FORCER, ~LDAC_Quad
 *  \note sets ~LDAC_FORCER and ~LDAC_Quad
 **/
 void DAC_initCtlLDAC(void) {
	 MAP_SysCtlPeripheralEnable(DAC_LDAC_GPIO_PERIPH);
	 MAP_GPIOPinTypeGPIOOutput(DAC_LDAC_GPIO_BASE, DAC_LDAC_QUAD_PIN | DAC_LDAC_FORCER_PIN);
	 MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_QUAD_PIN | DAC_LDAC_FORCER_PIN, DAC_LDAC_QUAD_PIN | DAC_LDAC_FORCER_PIN); // Initialize active low control to output high
 }

/**
 * Initialize clear Control
 *  initializes: ~CLR - control to clear the DAC output
 *  \note sets ~CLR
 **/
void DAC_initCtlCLR(void) {
	MAP_SysCtlPeripheralEnable(DAC_CLR_GPIO_PERIPH);
	MAP_GPIOPinTypeGPIOOutput(DAC_CLR_GPIO_BASE, DAC_CLR_PIN);
	MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_CLR_PIN, DAC_CLR_PIN); // Initialize active low control to output high
 }

/**
 * Sets the voltage output range value of the given dac
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 *
 *	 Inputs:
 *	  \param dacAddress (uint32_t): The dac selected to have its range set
 *	         values: DAC_ADD_A, DAC_ADD_B, DAC_ADD_C, DAC_ADD_ALL
 *	  \param rangeValue (uint32_t): the output voltage range value to select
 **/
 void DAC_setRange(uint32_t dacAddress, uint32_t rangeValue) {
	 MAP_SSIDataPut(DAC_SSI_BASE, (DAC_REG_RANGE << 3)|dacAddress); // Input reg Command
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // First data byte (don't care)
	 MAP_SSIDataPut(DAC_SSI_BASE, rangeValue); // Second data byte (range value)
 }

// uint32_t getRange(uint32_t dacAddress)

/**
 * Sets the power control settings to power up or down each DAC
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 *
 *  \param pwrSettingsValue powers up the selected DAC(s)
 *   Must be the bitwise or of one or more of the following values:
 *   DAC_PUA, DAC_PUB, DAC_PUC, DAC_PUD
 *   Those DACs not included will be cleared.
 **/
 void DAC_setPowerCtl(uint32_t pwrSettingsValue) {
	 MAP_SSIDataPut(DAC_SSI_BASE, (DAC_REG_PWR << 3)); // Input reg Command
	 MAP_SSIDataPut(DAC_SSI_BASE, (pwrSettingsValue >> 8)); // First data byte
	 MAP_SSIDataPut(DAC_SSI_BASE, pwrSettingsValue); // Second data byte
 }

 // uint32_t DAC_getPowerSettings(uint32_t dacAddress);
 // uint32_t DAC_getPowerStatus(uint32_t dacAddress);

/**
 * Sets the DAC control register settings
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 *
 *  \param ctlSettingsValue sets the selected settings.
 *  Must be the bitwise or of one or more of the following values:
 *  DAC_CTL_TSD, DAC_CTL_CLAMP, DAC_CTL_CLR_SEL
 *  Those settings not included will be cleared.
 **/
 void DAC_setSettingsCtl(uint32_t ctlSettingsValue) {
	 MAP_SSIDataPut(DAC_SSI_BASE, (DAC_REG_CTL << 3) | DAC_CTL_SETTINGS); // Input reg Command
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // First data byte (don't care)
	 MAP_SSIDataPut(DAC_SSI_BASE, ctlSettingsValue); // Second data byte (settings)
 }

 // uint32_t DAC_readSettingsCtl(void)

/**
 * Clears the output of DACs A-D by writing to the control register.
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 **/
 void DAC_clearDACs(void) {
	 MAP_SSIDataPut(DAC_SSI_BASE, (DAC_REG_CTL << 3) | DAC_CTL_CLR_DATA); // Input reg Command
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // First data byte (don't care)
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // Second data byte (don't care)
  }

/**
 * Clears the output of all 8 DACs by pulling ~CLR pins low.
 **/
 void DAC_clearDACsPin(void) {
	 MAP_SysCtlDelay(4);
	 MAP_GPIOPinWrite(DAC_CLR_GPIO_BASE, DAC_CLR_PIN, 0x00);
	 MAP_SysCtlDelay(1);
	 MAP_GPIOPinWrite(DAC_CLR_GPIO_BASE, DAC_CLR_PIN, DAC_CLR_PIN);
 }

/** Loads data into the DACs from the data registers of all 4 DACs
 *  /note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 **/
 void DAC_loadDACs(void) {
	 MAP_SSIDataPut(DAC_SSI_BASE, (DAC_REG_CTL << 3) | DAC_CTL_LOAD_DATA); // Input reg Command
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // First data byte (don't care)
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // Second data byte (don't care)
	 MAP_SysCtlDelay(5000);
 }

/**
 * Loads all 4 dacs with the value currently in their data register by
 * pulling the ~LDAC pin low
 **/
 void DAC_loadDACsPin(void) {
	 //MAP_SysCtlDelay(5);
	 MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_FORCER_PIN, 0x00);
	 //MAP_SysCtlDelay(5);
	 MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_FORCER_PIN, 0x02);
 }

 /**
  * Loads all 4 dacs with the value currently in their data register by
  * pulling the ~LDAC_FORCER pin low
  *
  * \note Uses the timer in one shot mode to pulse ~LDAC_FORCER low
  **/
  void DAC_loadDACsPinTimer(void) {
	  MAP_TimerEnable(DAC_LDAC_FORCER_TIMER_BASE, DAC_LDAC_FORCER_TIMER);
  }

/**
 * Initializes Timers for one shot operation of the LDAC pulse
 *
 * \param enForcer - enables the LDAC_FORCER timer
 * \param enQuad - enables the LDAC_QUAD timer
 * \param pulseWidth - Number of clock cycles to pulse the LDAC signal low.
 *
 * \note Not available on the GTBE-TM4C123GXL
 **/
void DAC_initTimersLDAC(bool enForcer, bool enQuad, uint32_t pulseWidth) {
	// LDAC_FORCER Timer
	MAP_SysCtlPeripheralEnable(DAC_LDAC_GPIO_PERIPH);
	if(enForcer) {
		MAP_GPIOPinTypeTimer(DAC_LDAC_GPIO_BASE, DAC_LDAC_FORCER_PIN);
		MAP_GPIOPinConfigure(DAC_LDAC_FORCER_PIN_CONFIG);
		MAP_SysCtlPeripheralEnable(DAC_LDAC_FORCER_TIMER_PERIPH);
		MAP_TimerConfigure(DAC_LDAC_FORCER_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT | TIMER_CFG_B_ACT_CLRSETTO);
		MAP_TimerLoadSet(DAC_LDAC_FORCER_TIMER_BASE, DAC_LDAC_FORCER_TIMER, pulseWidth);
	}
	// LDAC_QUAD Timer
	if(enQuad) {
		MAP_GPIOPinTypeTimer(DAC_LDAC_GPIO_BASE, DAC_LDAC_QUAD_PIN);
		MAP_GPIOPinConfigure(DAC_LDAC_QUAD_PIN_CONFIG);
		MAP_SysCtlPeripheralEnable(DAC_LDAC_QUAD_TIMER_PERIPH);
		MAP_TimerConfigure(DAC_LDAC_QUAD_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_A_ACT_CLRSETTO);
		MAP_TimerLoadSet(DAC_LDAC_QUAD_TIMER_BASE, pulseWidth, TIMER_A);
	}
}

/**
 * Writes a nop command to the DAC
 *  This function can be used to skip writing to a DAC in a daisy chain configuration.
 **/
 void DAC_writeNop(void) {
	 MAP_SSIDataPut(DAC_SSI_BASE, (DAC_REG_CTL << 3) | DAC_CTL_NOP); // Input reg Command
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // First data byte (don't care)
	 MAP_SSIDataPut(DAC_SSI_BASE, 0x00000000); // Second data byte (don't care)
 }

/**
 * Updates the data in the DAC's data register to the specified voltage
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes
 *  Assumes a Vref = Vrefp-Vrefn = 2.5v
 *  The data must be loaded before it will be output
 *
 *  \param dacAddress Selected DAC(s)
 *  \param rangeValue the output voltage range value for the selected DACs
 *  \param bin Digital format selection.  Selected using pin BIN/~2sCOMP in hardware
 *         Values: OFFSET_BINARY or TWOS_COMPLEMENT
 *  \param voltage Voltage to be output on the chosen DAC(s)
 **/
 void DAC_updateDataVolt(uint32_t dacAddress, uint32_t rangeValue, _Bool bin,
		 	 	 	 	 float voltage) {
	 uint32_t data;
	 if(dacAddress == DAC_ADDR_NONE_AD) {
		 DAC_writeNop();
	 }
	 else {
		 // Unipolar: Vout = Vref * Gain * D/(2^N)
		 // Bipolar : Vout = Vref * Gain * D/(2^N) - gain * Vref/2
		 switch(rangeValue) {
			 case DAC_RANGE_P5V     : data = (uint32_t) (131072 * voltage + 5)/10; break;
			 case DAC_RANGE_P10V    : data = (uint32_t) (65536 * voltage + 5)/10; break;
			 case DAC_RANGE_P10P8V  : data = (uint32_t) (327680 * voltage + 27)/54; break;
			 case DAC_RANGE_PM5V    : data = (uint32_t) (65536 * (voltage + 5) + 5)/10; break;
			 case DAC_RANGE_PM10V   : data = (uint32_t) (32768 * (voltage + 10) + 5)/10; break;
			 case DAC_RANGE_PM10P8V : data = (uint32_t) (81920 * voltage + 884763)/432; break;
			 default                : data = 0;
		 }
		 // Convert to two's complement from binary offset format
		 if(!bin) {
			 data ^= 0x80000000; //inverts MSB (sign bit)
		 }

		 MAP_SSIDataPut(DAC_SSI_BASE, dacAddress);  // Input reg Command
		 MAP_SSIDataPut(DAC_SSI_BASE, (data >> 8)); // First data byte
		 MAP_SSIDataPut(DAC_SSI_BASE, data);        // Second data byte
 	 }
 }

/**
 * Updates the data in the DAC's data register
 *  The data must be loaded before it will be output
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes
 *
 *  \param dacAddress Selected DAC(s)
 *  \param data Digital output to be placed in the data register of the selected
 *  DAC(s)
 **/
 void DAC_updateDataDig(uint32_t dacAddress, uint32_t data) {
	 if(dacAddress == DAC_ADDR_NONE_AD) {
		 DAC_writeNop();
	 }
	 else {
		 MAP_SSIDataPut(DAC_SSI_BASE, dacAddress);  // Input reg Command
		 MAP_SSIDataPut(DAC_SSI_BASE, (data >> 8)); // First data byte
		 MAP_SSIDataPut(DAC_SSI_BASE, data);        // Second data byte
	 }
 }

#ifdef DAC_UDMA_MODE

 /**
  * Initializes the DAC with the SSI0 uDMA enabled
  *  Sets the output voltage range, turns on the selected DACs and sets the control
  *  settings.
  *  NOTE: Make sure DAC_WRITE_DELAY is defined properly
  *
  *  Sets all DAC voltage output ranges to -5v to +5v
  *  turns on DAC output A and B
  *  Turns on thermal shutdown, output current clamp, and turns off SDO
  **/
  void DAC_initDACuDMA(uint32_t rangeValue, uint32_t pwrSettingsValue, uint32_t SysClkFreq) {
	 DAC_initSSIuDMA(SysClkFreq);
 	 DAC_initCtlLDAC();
	 DAC_initCtlCLR();

	 DAC_setSettingsCtl(DAC_CTL_TSD | DAC_CTL_CLAMP | DAC_CTL_SDO_DIS);
 	 MAP_SysCtlDelay(4000);
 	 DAC_setRange(DAC_ADDR_ALL_AD, rangeValue);
 	 MAP_SysCtlDelay(4000);
 	 DAC_setPowerCtl(pwrSettingsValue);
 	 MAP_SysCtlDelay(4000);
  }

/**
 * initializes SSI0 for use with the DAC.  Also enables the SSI0 uDMA.
 **/
 void DAC_initSSIuDMA(uint32_t SysClkFreq) {
 	uint32_t trashBin[1] = {0};

 	// Enable Peripherals
 	MAP_SysCtlPeripheralEnable(DAC_SSI_PERIPH);
 	MAP_SysCtlPeripheralEnable(DAC_SSI_GPIO_PERIPH);

 	//Initialize the SSI0 TX uDMA
 	//SSIDMAEnable(DAC_SSI_BASE, SSI_DMA_TX);
 	// Set the pin muxing to SSI0 on port A
 	MAP_GPIOPinConfigure(DAC_SSI_CLK_PIN_CONFIG);
 	MAP_GPIOPinConfigure(DAC_SSI_FSS_PIN_CONFIG);
 	MAP_GPIOPinConfigure(DAC_SSI_RX_PIN_CONFIG);
 	MAP_GPIOPinConfigure(DAC_SSI_TX_PIN_CONFIG);
 	MAP_GPIOPinTypeSSI(DAC_SSI_GPIO_BASE,DAC_SSI_TX_PIN|DAC_SSI_RX_PIN|DAC_SSI_FSS_PIN|DAC_SSI_CLK_PIN);

 	// SPI Mode1
 	//MAP_SSIConfigSetExpClk(DAC_SSI_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_1,
 	//					   SSI_MODE_MASTER, DAC_FREQ_SSI_DATA, 8);
 	MAP_SSIConfigSetExpClk(DAC_SSI_BASE,SysClkFreq,SSI_FRF_MOTO_MODE_1,
 	 					   SSI_MODE_MASTER, DAC_FREQ_SSI_DATA, 8);
 	MAP_SSIEnable(DAC_SSI_BASE); // Enable SSI
 	//SSIDMAEnable(DAC_SSI_BASE, SSI_DMA_TX); // Enable SSI Tx uDMA
 	// Clear RX FIFO
 	while (MAP_SSIDataGetNonBlocking(DAC_SSI_BASE, &trashBin[0])) {
 	     }
 	//MAP_IntEnable(INT_SSI0);
 }

/**
 * setup of the uDMA to transfer 6 bytes of data using the auto mode.
 *
 * \note Be sure to enable the uDMA at the system level first
 *
 * \note The alternating global output buffers are named
 *  DAC_g_bufferPRI and DAC_g_bufferALT
 *
 **/
void DAC_inituDMAautoSSI(void) {
	// Enable the uDMA controller error interrupt.  This interrupt will occur
	// if there is a bus error during a transfer.
	//MAP_IntEnable(INT_UDMAERR);

    // Enable the uDMA controller.
    //uDMAEnable();

	//Setup channel selection
	uDMAChannelAssign(DAC_SSI_TX_UDMA_CHANNEL_SELECTION);

	// Place the uDMA channel attributes in a known state. These should already be disabled by default.
	uDMAChannelAttributeDisable(DAC_SSI_TX_UDMA_CHANNEL ,
	                            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
	                            (UDMA_ATTR_HIGH_PRIORITY |
	                            UDMA_ATTR_REQMASK));
	// Configure the control parameters for the SSI0 TX channel.  The channel
	// will be used to update the dac output
	uDMAChannelControlSet(DAC_SSI_TX_UDMA_CHANNEL  | UDMA_PRI_SELECT,
						  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
						  UDMA_ARB_8);

	// Set up the transfer parameters for the SSI0 Tx channel.  This will
	// configure the transfer buffers and the transfer size.
	uDMAChannelTransferSet(DAC_SSI_TX_UDMA_CHANNEL  | UDMA_PRI_SELECT,
						   UDMA_MODE_AUTO,
						   (void *)DAC_g_bufferPRI, (void *)(DAC_SSI_BASE + SSI_O_DR),
						   3);

	// Give the DAC high priority for use of the uDMA
	//uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX ,	UDMA_ATTR_HIGH_PRIORITY);
	//uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX , UDMA_ATTR_ALL);
	uDMAChannelAttributeEnable(DAC_SSI_TX_UDMA_CHANNEL , UDMA_ATTR_REQMASK);
	// Now the software channel is primed to start a transfer.  The channel
	// must be enabled.  For software based transfers, a request must be
	// issued.  After this, the uDMA memory transfer begins.
	uDMAChannelEnable(DAC_SSI_TX_UDMA_CHANNEL );
	IntEnable(INT_UDMA); // Enables the Software channel interrupt that triggers
						   // upon completion of a software transfer.
	// uDMAChannelRequest(UDMA_CHANNEL_SSI0RX);
}

void DAC_uDMAsw_ISR(void) {
	//DAC_loadDACsPin();
	//DAC_loadDACsPin();
	//uDMADisable();
	SSIDMADisable(DAC_SSI_BASE, SSI_DMA_TX); // Enable SSI Tx uDMA
	uDMAChannelDisable(DAC_SSI_TX_UDMA_CHANNEL);
}

#endif

/******************************************
 * DAC Functions: Daisy Chain Operation   *
 *  Supports the daisy chain of 2 devices *
 ******************************************/

 /**
  * Initializes the DAC in Daisy Chain Operation
  *  Sets the output voltage range, turns on the selected DACs and sets the
  *  control settings.
  *  /note Make sure DAC_WRITE_DELAY is defined properly
  *
  *  /param Output voltage range of all DACs
  *  Sets all DAC voltage output ranges to -5v to +5v
  *  turns on DAC output A and B
  *  Turns on thermal shutdown, output current clamp, and turns off SDO
  **/
  void DACd_initDAC(uint32_t rangeValue, uint32_t pwrSettingsValue, uint32_t SysClkFreq) {
 	 DAC_initSSI(SysClkFreq);
 	DAC_initCtlLDAC();
 	DAC_initCtlCLR();
 	 // Initialize active low controls to output high
 	MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_FORCER_PIN|DAC_LDAC_QUAD_PIN|DAC_CLR_PIN, DAC_LDAC_FORCER_PIN|DAC_LDAC_QUAD_PIN|DAC_CLR_PIN); // Initialize active low controls to output high
 	 DACd_setSettingsCtl((DAC_CTL_TSD | DAC_CTL_CLAMP) |
 			 	 	 	 (DAC_CTL_TSD_EH | DAC_CTL_CLAMP_EH | DAC_CTL_SDO_DIS_EH));
 	 MAP_SysCtlDelay(4000);
 	 DACd_setRange(DAC_ADDR_ALL_AH, rangeValue);
 	 MAP_SysCtlDelay(4000);
 	 DACd_setPowerCtl(pwrSettingsValue);
 	 MAP_SysCtlDelay(4000);
  }

/**
 * Sets the DAC control register settings in daisy chain operation
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 *
 *  \param ctlSettingsValue sets the selected settings for each DAC.
 *  It is the logical or of a value of a normal and a daisy chain operation parameter
 *  Must be the bitwise or of one or more of the following values:
 *  DAC_CTL_TSD, DAC_CTL_CLAMP, DAC_CTL_CLR_SEL, DAC_CTL_SKIP_AD
 *  with the birwise or of one or more of the following values:
 *  DAC_CTL_TSD_EH, DAC_CTL_CLAMP_EH, DAC_CTL_CLR_SEL_EH, DAC_CTL_SKIP_EH
 *
 *  Those settings not included will be cleared.
 **/
 void DACd_setSettingsCtl(uint32_t ctlSettingsValue) {
	 if((ctlSettingsValue & 0x0000FF00) == DAC_CTL_SKIP_EH) {
		 DAC_writeNop();
	 }
	 else {
		 DAC_setSettingsCtl(ctlSettingsValue>>8);
	 }
	 if((ctlSettingsValue & 0x000000FF) == DAC_CTL_SKIP_AD) {
		 DAC_writeNop();
	 }
	 else {
		 DAC_setSettingsCtl(ctlSettingsValue);
	 }
 }

/**
 * Sets the voltage output range value of the given daisy chained DACs
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 *
 *	 Inputs:
 *	  \param dacAddress (uint32_t): The dac selected to have its range set
 *	  \param rangeValue (uint32_t): the output voltage range value to select
 **/
 void DACd_setRange(uint32_t dacAddress, uint32_t rangeValue) {
	 if((rangeValue & 0x0000FF00) == DAC_RANGE_SKIP_EH) {
		 DAC_writeNop();
	 }
	 else {
		 DAC_setRange(dacAddress>>8, rangeValue>>8);
	 }
	 if((rangeValue & 0x000000FF) == DAC_RANGE_SKIP_AD) {
		 DAC_writeNop();
	 }
	 else {
		 DAC_setRange(dacAddress, rangeValue);
	 }
 }

/**
 * Sets the power control settings to power up or down each DAC when using Daisy
 * chain operation.
 *  \note A delay must be placed between consecutive DAC write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 *
 *  \param pwrSettingsValue powers up the selected DAC(s)
 *   Must be the bitwise or of a stand alone operation parameter
 *   and a daisy chain operation parameter
 *
 *   Those DACs not included will be cleared.
 **/
 void DACd_setPowerCtl(uint32_t pwrSettingsValue) {
	 if((pwrSettingsValue & 0xFFFF0000) == DAC_PWR_SKIP_EH) {
		 DAC_writeNop();
	 }
	 else {
		 DAC_setPowerCtl(pwrSettingsValue>>16);
	 }
	 if((pwrSettingsValue & 0x0000FFFF) == DAC_PWR_SKIP_AD) {
		 DAC_writeNop();
	 }
	 else {
		 DAC_setPowerCtl(pwrSettingsValue);
	 }
 }

/**
 * Clears the output of all 8 DACs by writing to the control registers.
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 **/
 void DACd_clearDACs_AH(void) {
	 DAC_clearDACs();
	 DAC_clearDACs();
 }

 /**
 * Clears the output of all 8 DACs by writing to the control registers.
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 **/
 void DACd_clearDACs_EH(void) {
	 DAC_clearDACs();
	 DAC_writeNop();
 }

 /** Loads data into the DACs from the data registers of DACs E-H
  *  \note A delay must be placed between consecutive dac write commands.
  *        This must be long enough to ensure that ~sync goes high between writes.
  **/
  void DACd_loadDACs_EH(void) {
 	 DAC_loadDACs();
 	 DAC_writeNop();
  }

/** Loads data into the DACs from the data registers of all 8 DACs
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes.
 **/
 void DACd_loadDACs_AH(void) {
	 DAC_loadDACs();
	 DAC_loadDACs();
 }

 /**
  * Loads all dacs E-H with the value currently in their data register by
  * pulling the ~LDAC pin low
  **/
 void DACd_loadDACsPin_EH(void) {
	MAP_SysCtlDelay(4);
	MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_QUAD_PIN, 0x00);
	MAP_SysCtlDelay(1);
	MAP_GPIOPinWrite(DAC_LDAC_GPIO_BASE, DAC_LDAC_QUAD_PIN, DAC_LDAC_QUAD_PIN);
 }
/**
 * Updates the data in the DAC's data register to the specified voltage
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes
 *  Assumes a Vref = Vrefp-Vrefn = 2.5v
 *  The data must be loaded before it will be output
 *
 *  \param dacAddress Selected DAC(s)
 *   Must be the bitwise or of a stand alone operation parameter
 *   and a daisy chain operation parameter
 *  \param rangeValue the output voltage range value for the selected DACs
 *   	   Must be the bitwise or of a stand alone operation parameter
 *   	   and a daisy chain operation parameter
 *  \param bin_AD Digital format selection for DACs A-D.
 *  \param bin_EH Digital format selection for DACs E-H.
 *  	   Selected using pin BIN/~2sCOMP in hardware
 *         Values: OFFSET_BINARY or TWOS_COMPLEMENT
 *  \param voltage_AD Voltage to be output on the chosen A-D DAC(s)
 *  \param voltage_EH Voltage to be output on the chosen E-H DAC(s)
 **/
 void DACd_updateDataVolt(uint32_t dacAddress, uint32_t rangeValue,
		 	 	 	 	 _Bool bin_AD, _Bool bin_EH,
		 	 	 	 	 float voltage_AD, float voltage_EH) {
	 DAC_updateDataVolt(dacAddress >> 8, rangeValue >> 8, bin_EH, voltage_EH);
	 DAC_updateDataVolt(dacAddress, rangeValue, bin_AD, voltage_AD);
 }

/**
 * Updates the data in the DAC's data register
 *  The data must be loaded before it will be output
 
 *  \note A delay must be placed between consecutive dac write commands.
 *        This must be long enough to ensure that ~sync goes high between writes
 *
 *  \param dacAddress Selected DAC(s)
 *   Must be the bitwise or of a stand alone operation parameter
 *   and a daisy chain operation parameter
 *
 *  \param data_AD Digital output to be placed in the data register of the DACs 
 *   A through D
 *
 *  \param data_EH Digital output to be placed in the data register of the DACs 
 *   E through H (daisy-chained device)
 **/
 void DACd_updateDataDig(uint32_t dacAddress, uint32_t data_AD, uint32_t data_EH) {
	 DAC_updateDataDig(dacAddress >> 8, data_EH);
	 DAC_updateDataDig(dacAddress, data_AD);
 }

 //*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
 
