/******************************************************************************
 * temp_ADT7310.c - Drivers for the ADI ADT7310 SPI digital temperature
 *  sensor
 *
 *  Hardware Connections
 *  Connected to SSI0
 *   SPI CLK -> PA2
 * 	 SPI CS  -> PA3
 *	 SPI RX  -> PA4
 *	 SPI TX  -> PA5
 *   Vdd     -> 3.3v
 *   gnd     -> gnd
 *
 *  Author: Curtis Mayberry (CLM)
 *  E-mail: Curtisma3@gmail.com
 *  Website: CurtisMayberry.com, Curtisma.org
 *  Georgia Tech IMEMS
 *
 *  Revisions
 *  rev1 Feb 2014 CLM
 *  rev2 Mar 2014 CLM
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/
 
//*****************************************************************************
//
//! \defgroup temp_adt7310 Temperature Sensor
//! \brief Drivers for the ADI ADT7310 SPI digital temperature sensor
//!  Files:
//!   temp_ADT7310.c
//!   temp_ADT7310.h
//!
//! \author Curtis Mayberry
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup temp_adt7310
//! @{
//
//*****************************************************************************
 
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

// Tivaware
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"

// Launchpad Drivers
#include "examples/boards/ek-tm4c123gxl/drivers/rgb.h"

// GTBE Lib
#include "temp_ADT7310.h"
#include "tw_extension.h"

/********************
 * Sensor Functions *
 ********************/
// Used to configure the sensor and process the temperature data collected from the sensor.

/**
 * Configures the temperature sensor for continuous read mode and reads the
 *  status register
 *  Utilizes SSI0
 *
 *  \param statusreg - A pointer to store the status information that is read
 **/
void temp_configSensorContinuousRead(uint32_t *statusreg) {
	uint32_t commandWriteConfigReg  = 0x00000C01;
	uint32_t commandReadStatusReg   = 0x00000040; // 0b01000000
	uint32_t commandReadStatusEmpty = 0x00000000;
	uint32_t trashBin[2];

	// Setup Configuration Register (register address: 0x01
	SSIDataPut(SSI0_BASE, commandWriteConfigReg);
	SSIDataPut(SSI0_BASE, commandReadStatusEmpty);
	// SysCtlDelay(4170000); // (3 cycles/loop) * (1/50MHz) * (4170000 loop cycles) = 250.2ms

	// Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	while(SSIDataGetNonBlocking(SSI0_BASE, &trashBin[0]))
	{
	}

	// Read status register (Address: 0x0)
	SSIDataPut(SSI0_BASE, commandReadStatusReg);
	SSIDataGet(SSI0_BASE, &trashBin[1]);
	SSIDataPut(SSI0_BASE, commandReadStatusEmpty);
	SSIDataGet(SSI0_BASE, statusreg);

	//Need to adjust for different processor frequencies
	SysCtlDelay(4170000); // (3 cycles/loop) * (1/50MHz) * (4170000 loop cycles) = 250.2ms
}

/**
 * Resets the SPI bus to clear any partial instructions, etc.
 *  Utilizes SSI0
 **/
void temp_resetSPI(void) {
	uint32_t commandReset  = 0x0000FFFF;
	uint32_t trashBin[1];

	// Reset by 32 cycles of 1 on Din
	SSIDataPut(SSI0_BASE, commandReset);
	SSIDataPut(SSI0_BASE, commandReset);

	// Empty Rx FIFO
	while(SSIDataGetNonBlocking(SSI0_BASE, &trashBin[0])) {
	}

	// Delay for the required 500us
	SysCtlDelay(8334); // (3 cycles/loop) * (1/50MHz) * (4170000 loop cycles) = 600us (500us needed after reset)
}

/**
 * Converts digital temperature reading to degrees Celcius
 *
 * \param reading -  13 bit digital temperature reading from the temperature sensor
 * \return Temperature in degrees Celcius
 **/
float temp_degC13bitReading(uint32_t reading) {
	reading = reading>>3;
	if (reading < 8192) {
		// Positive Reading
		return reading/16;
	}
	else {
		// Negative Reading
		return (reading-8192)/16;
	}
}

/**
 * Converts digital temperature reading to degrees Fahrenheit
 *
 * \param reading -  13 bit digital temperature reading from the temperature sensor
 * \return Temperature in degrees Fahrenheit
 **/
float temp_degF13bitReading(uint32_t reading) {
	reading = reading>>3;
	if (reading < 8192) {
		// Positive Reading
		return (9*reading+2560)/80; // degF = (9/5)*degC + 32
	}
	else {
		// Negative Reading
		return (9*reading-5632)/80;
	}
}

/******************
 * uDMA Functions *
 ******************/
// The uDMA is used to transfer each reading from the temperature sensor directly to memory without using the CPU.

/**
 * uDMA transfer error handler
 *  Need to add to NVIC table in the "uDMA Error" Row
 **/
void temp_uDMAErrorHandler(void) {
    uint32_t ui32Status;
    uint32_t RGBcolorError[3] = {0xFFFF, 0x0000, 0x0000};

    // Check for uDMA error bit
    ui32Status = uDMAErrorStatusGet();

    // If there is a uDMA error, then clear the error,
    // increment the error counter, and turn on the red led.
    if(ui32Status) {
        uDMAErrorStatusClear();
        //g_DMAErrCount++;
        RGBSet(RGBcolorError,0.5);
    }
}

/*****************
 * SSI Functions *
 *****************/
// SSI0 communicates with the temperature sensor using the SPI protocol

/**
 * Initializes the SSI0 to be used with the SSI0 uDMA
 **/
void temp_initSSI0uDMA(void) {
	uint32_t trashBin[1];
	// Enable Peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Set the pin muxing to SSI0 on port A
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2);

	// Bit rate:10,000 baud
	// SPI Mode3
	SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_3,SSI_MODE_MASTER,10000,16);

	// Enable SSI DMA
	SSIEnable(SSI0_BASE);
	SSIDMAEnable(SSI0_BASE, SSI_DMA_RX);
	while (ROM_SSIDataGetNonBlocking(SSI0_BASE, trashBin)) {
	     }
	ROM_IntEnable(INT_SSI0);
}

/*******************
 * Timer Functions *
 *******************/
// Timer 2 is used to read the temp sensor every 250ms.

/**
 * Initializes Timer 2A to control read sequence timing
 **/
void temp_initTimer2A(void) {
	uint32_t timerCount = 0;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Full-width periodic timer

	// Set the timer period to 250ms
	timerCount = (SysCtlClockGet()) /12500000;
	TimerLoadSet(TIMER2_BASE, TIMER_A, timerCount -1);

	//Start Timer Interrupt
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	TimerEnable(TIMER2_BASE, TIMER_A);
}

/**
 * Timer Interrupt handler
 *  Be sure that the interrupt is added to the NVIC by adding it to the
 *  "Timer 2 subtimer A” location in tm4c123gh6pm_startup_CCS.c
 **/
void temp_Timer2IntHandler(void) {
	uint32_t commandEmpty = 0x00000000;
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	SSIDataPut(SSI0_BASE, commandEmpty);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
