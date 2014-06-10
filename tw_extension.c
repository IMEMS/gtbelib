/******************************************************************************
 *
 * tw_extension.c - Extension for the Tivaware Driver Library for TI Tiva C
 *  microcontrollers. For use with the TI Tiva C Launchpad EK-TM4C123GXL
 *  development board.
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

//*****************************************************************************
//
//! \defgroup tivaware_extension Tivaware Extension
//! \brief Extension for the Tivaware Driver Library for TI Tiva C MCUs
//!  Used with the Georgia Tech Back End (GTBE)
//!  GTBE-EK-TM4C123GXL
//!  GTBE-EK-TM4C1294XL
//!  Files:
//!   tw_extension.c
//!   tw_extension.h
//!
//! \author Curtis Mayberry
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup tivaware_extension
//! @{
//
//*****************************************************************************
 
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"

// Tivaware
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/flash.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"

// Launchpad Drivers
#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
#include "examples/boards/ek-tm4c123gxl/drivers/rgb.h"
#endif

// GTBE Lib
#include "tw_extension.h"

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

/*******************
 * Flash Functions *
 *******************/

/**
 * Initializes the Flash
 *  Protects the part of the flash memory from address codeStart through
 *   codeStart + codeReserveLength by setting to only execute (FlashExecuteOnly)
 *  Sets up the flash interrupt in case the program attempts to access the
 *   protected flash portions.  Make sure the ISR is added to the NVIC table
 *
 *  \note Make sure the NVIC is not contained in the protected code memory
 *
 *  \param codeStart - start location of flash memory that is to be reserved
 *  	it must land on a 2KB boundry
 *  \param codeReserveLength - length of code to protect in flash memory
 *  	it must land on a 2KB boundry, set to 0x0 to skip protection
 *  \param dataStart - start location of flash memory that is to be erased
 *  	it must land on a 1KB boundry
 *  \param dataEraseLength  - length of data to erase in flash memory
 *  	it must land on a 1KB boundry, set to 0x0 to skip erasure
 **/
int32_t twe_initFlash(uint32_t codeStart, uint32_t codeReserveLength,
					  uint32_t dataStart, uint32_t dataEraseLength) {
	int32_t status = 0;
	//protect Code area
	status = twe_protectFlashRange(codeStart, codeReserveLength, FlashExecuteOnly);
	// Erase Data in flash memory
	status = twe_eraseFlashRange(dataStart, dataEraseLength);
	//setup Flash Interrupt to handle any improper write attempts to flash
	FlashIntEnable(FLASH_INT_PROGRAM);
	return status;
}

/**
 * Erases the indicated flash memory
 
 *  \param startAddress Start location of flash memory that is to be erased
 *  	it must land on a 1KB boundry
 *  \param eraseLength Length of data to erase in flash memory
 *  	it must land on a 1KB boundry, set to 0x0 to skip erasure
 **/
int32_t twe_eraseFlashRange(uint32_t startAddress, uint32_t eraseLength) {
	int32_t status = 0;
	uint32_t modCheck;
	uint32_t block;
	modCheck = startAddress - startAddress/0x400;
	modCheck = modCheck + eraseLength/0x400;
	ASSERT(modCheck == 0);
	if(eraseLength > 0x0) {
		for(block = 0; block < (eraseLength/0x400); block++) {
			status = FlashErase(startAddress + 0x400 * block) // Erases each 1KB block
			ASSERT(status == 0);
		}
	}
	return status;
}

/**
 * Protects the indicated flash memory
 *  \param startAddress Start location of flash memory that is to be reserved
 *  	it must land on a 2KB boundry
 *  \param protectLength Length of code to protect in flash memory
 *  	it must land on a 2KB boundry, set to 0x0 to skip protection
 *  \param eProtect Protection type
 **/
int32_t twe_protectFlashRange(uint32_t startAddress, uint32_t protectLength, tFlashProtection eProtect) {
	int32_t status = 0;
	uint32_t modCheck;
	uint32_t block;
	// Check codeStart and codeReserveLength to ensure the protected memory
	//  block starts and ends on a 2KB boundry
	modCheck = startAddress - startAddress/0x800;
	modCheck = modCheck + protectLength/0x800;
	ASSERT(modCheck == 0);
	// Setup code protection
	if(protectLength > 0x0) {
		for(block = 0; block < (protectLength/0x800); block++) {
			status = FlashProtectSet(0x800 * block, eProtect);
			ASSERT(status == 0);
		}
	}
	return status;
}

/**
 * Interrupt handler (ISR) that executes when an improper write to flash is attempted
 **/
void twe_FLASH_badWriteISR(void) {
	while(1) {
		// Do nothing - save state for examination
	}
}

/******************
 * FPU Functions *
 ******************/

/**
 * Initializes the FPU
 **/
void twe_initFPU(void) {
	FPUEnable();
}

/**
 * Initializes the FPU with lazy stacking enabled
 **/
void twe_initFPUlazy(void) {
	// Lazy Stacking increases interrupt latency and stack usage
	//  (only need if doing floating pt. in interrupts)
	FPULazyStackingEnable();
	FPUEnable();
}

/******************
 * UART Functions *
 ******************/

/**
 * Initializes the UART
 *
 *  \param SysClkFreq - clock frequency of the system
 *
 *  \param baudRate - baud rate of the UART e.g. 115200 to connect to PC
 *
 *  \note UART is connected to the stellaris virtual serial port through the USB connection
 *
 *  \note Configuration:
 *   8 data bits
 *   one stop bit
 *   no parity
 **/
void twe_initUART(uint32_t SysClkFreq, uint32_t baudRate) {
	 MAP_SysCtlPeripheralEnable(TWE_UART_COMM_PERIPH);
	 MAP_SysCtlPeripheralEnable(TWE_UART_COMM_GPIO_PERIPH);

	 MAP_GPIOPinConfigure(TWE_UART_COMM_RX_PIN_CONFIG);
	 MAP_GPIOPinConfigure(TWE_UART_COMM_TX_PIN_CONFIG);
	 MAP_GPIOPinTypeUART(TWE_UART_COMM_GPIO_BASE, TWE_UART_COMM_RX_PIN | TWE_UART_COMM_TX_PIN);
	 /*
	 MAP_UARTConfigSetExpClk(TWE_UART_COMM_BASE, SysCtlClockGet(), 115200,
	 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	 MAP_UARTConfigSetExpClk(TWE_UART_COMM_BASE, SysClkFreq, 4608000,
	 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	  */
	 MAP_UARTConfigSetExpClk(TWE_UART_COMM_BASE, SysClkFreq, baudRate,
	 	 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
 }

#ifdef UART_OUT_MODE
 /**
  * Initializes the UART uDMA settings
  **/
 void twe_initUARTtxUDMA(void) {
	#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
		uDMAChannelAssign(UDMA_CH11_UART6TX);
	#endif
	UARTIntEnable(TWE_UART_COMM_BASE, UART_INT_TX);
	 // Place the uDMA channel attributes in a known state. These should already be disabled by default.
	 	uDMAChannelAttributeDisable(TWE_UART_COMM_UDMA_CHANNEL ,
	 	                            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
	 	                            (UDMA_ATTR_HIGH_PRIORITY |
	 	                            UDMA_ATTR_REQMASK));
	 	// Configure the control parameters for the UART0 TX channel.  The channel
	 	// will be used to send the output data
	 	uDMAChannelControlSet(TWE_UART_COMM_UDMA_CHANNEL  | UDMA_PRI_SELECT,
	 						  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
	 						  UDMA_ARB_8);

	 	// Set up the transfer parameters for the UART0 Tx channel.  This will
	 	// configure the transfer buffer and the transfer size.
	 	uDMAChannelTransferSet(TWE_UART_COMM_UDMA_CHANNEL  | UDMA_PRI_SELECT,
	 						   UDMA_MODE_AUTO,
	 						   (void *)twe_g_UARTtxBufferPRI, (void *)(TWE_UART_COMM_BASE + UART_O_DR),
	 						   TWE_UART_TX_BUFFER_LENGTH);

	 	uDMAChannelAttributeEnable(TWE_UART_COMM_UDMA_CHANNEL , UDMA_ATTR_REQMASK);
	 	// Now the software channel is primed to start a transfer.  The channel
	 	// must be enabled.  For software based transfers, a request must be
	 	// issued.  After this, the uDMA memory transfer begins.
	 	//uDMAChannelEnable(UDMA_CHANNEL_UART0TX | UART_INT_FE);
	 	//IntEnable(INT_UDMA); // Enables the Software channel interrupt that triggers
	 						 //  upon completion of a software transfer.
 }
#endif
/*
void uart0ISR(void) {
	HWREG(UART0_BASE + UART_O_ICR) |= 0x000017F2;
	if((twe_g_UARTtxBufferSelectFlag == TWE_UART_TX_BUFFER_PRI)) {
		twe_g_UARTtxBufferSelectFlag = TWE_UART_TX_BUFFER_ALT;
		//uDMAChannelTransferSet(UDMA_CHANNEL_UART0TX  | UDMA_PRI_SELECT,
		//					   UDMA_MODE_AUTO,
		//					   (void *)twe_g_UARTtxBufferALT, (void *)(UART0_BASE + UART_O_DR),
		//					   TWE_UART_TX_BUFFER_LENGTH);
		HWREG(HWREG(UDMA_CTLBASE) + (UDMA_CHANNEL_UART0TX << 4) + UDMA_O_SRCENDP) = (uint32_t)twe_g_UARTtxBufferALT + TWE_UART_TX_BUFFER_LENGTH - 1;
		HWREG(HWREG(UDMA_CTLBASE) + (UDMA_CHANNEL_UART0TX << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_AUTO | (((TWE_UART_TX_BUFFER_LENGTH-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));
	}
	else {
		twe_g_UARTtxBufferSelectFlag = TWE_UART_TX_BUFFER_PRI;
		//uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX  | UDMA_PRI_SELECT,
		//					   UDMA_MODE_AUTO,
		//					   (void *)twe_g_UARTtxBufferPRI, (void *)(SSI0_BASE + SSI_O_DR),
		//					   3);
		HWREG(HWREG(UDMA_CTLBASE) + (UDMA_CHANNEL_UART0TX << 4) + UDMA_O_SRCENDP) = (uint32_t)twe_g_UARTtxBufferPRI + TWE_UART_TX_BUFFER_LENGTH - 1;
		HWREG(HWREG(UDMA_CTLBASE) + (UDMA_CHANNEL_UART0TX << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_AUTO | (((TWE_UART_TX_BUFFER_LENGTH-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));
	}
}
*/

/**
 * Initializes the UART interrupt for commands
 **/
 /*
 void initUart0CommandInterrupt(void) {
	 MAP_UARTIntEnable(TWE_UART_COMM_BASE, UART_INT_RX);
 }
 */
/**
 * Receives commands for the device
 **/
 /*
 void uart0MonitorISR(void) {
	 twe_g_UARTCommand = UARTCharGetNonBlocking(UART0_BASE);
 }
*/


/****************************
 * System Control Functions *
 ****************************/
void twe_initSystem80MHz(void) {
	// Set system clock to 80 MHz (400MHz main PLL (divided by 5 - uses DIV400 bit)  [16MHz external xtal drives PLL]
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	MAP_SysCtlPeripheralClockGating(true); // Enable peripherals to operate when CPU is in sleep.
}

/******************
 * uDMA Functions *
 ******************/

/**
 * Initialize the uDMA controller at the system level.  It also enables it to continue
 * to run while the processor is in sleep.
 *
 * \note Individual uDMA channels need to be initialized seperately.
 *
 * \note Must also initialize the uDMA table immediately following this call.
 * i.e. MAP_uDMAControlBaseSet(uDMAcontrolTable);
 **/
void twe_initUDMAcontroller(void) {
	// Enable the uDMA controller at the system level.  Enable it to continue
	// to run while the processor is in sleep.
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	// Enable the uDMA controller error interrupt.  This interrupt will occur
	// if there is a bus error during a transfer.
	MAP_IntEnable(INT_UDMAERR);

    // Enable the uDMA controller.
    uDMAEnable();
    //IntEnable(INT_UDMA);
}


/**
 * uDMA Error Handler
 *
 * \note Need to add the ISR to the NVIC table in the row labeled "uDMA Error"
 *
 **/
void twe_uDMAErrorHandler(void) {
	uint32_t ui32Status;
	ui32Status = MAP_uDMAErrorStatusGet();
	if(ui32Status) {
		MAP_uDMAErrorStatusClear();
		g_DMAErrCount++;
	}
	uDMAChannelDisable(UDMA_CHANNEL_SSI0TX);
	SSIDMADisable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);
}

/******************
 * GPIO Functions *
 ******************/

/**
 * Initializes the processing indicator GPIO output
 **/
void twe_initProcessingIndicator(void) {
	 SysCtlPeripheralEnable(TWE_PROCESSING_GPIO_PERIPH);
	 GPIOPinTypeGPIOOutput(TWE_PROCESSING_GPIO_BASE, TWE_PROCESSING_PIN);
	 GPIOPinWrite(TWE_PROCESSING_GPIO_BASE, TWE_PROCESSING_PIN, 0x00);
}

#ifdef PART_TM4C1294NCPDT
/**
 * Initializes LED D1 on the EK-TM4C1294XL as an error indicator
 **/
void twe_initErrorLED(void) {
	SysCtlPeripheralEnable(TWE_LED_PERIPH);
	GPIOPinTypeGPIOOutput(TWE_LED_GPIO_BASE, TWE_LED_D1_PIN);
	GPIOPinWrite(TWE_LED_GPIO_BASE, TWE_LED_D1_PIN, 0x0);
}

/**
 * Turns on the error LED
 *
 * \note D1 on the EK-TM4C1294XL
 **/
void twe_setErrorLED(void) {
	MAP_GPIOPinWrite(TWE_LED_GPIO_BASE, TWE_LED_D1_PIN, TWE_LED_D1_PIN);
}
#endif

/***************************
 * RGB Extension Functions *
 ***************************/
#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL

/**
 *
 * Initializes the Timer and GPIO functionality associated with the RGB LED
 *  to provide a solid LED output that doesn't blink.
 *
 * \note This function is an extension of the rgb driver of the EK-TM4C123GXL
 * Firmware Package provided by Texas Instruments.
 *
 * \note Doesn't use WTIMER5, which normally controls the blinking of the LEDs
 *
 * \note The RGB library does use Timer 0B and Timer 1A and Timer 1B
 *
 * \param ui32Enable enables RGB immediately if set.
 *
 * This function must be called during application initialization to
 * configure the GPIO pins to which the LEDs are attached.  It enables
 * the port used by the LEDs and configures each color's Timer. It optionally
 * enables the RGB LED by configuring the GPIO pins and starting the timers.
 *
 * \return None.
 *
 **/
void twe_RGBInitSolid(uint32_t ui32Enable) {
    // Enable the GPIO Port and Timer for each LED
    MAP_SysCtlPeripheralEnable(RED_GPIO_PERIPH);
    MAP_SysCtlPeripheralEnable(RED_TIMER_PERIPH);

    MAP_SysCtlPeripheralEnable(GREEN_GPIO_PERIPH);
    MAP_SysCtlPeripheralEnable(GREEN_TIMER_PERIPH);

    MAP_SysCtlPeripheralEnable(BLUE_GPIO_PERIPH);
    MAP_SysCtlPeripheralEnable(BLUE_TIMER_PERIPH);

    // Configure each timer for output mode
    HWREG(GREEN_TIMER_BASE + TIMER_O_CFG)   = 0x04;
    HWREG(GREEN_TIMER_BASE + TIMER_O_TAMR)  = 0x0A;
    HWREG(GREEN_TIMER_BASE + TIMER_O_TAILR) = 0xFFFF;

    HWREG(BLUE_TIMER_BASE + TIMER_O_CFG)   = 0x04;
    HWREG(BLUE_TIMER_BASE + TIMER_O_TBMR)  = 0x0A;
    HWREG(BLUE_TIMER_BASE + TIMER_O_TBILR) = 0xFFFF;

    HWREG(RED_TIMER_BASE + TIMER_O_CFG)   = 0x04;
    HWREG(RED_TIMER_BASE + TIMER_O_TBMR)  = 0x0A;
    HWREG(RED_TIMER_BASE + TIMER_O_TBILR) = 0xFFFF;

    // Invert the output signals.
    HWREG(RED_TIMER_BASE + TIMER_O_CTL)   |= 0x4000;
    HWREG(GREEN_TIMER_BASE + TIMER_O_CTL)   |= 0x40;
    HWREG(BLUE_TIMER_BASE + TIMER_O_CTL)   |= 0x4000;

    if(ui32Enable) {
        RGBEnable();
    }
}
/**
 *
 * Initializes the RGB and sets it to shine solid green.
 *
 * \note This function is an extension of the rgb driver of the EK-TM4C123GXL
 * firmware package provided by Texas Instruments.
 *
 * \note The RGB library does use Timer 0B and Timer 1A and Timer 1B
 *
 **/
void twe_RGBInitSetGreen(void) {
	uint32_t RGBcolor[3];
	twe_RGBInitSolid(0); //initialize the RGB for a solid output
	RGBIntensitySet(0.3f); // Set the intensity level (0.0f to 1.0f)
	RGBcolor[RED] =   0x0000;
	RGBcolor[GREEN] = 0xFFFF; // set the color to green
	RGBcolor[BLUE] =  0x0000;
	RGBColorSet(RGBcolor);

	RGBEnable();
}

#endif

/*****************
 * SSI Functions *
 *****************/

/**
  * Enables the TX end of transmission (EOT) feature that allows the TXFF interrupt
  *  to trigger immediately when a transmission completes
  *
  *  \note The EOT interrupt triggers the  TXFF interrupt
  *
  *  \note The TM4C1294NCPDT also has an independent EOT interrupt
  *
  *  \note This enable is in the CR1 register rather than the interrupt mask (IM) register
  **/
void twe_SSIIntEnableEOT(uint32_t ui32Base) {

	// Check the argument.
	ASSERT(_SSIBaseValid(ui32Base));

	// Write to the control register
	HWREG(ui32Base + SSI_O_CR1) |= 0x00000010;
}

#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
/**
 * Initializes the QSSI_COMM port to transmit or receive a data transmission
 *
 * \param RXmode - true  - initialize QSSI_COMM to read as a slave
 * 				   false - initialize QSSI_COMM to write as a master
 **/
void twe_initQSSI(uint32_t SysClkFreq, bool RXmode) {
 	// Enable Peripherals
	MAP_SysCtlPeripheralEnable(twe_QSSI_COMM_PERIPH);
 	MAP_SysCtlPeripheralEnable(twe_QSSI_COMM_CLK_FSS_GPIO_PERIPH);
 	MAP_SysCtlPeripheralEnable(twe_QSSI_COMM_XDAT01_GPIO_PERIPH);
 	MAP_SysCtlPeripheralEnable(twe_QSSI_COMM_XDAT23_GPIO_PERIPH);

 	// Set the pin muxing
 	MAP_GPIOPinConfigure(twe_QSSI_COMM_CLK_PIN_CONFIG);
 	MAP_GPIOPinConfigure(twe_QSSI_COMM_FSS_PIN_CONFIG);
 	MAP_GPIOPinConfigure(twe_QSSI_COMM_DAT0_PIN_CONFIG);
 	MAP_GPIOPinConfigure(twe_QSSI_COMM_DAT1_PIN_CONFIG);
 	MAP_GPIOPinConfigure(twe_QSSI_COMM_DAT2_PIN_CONFIG);
 	MAP_GPIOPinConfigure(twe_QSSI_COMM_DAT3_PIN_CONFIG);

 	MAP_GPIOPinTypeSSI(twe_QSSI_COMM_CLK_FSS_GPIO_BASE, twe_QSSI_COMM_CLK_PIN  | twe_QSSI_COMM_FSS_PIN);
 	MAP_GPIOPinTypeSSI(twe_QSSI_COMM_XDAT01_GPIO_BASE,  twe_QSSI_COMM_DAT0_PIN | twe_QSSI_COMM_DAT1_PIN);
 	MAP_GPIOPinTypeSSI(twe_QSSI_COMM_XDAT23_GPIO_BASE,  twe_QSSI_COMM_DAT2_PIN | twe_QSSI_COMM_DAT3_PIN);

 	// Must be in SPI Mode0 for QSSI (Advanced) mode
 	if(RXmode) {
 	MAP_SSIConfigSetExpClk(twe_QSSI_COMM_BASE, SysClkFreq, SSI_FRF_MOTO_MODE_0,
 	 					   SSI_MODE_SLAVE, twe_QSSI_COMM_BAUD, 8);
 	SSIAdvModeSet(twe_QSSI_COMM_BASE, SSI_ADV_MODE_QUAD_READ);
 	SSIDataPut(twe_QSSI_COMM_BASE,0x00);
 	}
 	else {
 		SSIConfigSetExpClk(twe_QSSI_COMM_BASE, SysClkFreq, SSI_FRF_MOTO_MODE_0,
 		 	 				   SSI_MODE_MASTER, twe_QSSI_COMM_BAUD, 8);
 		SSIAdvModeSet(twe_QSSI_COMM_BASE, SSI_ADV_MODE_QUAD_WRITE);
 	}
 	// Enable SSI
 	MAP_SSIEnable(twe_QSSI_COMM_BASE);
 	//SSIDMAEnable(ADC_SSI_BASE, SSI_DMA_RX); // Enable SSI uDMA
}

#ifdef QSSI_OUT_MODE
/**
 * Initializes the QSSI RX uDMA to transfer 4 bytes from the QSSI RX FIFO to the UART TX FIFO
 **/
void twe_initQSSIuDMArx(void) {
	SSIDMAEnable(twe_QSSI_COMM_BASE, SSI_DMA_RX);

	uDMAChannelAssign(twe_QSSI_COMM_RX_UDMA_CHANNEL_ASSIGN);

	// Place the uDMA channel attributes in a known state. These should already be disabled by default.
	uDMAChannelAttributeDisable(twe_QSSI_COMM_RX_UDMA_CHANNEL,
	                            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
	                            (UDMA_ATTR_HIGH_PRIORITY |
	                            UDMA_ATTR_REQMASK));
	// Configure the control parameters for the SSI2 RX channel.  The channel
	// will be used to transfer the ADC measurements to memory.
	uDMAChannelControlSet(twe_QSSI_COMM_RX_UDMA_CHANNEL | UDMA_PRI_SELECT,
						  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE |
						  UDMA_ARB_4);
	// Set up the transfer parameters for the SSI2 Rx channel.  This will
	// configure the transfer buffers and the transfer size.
	uDMAChannelTransferSet(twe_QSSI_COMM_RX_UDMA_CHANNEL | UDMA_PRI_SELECT,
						   UDMA_MODE_BASIC,
						   (void *)(twe_QSSI_COMM_BASE + SSI_O_DR), (void *)(TWE_UART_COMM_BASE + UART_O_DR),
						   4);
	uDMAChannelAttributeEnable(twe_QSSI_COMM_RX_UDMA_CHANNEL, UDMA_ATTR_HIGH_PRIORITY);
	//SSIIntEnable(ADC_SSI_BASE, SSI_DMARX);
	//IntEnable(ADC_SSI_INT);

	uDMAChannelEnable(twe_QSSI_COMM_RX_UDMA_CHANNEL);
}

/**
 * Initializes the QSSI RTX uDMA to transfer 4 bytes from memory to the QSSI TX FIFO
 **/
void twe_initQSSIuDMAtx(void) {
	SSIDMAEnable(twe_QSSI_COMM_BASE, SSI_DMA_TX);

	uDMAChannelAssign(twe_QSSI_COMM_TX_UDMA_CHANNEL_ASSIGN);

	// Place the uDMA channel attributes in a known state. These should already be disabled by default.
	uDMAChannelAttributeDisable(twe_QSSI_COMM_TX_UDMA_CHANNEL,
	                            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
	                            (UDMA_ATTR_HIGH_PRIORITY |
	                            UDMA_ATTR_REQMASK));
	// Configure the control parameters for the SSI2 RX channel.  The channel
	// will be used to transfer the ADC measurements to memory.
	uDMAChannelControlSet(twe_QSSI_COMM_TX_UDMA_CHANNEL | UDMA_PRI_SELECT,
						  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
						  UDMA_ARB_4);
	// Set up the transfer parameters for the SSI2 Rx channel.  This will
	// configure the transfer buffers and the transfer size.
	uDMAChannelTransferSet(twe_QSSI_COMM_TX_UDMA_CHANNEL | UDMA_PRI_SELECT,
						   UDMA_MODE_BASIC,
						   (void *)(twe_g_QSSItxBuffer), (void *)(TWE_UART_COMM_BASE + UART_O_DR),
						   4);
	uDMAChannelAttributeEnable(twe_QSSI_COMM_TX_UDMA_CHANNEL, UDMA_ATTR_HIGH_PRIORITY);
	//SSIIntEnable(ADC_SSI_BASE, SSI_DMARX);
	//IntEnable(ADC_SSI_INT);

	uDMAChannelEnable(twe_QSSI_COMM_TX_UDMA_CHANNEL);
}
#endif
#endif

/*****************
 * I2C Functions *
 *****************/

/**
 * Verifies that a transmission from the master has completed successfully
 *
 * \param ui32base Base address of the I2C peripheral 
 *
 * \param burst Set to true if burst mode was used for the transmission
 *
 * \param receive Set to true if a receive transmission is to be verified
 * Set to false if a send transmission is to be verified
 *
 * \note Waits until a transmission is complete and then checks that no errors have occurred
 *
 * \note If an error occurs the I2C transmission is stopped, the error LED is lit and the
 * program enters an infinite loop to hold the state.
 **/
void twe_I2CMasterVerify(uint32_t ui32base, bool burst, bool receive) {
	while(I2CMasterBusy(ui32Base)) {} // Wait until the transfer is complete
	//uint32_t errorStatus = I2CMasterErr(ui32Base);
	if(I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE) {

		// An error has occured
		if(burst && !receive) {
			I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
		}
		else if(burst && !receive) {
			I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
		}

		while(1) {} // Capture state
	}
}


/*******************
 * Timer Functions *
 *******************/

/**
 * Initializes Timer for continuous operation
 *	Can be used as a multipurpose timer
 *	e.g. To test out the Program without the ADC functionality
 * \param enForcer - enables the LDAC_FORCER timer
 * \param enQuad - enables the LDAC_QUAD timer
 * \param pulseWidth - Number of clock cycles to pulse the LDAC signal low.
 *
 * \note Not available on the GTBE-TM4C123GXL
 **/
/*
void twe_initTimer(uint32_t Period) {
	// LDAC_FORCER Timer
	//MAP_SysCtlPeripheralEnable(twe_TIMER_GPIO_PERIPH);

	//MAP_GPIOPinTypeTimer(twe_TIMER_GPIO_BASE, twe_TIMER_PIN);
	//MAP_GPIOPinConfigure(twe_TIMERPIN_CONFIG);
	MAP_SysCtlPeripheralEnable(twe_TIMER_PERIPH);
	MAP_TimerConfigure(twe_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT | TIMER_CFG_B_ACT_CLRSETTO);
	MAP_TimerLoadSet(twe_TIMER_BASE, twe_TIMER, Period);

}
*/

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
