/******************************************************************************
 * temp_ADT7310.h - headers for the ADI ADT7310 SPI digital temperature
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

#ifndef TEMP_ADT7310_H_
#define TEMP_ADT7310_H_

// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
#ifdef __cplusplus
extern "C"
{
#endif

#define MEM_BUFFER_SIZE 1 // Size of the reading buffer size


/**************
 * Prototypes *
 **************/
// Sensor
extern void  temp_configSensorContinuousRead(uint32_t *statusreg);
extern void  temp_resetSPI(void);
extern float temp_degC13bitReading(uint32_t reading);
extern float temp_degF13bitReading(uint32_t reading);
// uDMA
extern void temp_inituDMA(void);
extern void temp_uDMAErrorHandler(void);
// SSI
extern void temp_initSSI0uDMA(void);
// Timer
extern void temp_initTimer2A(void);
extern void temp_Timer2IntHandler(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* TEMP_ADT7310_H_ */

