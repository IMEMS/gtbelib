/******************************************************************************
 *
 * synth_si4351.h - Header for the Silicon Labs si5351 Frequency synthesizer
 *  Interfaces the TI tiva C Launchpad with the Silicon Labs si5351 frequency
 *  synthesizer
 *
 *  \note Used to generate the ADC CLK on the GTBE-TM4C1294XL
 *
 *	\note Predefined register settings are defined in synth_si4351_reg_map.h
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  rev1 May 2014
 *
 *  Originally written for the MRIG gyroscope project
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/

#ifndef SI5351_H_
#define SI5351_H_

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

#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL
	// Serial Connection: I2C0
	#define SYNTH_I2C_PERIPH 					SYSCTL_PERIPH_I2C0
	#define SYNTH_I2C_BASE						I2C0_BASE
	#define SYNTH_GPIO_PERIPH					SYSCTL_PERIPH_GPIOB
	#define SYNTH_GPIO_BASE						GPIO_PORTB_BASE
	#define SYNTH_I2C_SCL_PIN_CONFIG			GPIO_PB2_I2C0SCL
	#define SYNTH_I2C_SDA_PIN_CONFIG			GPIO_PB3_I2C0SDA
	#define SYNTH_SCL_PIN						GPIO_PIN_2
	#define SYNTH_SDA_PIN						GPIO_PIN_3
	#define SYNTH_I2C_SLAVE_ADDRESS				0x60

	// ADC_RECLK_ENL Control
	#define SYNTH_ADC_RECLK_ENL_GPIO_PERIPH		SYSCTL_PERIPH_GPION
	#define SYNTH_ADC_RECLK_ENL_GPIO_BASE		GPIO_PORTN_BASE
	#define SYNTH_ADC_RECLK_ENL_PIN				GPIO_PIN_2
#endif

/***************************
 * Register Setting Values *
 ***************************/
// Interrupt Status Mask (reg 2)
#define SYNTH_REG_VAL_EN_ALL_INTERRUPT_MASKS 	0xF0

// Output Enable Control (reg 3)
#define SYNTH_REG_VAL_OUTPUT_DISABLE			0xFF

/****************
 * Register Map *
 ****************/
// Register Map is defined in Silicon Labs app note
//  AN619: Manually Generating an Si5351 Register Map

// Device Status
#define SYNTH_STATUS_REG 				0
#define SYNTH_STATUS_SYS_INIT			0x80
#define SYNTH_STATUS_LOL_B				0x40 // Loss of lock PLL B
#define SYNTH_STATUS_LOL_A				0x20 // Loss of lock PLL A
#define SYNTH_STATUS_LOS				0x10 // CLKIN Loss Of Signal (Si5351C Only)
// Interrupt Status Sticky
#define SYNTH_INT_STATUS_REG 			1
#define SYNTH_INT_STATUS_SYS_INIT_STKY	0x80
#define SYNTH_INT_STATUS_LOL_B_STKY		0x40
#define SYNTH_INT_STATUS_LOL_A_STKY		0x20
#define SYNTH_INT_STATUS_LOS_STKY		0x10
// Interrupt Status Mask
#define SYNTH_INT_STATUS_MASK_REG 			2
#define SYNTH_INT_STATUS_MASK_SYS_INIT_MASK	0x80
#define SYNTH_INT_STATUS_MASK_LOL_B_MASK	0x40
#define SYNTH_INT_STATUS_MASK_LOL_A_MASK	0x20
#define SYNTH_INT_STATUS_MASK_LOS_MASK		0x10
// Output Enable Control
#define SYNTH_OUT_EN_CTL_REG			3
#define SYNTH_OUT_EN_CTL_CLK7_OEB		0x80
#define SYNTH_OUT_EN_CTL_CLK6_OEB		0x40
#define SYNTH_OUT_EN_CTL_CLK5_OEB		0x20
#define SYNTH_OUT_EN_CTL_CLK4_OEB		0x10
#define SYNTH_OUT_EN_CTL_CLK3_OEB		0x08
#define SYNTH_OUT_EN_CTL_CLK2_OEB		0x04
#define SYNTH_OUT_EN_CTL_CLK1_OEB		0x02
#define SYNTH_OUT_EN_CTL_CLK0_OEB		0x01
// OEB Pin Enable Control Mask
#define SYNTH_OUT_EN_PIN_MASK_REG			9
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK7		0x80
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK6		0x40
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK5		0x20
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK4		0x10
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK3		0x08
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK2		0x04
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK1		0x02
#define SYNTH_OUT_EN_PIN_MASK_OEB_MASK0		0x01
// PLL Input Source
#define SYNTH_INPUT_SRC_REG				15
#define SYNTH_INPUT_PIN_CLKIN_DIV1		0x80
#define SYNTH_INPUT_PIN_CLKIN_DIV0		0x40
#define SYNTH_INPUT_PIN_PLLB_SRC		0x08
#define SYNTH_INPUT_PIN_PLLA_SRC		0x04
// CLK0 Control
#define SYNTH_CTL_CLK0_REG			16
#define SYNTH_CTL_CLK0_PDN			0x80
#define SYNTH_CTL_CLK0_MS0_INT		0x40
#define SYNTH_CTL_CLK0_MS0_SRC		0x20
#define SYNTH_CTL_CLK0_INV			0x10
#define SYNTH_CTL_CLK0_SRC1			0x08
#define SYNTH_CTL_CLK0_SRC0			0x04
#define SYNTH_CTL_CLK0_IDRV1		0x02
#define SYNTH_CTL_CLK0_IDRV0		0x01
// CLK1 Control
#define SYNTH_CTL_CLK1_REG			17
#define SYNTH_CTL_CLK1_PDN			0x80
#define SYNTH_CTL_CLK1_MS1_INT		0x40
#define SYNTH_CTL_CLK1_MS1_SRC		0x20
#define SYNTH_CTL_CLK1_INV			0x10
#define SYNTH_CTL_CLK1_SRC1			0x08
#define SYNTH_CTL_CLK1_SRC0			0x04
#define SYNTH_CTL_CLK1_IDRV1		0x02
#define SYNTH_CTL_CLK1_IDRV0		0x01
// CLK2 Control
#define SYNTH_CTL_CLK2_REG			18
#define SYNTH_CTL_CLK2_PDN			0x80
#define SYNTH_CTL_CLK2_MS2_INT		0x40
#define SYNTH_CTL_CLK2_MS2_SRC		0x20
#define SYNTH_CTL_CLK2_INV			0x10
#define SYNTH_CTL_CLK2_SRC1			0x08
#define SYNTH_CTL_CLK2_SRC0			0x04
#define SYNTH_CTL_CLK2_IDRV1		0x02
#define SYNTH_CTL_CLK2_IDRV0		0x01
// CLK3 Control
#define SYNTH_CTL_CLK3_REG			19
#define SYNTH_CTL_CLK3_PDN			0x80
#define SYNTH_CTL_CLK3_MS3_INT		0x40
#define SYNTH_CTL_CLK3_MS3_SRC		0x20
#define SYNTH_CTL_CLK3_INV			0x10
#define SYNTH_CTL_CLK3_SRC1			0x08
#define SYNTH_CTL_CLK3_SRC0			0x04
#define SYNTH_CTL_CLK3_IDRV1		0x02
#define SYNTH_CTL_CLK3_IDRV0		0x01
// CLK4 Control
#define SYNTH_CTL_CLK4_REG			20
#define SYNTH_CTL_CLK4_PDN			0x80
#define SYNTH_CTL_CLK4_MS4_INT		0x40
#define SYNTH_CTL_CLK4_MS4_SRC		0x20
#define SYNTH_CTL_CLK4_INV			0x10
#define SYNTH_CTL_CLK4_SRC1			0x08
#define SYNTH_CTL_CLK4_SRC0			0x04
#define SYNTH_CTL_CLK4_IDRV1		0x02
#define SYNTH_CTL_CLK4_IDRV0		0x01
// CLK5 Control
#define SYNTH_CTL_CLK5_REG			21
#define SYNTH_CTL_CLK5_PDN			0x80
#define SYNTH_CTL_CLK5_MS5_INT		0x40
#define SYNTH_CTL_CLK5_MS5_SRC		0x20
#define SYNTH_CTL_CLK5_INV			0x10
#define SYNTH_CTL_CLK5_SRC1			0x08
#define SYNTH_CTL_CLK5_SRC0			0x04
#define SYNTH_CTL_CLK5_IDRV1		0x02
#define SYNTH_CTL_CLK5_IDRV0		0x01
// CLK6 Control
#define SYNTH_CTL_CLK6_REG			22
#define SYNTH_CTL_CLK6_PDN			0x80
#define SYNTH_CTL_CLK6_MS6_INT		0x40
#define SYNTH_CTL_CLK6_MS6_SRC		0x20
#define SYNTH_CTL_CLK6_INV			0x10
#define SYNTH_CTL_CLK6_SRC1			0x08
#define SYNTH_CTL_CLK6_SRC0			0x04
#define SYNTH_CTL_CLK6_IDRV1		0x02
#define SYNTH_CTL_CLK6_IDRV0		0x01
// CLK7 Control
#define SYNTH_CTL_CLK7_REG			23
#define SYNTH_CTL_CLK7_PDN			0x80
#define SYNTH_CTL_CLK7_MS7_INT		0x40
#define SYNTH_CTL_CLK7_MS7_SRC		0x20
#define SYNTH_CTL_CLK7_INV			0x10
#define SYNTH_CTL_CLK7_SRC1			0x08
#define SYNTH_CTL_CLK7_SRC0			0x04
#define SYNTH_CTL_CLK7_IDRV1		0x02
#define SYNTH_CTL_CLK7_IDRV0		0x01

/* Some registers remain undefined (regs. 24 - 92 and 149 - 170) */

// PLL Reset
#define SYNTH_RST_PLL_REG			177
#define SYNTH_RST_PLL_PLLB_RST		0x80
#define SYNTH_RST_PLL_PLLA_RST		0x20
// Crystal Internal Load Capacitance
#define SYNTH_XTAL_CL_REG			183
#define SYNTH_XTAL_CL_6PF			0x52
#define SYNTH_XTAL_CL_8PF			0x92
#define SYNTH_XTAL_CL_10PF			0xD2
// Fanout Enable
#define SYNTH_FANOUT_EN_REG			187
#define SYNTH_FANOUT_EN_CLKIN		0x80
#define SYNTH_FANOUT_EN_XO			0x40
#define SYNTH_FANOUT_EN_MS			0x10

/********************
 * Synth Parameters *
 ********************/

/**************
 * Prototypes *
 **************/
// Initialization Functions
extern void synth_init(bool synthEN, uint8_t xtal_cl, uint8_t* regConfig, uint8_t outputEn);
extern void synth_initConfig(uint8_t* regConfig, uint8_t outputEn);
extern void synth_initADCreclkENL(bool bEnable);
// Command Functions
extern void synth_outputDisableAll(void);
extern void synth_outputEnable(uint8_t outputEn);
extern void synth_powerDownOutputDrivers(void);
extern void synth_setInterruptMasks(uint8_t interruptMasks);
extern uint32_t synth_readINTstatus(void);
extern void synth_clearINT(uint8_t interruptCLR);
extern void synth_writeRegConfig(uint8_t* regConfig);
extern void synth_PLLreset(uint8_t PLLrst);
extern uint32_t synth_readDeviceStatus(void);
extern void synth_setXTALcapacitance(uint8_t xtal_cl);
extern void synth_fanoutEnable(uint8_t fanoutEN);
// I2C Functions
extern void synth_initI2C(bool fast);
extern void synth_writeRegI2C(uint8_t regNum, uint8_t regValue);
extern void synth_writeConsecutiveRegsI2C(uint8_t regNumStart, uint8_t regCNT, uint8_t* regValues);
extern uint32_t synth_readRegI2C(uint8_t regNum);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* SI5351_H_ */
