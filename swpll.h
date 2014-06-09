/******************************************************************************
 *
 * swpll.h - headers for a software defined phase locked loop
 *  Used with the Georgia Tech Back End (GTBE)
 *   GTBE-EK-TM4C123GXL
 *	 GTBE-EK-TM4C1294XL
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  February 2014
 *
 *  Originally written for the MRIG gyroscope project
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/

/*************
 * Constants *
 *************/

#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif
#ifndef M_N_PI
#define M_N_PI                 -3.14159265358979323846
#endif
#ifndef M_PI_DIV2
#define M_PI_DIV2 				1.570796326794897
#endif
#ifndef M_N_PI_DIV2
#define M_N_PI_DIV2 		   -1.570796326794897
#endif
#ifndef M_PI_DIV4
//#define M_PI_DIV4 				0.785398163397448
#define M_PI_DIV4 				0.78539
#endif

/**************
 * Parameters *
 **************/

// PLL Loop Parameters
#define PHASE_OFFSET 		0.0
#define F_C		     		10000 // Center Frequency of the NCO
#define FS			 		78124.8648649 // Sampling Frequency of the input
#define FREQ_RATIO_FS_FC	7.81248648649
#define PHASE_INC	 		1
// Test Input data
#define Aout		 100000  //output amplitude
#define K_GAIN 		-0.01   // PLL PD gain
#define F_S         78124.8648649   // Sampling Frequency
//FIR Filter Data
//#define NUM_TAPS     16	// Number of Taps in the FIR filter
#define NUM_TAPS     8	// Number of Taps in the FIR filter
#define BLOCK_SIZE	 1		// Claculate a single data point each time

/************
 * TypeDefs *
 ************/

typedef struct {
	float* coeff;
	float  dataBuffer[NUM_TAPS];
	float* pData[NUM_TAPS];
	float* dataHead;
	float* dataBufferStart;
	float* dataBufferEnd;
} firInst;

/**************
 * Prototypes *
 **************/
extern void initFIR(firInst* firFilt, float* filtCoeff);
extern float FIRfilter(firInst* filt, float input);
