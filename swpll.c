/******************************************************************************
 *
 * swpll.c - drivers for a software defined phase locked loop
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

#include <stdint.h>

// GTBE Lib
#include "swpll.h"

/*****************
 * DSP Functions *
 *****************/

/* FIR Filters */

/**
 * Initializes an FIR filter
 **/
void initFIR(firInst* firFilt, float* filtCoeff) {
	firFilt->coeff = filtCoeff;
	uint32_t i;
	for(i = 0; i < NUM_TAPS; i++) {
		firFilt->dataBuffer[i] = 0;
		//firFilt->pData[i] = firFilt->dataBuffer + 4u*i;
		firFilt->pData[i] = &firFilt->dataBuffer[i];
	}
	firFilt->dataHead = firFilt->dataBuffer;
	firFilt->dataBufferStart = firFilt->dataBuffer;
	firFilt->dataBufferEnd = &firFilt->dataBuffer[NUM_TAPS-1];
}

/**
 * Applies an input to a running FIR filter, filt.  The input is added to a circular buffer
 *
 * \parameter filt - an initialized FIR filter
 * \parameter input - A new data point to add to the filter's data buffer
 **/
float FIRfilter(firInst* filt, float input) {
	float acc = 0;
	// Update filter data buffer with new data point
	*(filt->dataHead) = input;
	filt->dataHead++;
	if(filt->dataHead > filt->dataBufferEnd) {
		filt->dataHead = filt->dataBufferStart;
	}
	// Apply Filter
	uint32_t i;
	for(i=0; i < NUM_TAPS; i++) {
		acc += (*(filt->pData[i])) * (filt->coeff[i]);
		filt->pData[i]++;
		if(filt->pData[i] > filt->dataBufferEnd) {
			filt->pData[i] = filt->dataBufferStart;
		}
	}
	return acc;
}

/* atan2 Approximations */

/*
			if(I_err[0] >= 0) { // Quadrant 1,2,7,8
				if(Q_err[0] >= 0) { // Quadrant 1,2
					if(I_err[0]>Q_err[0]) { // Quadrant 1
						g_err.data.fn[0] = (Q_err[0]/I_err[0]) * (1.0584-0.273*(Q_err[0]/I_err[0]));
					}
					else { // Quadrant 2
						x = I_err[0]/Q_err[0];
						//g_err.data.fn[0] = M_PI_DIV2 - (I_err[0]/Q_err[0]) * (1.0584-0.273*(I_err[0]/Q_err[0]));
						//g_err.data.fn[0] = M_PI_DIV2 - (I_err[0]/Q_err[0]) * (1.0584-0.273*(I_err[0]/Q_err[0]));
						g_err.data.fn[0] = M_PI_DIV2 - x * (1.0584-0.273*x);
					}
				}
				else { // Quadrant 7,8 (Q < 0)
					if((-Q_err[0]) > I_err[0]) { // Quadrant 7
						g_err.data.fn[0] = M_N_PI_DIV2 - (I_err[0]/Q_err[0]) * (1.0584+0.273*(I_err[0]/Q_err[0]));
					}
					else { // Quadrant 8
						g_err.data.fn[0] = (Q_err[0]/I_err[0]) * (1.0584+0.273*(Q_err[0]/I_err[0]));
					}
				}
			}
			else  { // Quadrant 3,4,5,6 (I < 0)
				if(Q_err[0] >= 0) { // Quadrant 3,4
					if(Q_err[0] > (-I_err[0])) { // Quadrant 3
						g_err.data.fn[0] = M_PI_DIV2 - (I_err[0]/Q_err[0]) * (1.0584+0.273*(I_err[0]/Q_err[0]));
					}
					else { // Quadrant 4
						g_err.data.fn[0] = M_PI + (Q_err[0]/I_err[0]) * (1.0584+0.273*(Q_err[0]/I_err[0]));
					}
				}
				else { // Quadrant 5,6 (Q < 0)
					if(I_err[0] <= Q_err[0]) { // Quadrant 5
						g_err.data.fn[0] = M_N_PI + (Q_err[0]/I_err[0]) * (1.0584-0.273*(Q_err[0]/I_err[0]));
					}
					else { // Quadrant 6
						g_err.data.fn[0] = M_N_PI_DIV2 - (I_err[0]/Q_err[0]) * (1.0584-0.273*(I_err[0]/Q_err[0]));
					}
				}
			}
			*/

