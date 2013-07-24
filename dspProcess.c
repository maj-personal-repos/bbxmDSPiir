#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dspProcess.h"
#include "buffer.h"

/*
 * Discrete-Time IIR Filter (real)
 * -------------------------------
 * Filter Structure    : Direct-Form II, Second-Order Sections
 * Number of Sections  : 1
 * Stable              : Yes
 * Linear Phase        : No
 * Arithmetic          : fixed
 * Numerator           : s16,13 -> [-4 4) Q2.13
 * Denominator         : s16,14 -> [-2 2) Q1.14
 * Scale Values        : s16,19 -> [-6.250000e-02 6.250000e-02) Q0.19
 * Input               : s16,15 -> [-1 1) Q0.15
 * Section Input       : s16,13 -> [-4 4) Q2.13
 * Section Output      : s16,10 -> [-32 32) Q5.10
 * Output              : s16,10 -> [-32 32) Q5.10
 * State               : s16,15 -> [-1 1) Q0.15
 * Numerator Prod      : s32,28 -> [-8 8) Q3.28
 * Denominator Prod    : s32,29 -> [-4 4) Q2.29
 * Numerator Accum     : s40,28 -> [-2048 2048) Q11.28
 * Denominator Accum   : s40,29 -> [-1024 1024) Q10.29
 * Round Mode          : convergent
 * Overflow Mode       : wrap
 * Cast Before Sum     : true
 */

#define STATE_SHIFT 13
#define SEC_SHIFT 13
#define OUT_SCALE 5
#define MWSPT_NSEC 5

const int NL[MWSPT_NSEC] = { 1,3,1,3,1 };
const short NUM[MWSPT_NSEC][3] = {
  {
      537,      0,      0 
  },
  {
     8192,  16384,   8192 
  },
  {
      432,      0,      0 
  },
  {
     8192,  16384,   8192 
  },
  {
     8192,      0,      0 
  }
};
const int DL[MWSPT_NSEC] = { 1,3,1,3,1 };
const short DEN[MWSPT_NSEC][3] = {
  {
     8192,      0,      0 
  },
  {
     8192, -11434,   5388 
  },
  {
     8192,      0,      0 
  },
  {
     8192,  -9206,   2742 
  },
  {
     8192,      0,      0 
  }
};


#define ORDER (MWSPT_NSEC-1)/2

buffer *wnL[ORDER];
buffer *wnR[ORDER];


// Implements filtering for a second order section
int iir_sos(short xn, int section){
	int wn0 = 0;
	int wn1 = readn(wnL[section],1);
	int wn2 = readn(wnL[section],2);
	int yn = 0;
	int scaleIndex = 2*section;
	int sectionIndex = scaleIndex + 1;
	wn0 = (NUM[scaleIndex][0]*xn - DEN[sectionIndex][1]*wn1 - DEN[sectionIndex][2]*wn2);
	wn0 = (wn0 >> STATE_SHIFT) & 0xffff;
	yn = (DEN[sectionIndex][0]*wn0 + DEN[sectionIndex][1]*wn1 + DEN[sectionIndex][2]*wn2);
	yn = (yn >> SEC_SHIFT)& 0xffff;
	push(wnL[section],(short)wn0);
	return yn;
}

// Fourth order IIR filter
short iirL(short xn){
	int section;
	int input = xn;
	int yn;
	for(section = 0; section < ORDER; section ++){
		yn = iir_sos((short)input,section);
		input = yn;
	}
	return (short)(yn << OUT_SCALE);
}

// core dsp block processing
int dspBlockProcess(short *outputBuffer, short *inputBuffer, int samples, int * filter_on, double * volume){
	int i;
	if(*filter_on == 0) { // no filter
		memcpy((char *)outputBuffer, (char *)inputBuffer, 2*samples);
	}
	else if(*filter_on == 1) { // filter
		for (i=0; i < samples; i+=2){
			outputBuffer[i] = (short)(iirL(inputBuffer[i]));
			outputBuffer[i+1] = inputBuffer[i+1];
		}
	}
	return DSP_PROCESS_SUCCESS;
}

void initIIRBuffers(){
	int i ;
	for(i=0; i<ORDER; i++) {
		wnL[i] = malloc(sizeof(buffer));
		initBuffer(wnL[i]);
		wnR[i] = malloc(sizeof(buffer));
		initBuffer(wnR[i]);
	}
}

void destroyIIRBuffers(){
    int i ;
	for(i=0; i<ORDER; i++) {
		destroyBuffer(wnL[i]);
		free(wnL[i]);
		destroyBuffer(wnR[i]);
		free(wnR[i]);
	}
}
