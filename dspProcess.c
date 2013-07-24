#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dspProcess.h"
#include "buffer.h"

#define SHIFT 13
#define MWSPT_NSEC 9
const int NL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1 };
const short NUM[MWSPT_NSEC][3] = {
  {
      586,      0,      0 
  },
  {
     8192,  16384,   8192 
  },
  {
      498,      0,      0 
  },
  {
     8192,  16384,   8192 
  },
  {
      447,      0,      0 
  },
  {
     8192,  16384,   8192 
  },
  {
      423,      0,      0 
  },
  {
     8192,  16384,   8192 
  },
  {
     8192,      0,      0 
  }
};
const int DL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1 };
const short DEN[MWSPT_NSEC][3] = {
  {
     8192,      0,      0 
  },
  {
     8192, -12481,   6632 
  },
  {
     8192,      0,      0 
  },
  {
     8192, -10613,   4414 
  },
  {
     8192,      0,      0 
  },
  {
     8192,  -9523,   3118 
  },
  {
     8192,      0,      0 
  },
  {
     8192,  -9021,   2522 
  },
  {
     8192,      0,      0 
  }
};


#define SECTIONS (MWSPT_NSEC-1)/2

buffer *wnL[SECTIONS];
buffer *wnR[SECTIONS];


// Implements filtering for a second order section
int iir_sos(short xn, int section){
	int wn0 = 0;
	short wn1 = readn(wnL[section],0);
	short wn2 = readn(wnL[section],1);
	int yn = 0;
	int scaleIndex = 2*section;
	int sectionIndex = scaleIndex + 1;
	wn0 = ((NUM[scaleIndex][0]*xn)) - ((DEN[sectionIndex][1]*wn1)) - ((DEN[sectionIndex][2]*wn2));
	wn0 = wn0 >> SHIFT;
	yn = ((NUM[sectionIndex][0]*wn0)) + ((NUM[sectionIndex][1]*wn1)) + ((NUM[sectionIndex][2]*wn2));
	push(wnL[section],(short)wn0);
	return yn>>SHIFT;
}

// Fourth order IIR filter
short iirL(short xn){
	int section;
	int input = xn;
	int yn;
	for(section = 0; section < SECTIONS; section ++){
		yn = iir_sos((short)input,section);
		input = yn;
	}
	return (short)yn;
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
			outputBuffer[i+1] = (short)(iirL(inputBuffer[i]));
		}
	}
	return DSP_PROCESS_SUCCESS;
}

void initIIRBuffers(){
	int i ;
	for(i=0; i<SECTIONS; i++) {
		wnL[i] = malloc(sizeof(buffer));
		initBuffer(wnL[i]);
		wnR[i] = malloc(sizeof(buffer));
		initBuffer(wnR[i]);
	}
}

void destroyIIRBuffers(){
    int i ;
	for(i=0; i<SECTIONS; i++) {
		destroyBuffer(wnL[i]);
		free(wnL[i]);
		destroyBuffer(wnR[i]);
		free(wnR[i]);
	}
}
