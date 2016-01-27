#include <stdint.h>
#include "param.h"
#include "effect.h"
#include "dsp.h"

#define HW_Bias (0.7)      //volt
#define HW_Vref (2.2)      //volt
#define HW_Base (4096 - 1) //12-bit adc/dac module

int dsp_init(dsp* newdsp, char* name, char* aboutme, int stage_number, int sample_number){
	int i,j;
	newdsp->name = name;
	newdsp->aboutme = aboutme;
	newdsp->stage_number = stage_number;
	newdsp->sample_number = sample_number;
	newdsp->buffer = pvPortMalloc(sizeof(double)*sample_number);
	/* [HARMFULL]Be aware of memory allocation for double pointer !! */
	newdsp->stages = pvPortMalloc(sizeof(effect*)*stage_number);
	for(int i=0;i<stage_number;i++){
		/* [HARMFULL]Allocation directly!! Dont do this"cursor = newdsp->stages[i];" 01240732*/
		newdsp->stages[i] = pvPortMalloc(sizeof(effect));
		newdsp->stages[i]->params = pvPortMalloc(sizeof(param*)*6/*[FIXME]HARDCODE*/);
		for(int j=0;j<6/*[FIXME]HARDCODE*/;j++){
			newdsp->stages[i]->params[j] = pvPortMalloc(sizeof(param));
		}
	}
	return 1;
}

int dsp_setStage(dsp* dsp, int stage, int id, int* newParams){
	if(stage >= dsp->stage_number)
		return 0;

	/* set stage by effect id*/
	effect* cursor = dsp->stages[stage];
	if(cursor->id != id){
		effect_init(cursor, id);
	}

	/* set parameters */
	int i, num = cursor->param_number;
	for(i = 0; i < num; i++){
		param_change(cursor->params[i], newParams[i]);
	}
	return 1;
}

int dsp_execute(dsp* dsp, uint32_t* adcBuffer, uint32_t* dacBuffer){
	Normalize(adcBuffer, dsp->buffer, dsp->sample_number);
	int i;
	effect* cursor;
	for(i = 0; i < dsp->stage_number; i++){
		cursor = dsp->stages[i];
		if((cursor->id > 0) && (cursor->id < Effect_maxnum))/*[FIXME]*/
			if(cursor->id != 0)
			/*Magic c*/cursor->func(dsp->buffer,
					dsp->sample_number,
					cursor->params);
	}
	Denormalize(dsp->buffer,dacBuffer,dsp->sample_number);

	return 1;
}

int dsp_close(dsp* dsp){
	return 1;
}

void Normalize(uint32_t* before, double* after, int sample_number){
	int i;
	for(i = 0; i < sample_number; i++){
		after[i] = before[i];
		after[i] = after[i] * HW_Vref / HW_Base - HW_Bias;
	}
}

void Denormalize(double* before, uint32_t* after, int sample_number){
	int i;
	for(i = 0; i < sample_number; i++){
		before[i] += HW_Bias;
		before[i] = before[i] * HW_Base / HW_Vref;
		after[i] = before[i];
	}
}
