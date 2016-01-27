#include "param.h"
#include "effect.h"

/**********Effect list**********/
//Effect_maxnum is defined in effect.h
#define Effect_none 0
#include "Distortion0.h"
#define Effect_Distortion0 1
#include "Subsystem.h" //Ringmod
#define Effect_Subsystem 2
//#include "Delay0.h" //[FIXME]Test delay effect
//#define Effect_Subsystem 3
/*******************************/

int effect_init(effect* neweffect, int id){
	param* param;
	neweffect->id = id;
	switch(id){
		case Effect_none:
			neweffect->name = "None";
			neweffect->param_number = 0;
			break;
		case Effect_Distortion0:
			neweffect->name = "Distortion0";
			neweffect->func = Distortion0_step;
			neweffect->param_number = 2;

			param = neweffect->params[0];
			param->name = "tone";
			param->unit = " ";
			param->max = 1;
			param->min = 0;

			param = neweffect->params[1];
			param->name = "gain";
			param->unit = " ";
			param->max = 100;
			param->min = 0;
			break;
		case Effect_Subsystem:
			neweffect->name = "Subsystem";
			neweffect->func = Subsystem_step;
			neweffect->param_number = 3;

			param = neweffect->params[0];
			param->name = "freq";
			param->unit = "hz";
			param->max = 4000;
			param->min = 0;

			param = neweffect->params[1];
			param->name = "depth";
			param->unit = " ";
			param->max = 1;
			param->min = 0;

			param = neweffect->params[2];
			param->name = "fs";
			param->unit = "hz";
			param->max = 10000;
			param->min = 1000;
			break;
		default:
			break;
	}
	return 1;
}
