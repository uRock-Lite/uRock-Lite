#include "param.h"



int param_change(param* p, int percentage){
	p->now_p = percentage;
	p->now = p->min + (p->max - p->min) * percentage / 100;
}
