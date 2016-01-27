#ifndef PARAMETER
#define PARAMETER

typedef struct Parameter param;
struct Parameter{
	char* name;
	char* unit;
	double max;
	double min;
	double now;
	int now_p;
};
/* #1 */
int param_change(param* p, int percentage);
#endif
