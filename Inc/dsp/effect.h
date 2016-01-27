#ifndef EFFECT
#define EFFECT

typedef struct Effect effect;
struct Effect{
	char* name;
	int id;
	int param_number;
	param** params;
	void (*func)(double*,int,param*);
};
/* #1 */
int effect_init(effect* neweffect, int type);

/*[FIXME]This DEFINE SHOULD NOT BE HERE*/
#define Effect_maxnum 3
#endif
