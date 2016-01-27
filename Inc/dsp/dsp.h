#ifndef DSP
#define DSP
typedef struct Effect effect;/*[FIXME]*/
typedef struct DigitalSignalProcessor dsp;
struct DigitalSignalProcessor{
	char* name;
	char* aboutme;
	int stage_number;
	int sample_number;
	double* buffer;
	effect** stages;
};
/* #4 */
int dsp_init(dsp* dsp, char* name, char* aboutme, int stage_number, int sample_number);
int dsp_execute(dsp* dsp, uint32_t* adcBuffer, uint32_t* dacBuffer);
int dsp_setStage(dsp* dsp, int stage, int id, int* newParams);
int dsp_close(dsp* dsp);

/* private */
void Normalize(uint32_t* before, double* after, int sample_number);
void Denormalize(double* before, uint32_t* after, int sample_number);
#endif
