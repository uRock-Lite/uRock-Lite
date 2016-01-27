/*
 * File: Subsystem.c
 *
 * Code generated for Simulink model 'Subsystem'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Sun Jan 24 04:06:59 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Subsystem.h"
#include "Subsystem_private.h"

/* Block signals (auto storage) */
B_Subsystem_T Subsystem_B;

/* External inputs (root inport signals with auto storage) */
ExtU_Subsystem_T Subsystem_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_Subsystem_T Subsystem_Y;

/* Real-time model */
RT_MODEL_Subsystem_T Subsystem_M_;
RT_MODEL_Subsystem_T *const Subsystem_M = &Subsystem_M_;

/* Forward declaration for local functions */
static void Subsystem_sin(real_T x[64/*1024*/]);

/* Function for MATLAB Function: '<S1>/ringmod Function' */
static void Subsystem_sin(real_T x[64/*1024*/])
{
  int32_T k;
  for (k = 0; k < 64/*1024*/; k++) {
    x[k] = sin(x[k]);
  }
}

/* Model step function */
void Subsystem_step(double* buffer, int sample_number, param** params)
{
  real_T y;
  int32_T i;

  /******************************/
  //input
  double* signal = buffer;
  double freq = params[0]->now;//0~0~4000, 1000 "hz"
  double depth = params[1]->now;//0~1, 0.5 ""
  double fs  = params[2]->now;//1000~10000, 3000 "hz"
  //output
  double* Out1 = buffer;
  /******************************/

  /* MATLAB Function: '<S1>/ringmod Function' incorporates:
   *  Inport: '<Root>/freq'
   *  Inport: '<Root>/fs'
   */
  /* MATLAB Function 'Subsystem/ringmod Function': '<S2>:1' */
  /* RINGMOD ring modulates input signal at specified frequency and depth */
  /*  in is input signal */
  /*  modfreq is frequency of the ring mod */
  /*  depth ranges 0 (no ringmod added) to 1 (equal amplitude added) */
  /*  fs is sampling freq of input signal */
  /* '<S2>:1:8' */
  /* '<S2>:1:9' */
  y = 6.2831853071795862 * freq / fs;
  for (i = 0; i < sample_number/*1024*/; i++) {
    Subsystem_B.out[i] = (1.0 + (real_T)i) * y;
  }

  Subsystem_sin(Subsystem_B.out);

  /* '<S2>:1:10' */
  /* '<S2>:1:12' */
  for (i = 0; i < sample_number/*(1024*/; i++) {
    /* Outport: '<Root>/Out1' incorporates:
     *  Inport: '<Root>/depth'
     *  Inport: '<Root>/signal'
     *  MATLAB Function: '<S1>/ringmod Function'
     */
    Out1[i] = depth * Subsystem_B.out[i] *
      signal[i] + signal[i];
  }
}

/* Model initialize function */
void Subsystem_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(Subsystem_M, (NULL));

  /* external inputs */
  (void) memset((void *)&Subsystem_U, 0,
                sizeof(ExtU_Subsystem_T));

  /* external outputs */
  (void) memset(&Subsystem_Y.Out1[0], 0,
                64U/*1024*/*sizeof(real_T));
}

/* Model terminate function */
void Subsystem_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
