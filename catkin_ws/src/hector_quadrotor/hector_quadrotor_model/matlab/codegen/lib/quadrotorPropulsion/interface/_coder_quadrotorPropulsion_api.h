/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * _coder_quadrotorPropulsion_api.h
 *
 * Code generation for function '_coder_quadrotorPropulsion_api'
 *
 */

#ifndef _CODER_QUADROTORPROPULSION_API_H
#define _CODER_QUADROTORPROPULSION_API_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_PropulsionParameters
#define typedef_PropulsionParameters

typedef struct {
  real_T k_m;
  real_T k_t;
  real_T CT2s;
  real_T CT1s;
  real_T CT0s;
  real_T Psi;
  real_T J_M;
  real_T R_A;
  real_T alpha_m;
  real_T beta_m;
  real_T l_m;
} PropulsionParameters;

#endif                                 /*typedef_PropulsionParameters*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void motorspeed(real_T xin, real_T uin[2], PropulsionParameters
  *parameter, real_T dt, real_T *M_e, real_T *b_I, real_T *xpred);
extern void motorspeed_api(const mxArray * const prhs[4], int32_T nlhs, const
  mxArray *plhs[3]);
extern void quadrotorPropulsion(real_T xin[4], real_T uin[10],
  PropulsionParameters *parameter, real_T dt, real_T y[14], real_T xpred[4]);
extern void quadrotorPropulsion_api(const mxArray * const prhs[4], int32_T nlhs,
  const mxArray *plhs[2]);
extern void quadrotorPropulsion_atexit(void);
extern void quadrotorPropulsion_initialize(void);
extern void quadrotorPropulsion_terminate(void);
extern void quadrotorPropulsion_xil_shutdown(void);
extern void quadrotorPropulsion_xil_terminate(void);

#endif

/* End of code generation (_coder_quadrotorPropulsion_api.h) */
