/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * _coder_quadrotorDrag_api.h
 *
 * Code generation for function '_coder_quadrotorDrag_api'
 *
 */

#ifndef _CODER_QUADROTORDRAG_API_H
#define _CODER_QUADROTORDRAG_API_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_DragParameters
#define typedef_DragParameters

typedef struct {
  real_T C_wxy;
  real_T C_wz;
  real_T C_mxy;
  real_T C_mz;
} DragParameters;

#endif                                 /*typedef_DragParameters*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void quadrotorDrag(real_T uin[6], DragParameters *parameter, real_T dt,
  real_T y[6]);
extern void quadrotorDrag_api(const mxArray * const prhs[3], int32_T nlhs, const
  mxArray *plhs[1]);
extern void quadrotorDrag_atexit(void);
extern void quadrotorDrag_initialize(void);
extern void quadrotorDrag_terminate(void);
extern void quadrotorDrag_xil_shutdown(void);
extern void quadrotorDrag_xil_terminate(void);

#endif

/* End of code generation (_coder_quadrotorDrag_api.h) */
