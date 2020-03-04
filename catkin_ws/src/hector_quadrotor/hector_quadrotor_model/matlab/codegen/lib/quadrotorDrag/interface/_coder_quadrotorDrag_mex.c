/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * _coder_quadrotorDrag_mex.c
 *
 * Code generation for function '_coder_quadrotorDrag_mex'
 *
 */

/* Include files */
#include "_coder_quadrotorDrag_mex.h"
#include "_coder_quadrotorDrag_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void quadrotorDrag_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3]);

/* Function Definitions */
void quadrotorDrag_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[3])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        13, "quadrotorDrag");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "quadrotorDrag");
  }

  /* Call the function. */
  quadrotorDrag_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(quadrotorDrag_atexit);

  /* Module initialization. */
  quadrotorDrag_initialize();

  /* Dispatch the entry-point. */
  quadrotorDrag_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  quadrotorDrag_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_quadrotorDrag_mex.c) */
