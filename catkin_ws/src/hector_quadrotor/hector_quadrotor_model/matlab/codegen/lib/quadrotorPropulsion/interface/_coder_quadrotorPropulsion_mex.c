/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * _coder_quadrotorPropulsion_mex.c
 *
 * Code generation for function '_coder_quadrotorPropulsion_mex'
 *
 */

/* Include files */
#include "_coder_quadrotorPropulsion_mex.h"
#include "_coder_quadrotorPropulsion_api.h"

/* Variable Definitions */
static const char * emlrtEntryPoints[2] = { "quadrotorPropulsion", "motorspeed"
};

/* Function Declarations */
MEXFUNCTION_LINKAGE void motorspeed_mexFunction(int32_T nlhs, mxArray *plhs[3],
  int32_T nrhs, const mxArray *prhs[4]);
MEXFUNCTION_LINKAGE void quadrotorPropulsion_mexFunction(int32_T nlhs, mxArray
  *plhs[2], int32_T nrhs, const mxArray *prhs[4]);

/* Function Definitions */
void motorspeed_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs, const
  mxArray *prhs[4])
{
  const mxArray *outputs[3];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        10, "motorspeed");
  }

  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 10,
                        "motorspeed");
  }

  /* Call the function. */
  motorspeed_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
}

void quadrotorPropulsion_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T
  nrhs, const mxArray *prhs[4])
{
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        19, "quadrotorPropulsion");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 19,
                        "quadrotorPropulsion");
  }

  /* Call the function. */
  quadrotorPropulsion_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexAtExit(quadrotorPropulsion_atexit);

  /* Module initialization. */
  quadrotorPropulsion_initialize();
  st.tls = emlrtRootTLSGlobal;

  /* Dispatch the entry-point. */
  switch (emlrtGetEntryPointIndexR2016a(&st, nrhs, prhs, emlrtEntryPoints, 2)) {
   case 0:
    quadrotorPropulsion_mexFunction(nlhs, plhs, nrhs - 1, *(const mxArray *(*)[4])
      &prhs[1]);
    break;

   case 1:
    motorspeed_mexFunction(nlhs, plhs, nrhs - 1, *(const mxArray *(*)[4])&prhs[1]);
    break;
  }

  /* Module termination. */
  quadrotorPropulsion_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_quadrotorPropulsion_mex.c) */
