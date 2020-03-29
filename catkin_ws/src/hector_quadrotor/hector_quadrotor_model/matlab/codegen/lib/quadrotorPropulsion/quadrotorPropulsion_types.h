/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * quadrotorPropulsion_types.h
 *
 * Code generation for function 'quadrotorPropulsion_types'
 *
 */

#ifndef QUADROTORPROPULSION_TYPES_H
#define QUADROTORPROPULSION_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_PropulsionParameters
#define typedef_PropulsionParameters

typedef struct {
  double k_m;
  double k_t;
  double CT2s;
  double CT1s;
  double CT0s;
  double Psi;
  double J_M;
  double R_A;
  double alpha_m;
  double beta_m;
  double l_m;
} PropulsionParameters;

#endif                                 /*typedef_PropulsionParameters*/
#endif

/* End of code generation (quadrotorPropulsion_types.h) */
