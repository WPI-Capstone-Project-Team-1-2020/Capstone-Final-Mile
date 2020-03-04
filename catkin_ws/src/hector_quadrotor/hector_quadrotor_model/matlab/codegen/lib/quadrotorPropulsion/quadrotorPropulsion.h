/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * quadrotorPropulsion.h
 *
 * Code generation for function 'quadrotorPropulsion'
 *
 */

#ifndef QUADROTORPROPULSION_H
#define QUADROTORPROPULSION_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "quadrotorPropulsion_types.h"

/* Function Declarations */
extern void quadrotorPropulsion(const double xin[4], const double uin[10], const
  PropulsionParameters *parameter, double dt, double y[14], const double xpred[4]);

#endif

/* End of code generation (quadrotorPropulsion.h) */
