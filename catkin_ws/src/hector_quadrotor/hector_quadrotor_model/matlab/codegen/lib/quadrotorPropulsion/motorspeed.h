/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * motorspeed.h
 *
 * Code generation for function 'motorspeed'
 *
 */

#ifndef MOTORSPEED_H
#define MOTORSPEED_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "quadrotorPropulsion_types.h"

/* Function Declarations */
extern void motorspeed(double xin, const double uin[2], const
  PropulsionParameters *parameter, double dt, double *M_e, double *b_I, double
  *xpred);

#endif

/* End of code generation (motorspeed.h) */
