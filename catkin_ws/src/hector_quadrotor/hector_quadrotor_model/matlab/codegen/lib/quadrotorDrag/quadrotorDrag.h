/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * quadrotorDrag.h
 *
 * Code generation for function 'quadrotorDrag'
 *
 */

#ifndef QUADROTORDRAG_H
#define QUADROTORDRAG_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "quadrotorDrag_types.h"

/* Function Declarations */
extern void quadrotorDrag(const double uin[6], const DragParameters *parameter,
  double dt, double y[6]);

#endif

/* End of code generation (quadrotorDrag.h) */
