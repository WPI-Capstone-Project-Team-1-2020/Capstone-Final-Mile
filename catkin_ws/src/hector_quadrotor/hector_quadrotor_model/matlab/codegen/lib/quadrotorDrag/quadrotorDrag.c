/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * quadrotorDrag.c
 *
 * Code generation for function 'quadrotorDrag'
 *
 */

/* Include files */
#include "quadrotorDrag.h"
#include <math.h>

/* Function Definitions */
void quadrotorDrag(const double uin[6], const DragParameters *parameter, double
                   dt, double y[6])
{
  double absoluteVelocity;
  double absoluteAngularVelocity;
  double y_tmp;
  (void)dt;

  /*  initialize vectors */
  /*  Input variables */
  /*  Constants */
  /*  temporarily used vector */
  absoluteVelocity = sqrt((uin[0] * uin[0] + uin[1] * uin[1]) + uin[2] * uin[2]);
  absoluteAngularVelocity = sqrt((uin[3] * uin[3] + uin[4] * uin[4]) + uin[5] *
    uin[5]);

  /*  system outputs */
  /*  calculate drag force */
  y_tmp = parameter->C_wxy * absoluteVelocity;
  y[0] = y_tmp * uin[0];
  y[1] = y_tmp * uin[1];
  y[2] = parameter->C_wz * absoluteVelocity * uin[2];

  /*  calculate draq torque */
  y_tmp = parameter->C_mxy * absoluteAngularVelocity;
  y[3] = y_tmp * uin[3];
  y[4] = y_tmp * uin[4];
  y[5] = parameter->C_mz * absoluteAngularVelocity * uin[5];
}

/* End of code generation (quadrotorDrag.c) */
