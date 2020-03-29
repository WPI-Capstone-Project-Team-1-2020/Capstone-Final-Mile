/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * quadrotorPropulsion.c
 *
 * Code generation for function 'quadrotorPropulsion'
 *
 */

/* Include files */
#include "quadrotorPropulsion.h"
#include "motorspeed.h"

/* Function Definitions */
void quadrotorPropulsion(const double xin[4], const double uin[10], const
  PropulsionParameters *parameter, double dt, double y[14], const double xpred[4])
{
  double v_1_idx_3;
  double v_1_idx_0;
  double v_1_idx_1_tmp;
  double v_1_idx_1;
  double v_1_idx_2;
  (void)xin;
  (void)dt;

  /*  initialize vectors */
  /*  motorspeed */
  /*  Input variables */
  /*  Constants */
  v_1_idx_3 = parameter->l_m * uin[4];
  v_1_idx_0 = -uin[2] + v_1_idx_3;
  v_1_idx_1_tmp = parameter->l_m * uin[3];
  v_1_idx_1 = -uin[2] - v_1_idx_1_tmp;
  v_1_idx_2 = -uin[2] - v_1_idx_3;
  v_1_idx_3 = -uin[2] + v_1_idx_1_tmp;

  /*  calculate thrust for all 4 rotors */
  /*  System output, i.e. force and torque of quadrotor */
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = 0.0;

  /*  torque for rotating quadrocopter around x-axis is the mechanical torque */
  y[3] = (v_1_idx_3 - v_1_idx_1) * parameter->l_m;

  /*  torque for rotating quadrocopter around y-axis is the mechanical torque */
  y[4] = (v_1_idx_0 - v_1_idx_2) * parameter->l_m;

  /*  torque for rotating quadrocopter around z-axis is the electrical torque */
  y[5] = ((-v_1_idx_0 - v_1_idx_2) + v_1_idx_1) + v_1_idx_3;

  /*  motor speeds (rad/s) */
  /*  motor current (A) */
  y[6] = xpred[0];
  y[10] = v_1_idx_0;
  y[7] = xpred[1];
  y[11] = v_1_idx_1;
  y[8] = xpred[2];
  y[12] = v_1_idx_2;
  y[9] = xpred[3];
  y[13] = v_1_idx_3;

  /*  M_e(1:4) / Psi; */
}

/* End of code generation (quadrotorPropulsion.c) */
