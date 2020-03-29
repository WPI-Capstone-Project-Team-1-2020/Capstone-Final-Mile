/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * motorspeed.c
 *
 * Code generation for function 'motorspeed'
 *
 */

/* Include files */
#include "motorspeed.h"
#include "quadrotorPropulsion.h"
#include <math.h>

/* Function Definitions */
void motorspeed(double xin, const double uin[2], const PropulsionParameters
                *parameter, double dt, double *M_e, double *b_I, double *xpred)
{
  double temp;

  /*  Identification of Roxxy2827-34 motor with 10x4.5 propeller */
  /*  temporarily used Expressions */
  temp = (uin[0] * parameter->beta_m - parameter->Psi * xin) / (2.0 *
    parameter->R_A);
  *b_I = temp + sqrt(temp * temp + uin[0] * parameter->alpha_m / parameter->R_A);
  *M_e = parameter->Psi * *b_I;

  /*  electrical torque motor 1-4 */
  /*  new version */
  /*  old version */
  /*  fx   = (Psi/R_A*(U-Psi*omega_m) - M_m)/J_M; */
  /*  A    = -(Psi^2/R_A)/J_M; */
  /*  B(1) =  Psi/(J_M*R_A); */
  /*  B(2) = -1/J_M; */
  /*  system outputs. Use euler solver to predict next time step */
  /*  predicted motor speed */
  *xpred = xin + dt * (1.0 / parameter->J_M * (*M_e - (parameter->k_t * uin[1] +
    parameter->k_m * xin)));

  /*  electric torque */
  /* y = [M_e I]; */
  /*  system jacobian */
  /*  A       = 1 + dt*A; */
  /*  input jacobian */
  /*  B       = A*B*dt; */
}

/* End of code generation (motorspeed.c) */
