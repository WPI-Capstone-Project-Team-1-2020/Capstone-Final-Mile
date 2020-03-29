/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include files */
#include "main.h"
#include "motorspeed.h"
#include "quadrotorPropulsion.h"
#include "quadrotorPropulsion_terminate.h"

/* Function Declarations */
static void argInit_10x1_real_T(double result[10]);
static void argInit_2x1_real_T(double result[2]);
static void argInit_4x1_real_T(double result[4]);
static void argInit_PropulsionParameters(PropulsionParameters *result);
static double argInit_real_T(void);
static void main_motorspeed(void);
static void main_quadrotorPropulsion(void);

/* Function Definitions */
static void argInit_10x1_real_T(double result[10])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 10; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_2x1_real_T(double result[2])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;
}

static void argInit_4x1_real_T(double result[4])
{
  double result_tmp_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp_tmp = argInit_real_T();
  result[0] = result_tmp_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[2] = result_tmp_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[3] = argInit_real_T();
}

static void argInit_PropulsionParameters(PropulsionParameters *result)
{
  double result_tmp_tmp_tmp_tmp;

  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  result_tmp_tmp_tmp_tmp = argInit_real_T();
  result->k_m = result_tmp_tmp_tmp_tmp;
  result->k_t = result_tmp_tmp_tmp_tmp;
  result->CT2s = result_tmp_tmp_tmp_tmp;
  result->CT1s = result_tmp_tmp_tmp_tmp;
  result->CT0s = result_tmp_tmp_tmp_tmp;
  result->Psi = argInit_real_T();
  result->J_M = argInit_real_T();
  result->R_A = argInit_real_T();
  result->alpha_m = argInit_real_T();
  result->beta_m = argInit_real_T();
  result->l_m = argInit_real_T();
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void main_motorspeed(void)
{
  double dv[2];
  PropulsionParameters r;
  double M_e;
  double b_I;
  double xpred;

  /* Initialize function 'motorspeed' input arguments. */
  /* Initialize function input argument 'uin'. */
  /* Initialize function input argument 'parameter'. */
  /* Call the entry-point 'motorspeed'. */
  argInit_2x1_real_T(dv);
  argInit_PropulsionParameters(&r);
  motorspeed(argInit_real_T(), dv, &r, argInit_real_T(), &M_e, &b_I, &xpred);
}

static void main_quadrotorPropulsion(void)
{
  double dv[4];
  double dv1[10];
  PropulsionParameters r;
  double y[14];
  double xpred[4];

  /* Initialize function 'quadrotorPropulsion' input arguments. */
  /* Initialize function input argument 'xin'. */
  /* Initialize function input argument 'uin'. */
  /* Initialize function input argument 'parameter'. */
  /* Call the entry-point 'quadrotorPropulsion'. */
  argInit_4x1_real_T(dv);
  argInit_10x1_real_T(dv1);
  argInit_PropulsionParameters(&r);
  quadrotorPropulsion(dv, dv1, &r, argInit_real_T(), y, xpred);
}

int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_quadrotorPropulsion();
  main_motorspeed();

  /* Terminate the application.
     You do not need to do this more than one time. */
  quadrotorPropulsion_terminate();
  return 0;
}

/* End of code generation (main.c) */
