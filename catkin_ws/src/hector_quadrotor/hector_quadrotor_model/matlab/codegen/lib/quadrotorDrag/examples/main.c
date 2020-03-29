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
#include "quadrotorDrag.h"
#include "quadrotorDrag_terminate.h"

/* Function Declarations */
static void argInit_6x1_real_T(double result[6]);
static DragParameters argInit_DragParameters(void);
static double argInit_real_T(void);
static void main_quadrotorDrag(void);

/* Function Definitions */
static void argInit_6x1_real_T(double result[6])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static DragParameters argInit_DragParameters(void)
{
  DragParameters result;
  double result_tmp_tmp_tmp;

  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  result_tmp_tmp_tmp = argInit_real_T();
  result.C_wxy = result_tmp_tmp_tmp;
  result.C_wz = result_tmp_tmp_tmp;
  result.C_mxy = result_tmp_tmp_tmp;
  result.C_mz = result_tmp_tmp_tmp;
  return result;
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void main_quadrotorDrag(void)
{
  double dv[6];
  DragParameters r;
  double y[6];

  /* Initialize function 'quadrotorDrag' input arguments. */
  /* Initialize function input argument 'uin'. */
  /* Initialize function input argument 'parameter'. */
  /* Call the entry-point 'quadrotorDrag'. */
  argInit_6x1_real_T(dv);
  r = argInit_DragParameters();
  quadrotorDrag(dv, &r, argInit_real_T(), y);
}

int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_quadrotorDrag();

  /* Terminate the application.
     You do not need to do this more than one time. */
  quadrotorDrag_terminate();
  return 0;
}

/* End of code generation (main.c) */
