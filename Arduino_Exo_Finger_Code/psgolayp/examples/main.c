/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
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
/* Include Files */
#include "rt_nonfinite.h"
#include "psgolayp.h"
#include "main.h"
#include "psgolayp_terminate.h"
#include "psgolayp_initialize.h"

/* Function Declarations */
static void argInit_1x10_real_T(double result[10]);
static double argInit_real_T(void);
static void main_psgolayp(void);

/* Function Definitions */

/*
 * Arguments    : double result[10]
 * Return Type  : void
 */
static void argInit_1x10_real_T(double result[10])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 10; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_psgolayp(void)
{
  double dv0[10];
  double SG0[10];

  /* Initialize function 'psgolayp' input arguments. */
  /* Initialize function input argument 'A'. */
  /* Call the entry-point 'psgolayp'. */
  argInit_1x10_real_T(dv0);
  psgolayp(dv0, argInit_real_T(), argInit_real_T(), SG0);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  psgolayp_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_psgolayp();

  /* Terminate the application.
     You do not need to do this more than one time. */
  psgolayp_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
