/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: psgolayp_initialize.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "psgolayp.h"
#include "psgolayp_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void psgolayp_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * File trailer for psgolayp_initialize.c
 *
 * [EOF]
 */
