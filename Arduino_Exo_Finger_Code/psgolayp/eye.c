/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eye.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "psgolayp.h"
#include "eye.h"
#include "psgolayp_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : double varargin_1
 *                emxArray_real_T *I
 * Return Type  : void
 */
void eye(double varargin_1, emxArray_real_T *I)
{
  double t;
  int k;
  int loop_ub;
  if (varargin_1 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_1;
  }

  k = I->size[0] * I->size[1];
  I->size[0] = (int)t;
  I->size[1] = (int)t;
  emxEnsureCapacity((emxArray__common *)I, k, sizeof(double));
  loop_ub = (int)t * (int)t;
  for (k = 0; k < loop_ub; k++) {
    I->data[k] = 0.0;
  }

  if ((int)t > 0) {
    for (k = 0; k + 1 <= (int)t; k++) {
      I->data[k + I->size[0] * k] = 1.0;
    }
  }
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
