/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lusolve.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

#ifndef LUSOLVE_H
#define LUSOLVE_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "psgolayp_types.h"

/* Function Declarations */
extern void lusolve(const emxArray_real_T *A, const emxArray_real_T *B,
                    emxArray_real_T *X);

#endif

/*
 * File trailer for lusolve.h
 *
 * [EOF]
 */
