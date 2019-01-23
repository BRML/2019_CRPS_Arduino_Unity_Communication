/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_psgolayp_api.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

#ifndef _CODER_PSGOLAYP_API_H
#define _CODER_PSGOLAYP_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_psgolayp_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void psgolayp(real_T A[10], real_T k, real_T n, real_T SG0[10]);
extern void psgolayp_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
extern void psgolayp_atexit(void);
extern void psgolayp_initialize(void);
extern void psgolayp_terminate(void);
extern void psgolayp_xil_terminate(void);

#endif

/*
 * File trailer for _coder_psgolayp_api.h
 *
 * [EOF]
 */
