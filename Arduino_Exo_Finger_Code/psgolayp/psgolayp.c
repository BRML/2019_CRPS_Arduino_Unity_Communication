/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: psgolayp.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "psgolayp.h"
#include "psgolayp_emxutil.h"
#include "lusolve.h"
#include "xgeqp3.h"
#include "eye.h"

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = 1.0;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * psgolay compute x, x', x" from data A with k-polynomial over
 *    the preceding 2*n+1 points.
 *    Differs from standard sgolay in that it introduces NO delay
 *    2*n+1 is the number of past points that are used in the filter
 *    k is the order of the polynomial
 *
 *    Example use:
 *    T=0.05;t=(0:99)*T; s = sin(2*pi*t); s_noise = s + 0.5*rand(size(t));
 *    plot(s_noise); hold on; d=psgolayp(s_noise,2,20);plot(d,'g'); hold off;
 * Arguments    : const double A[10]
 *                double k
 *                double n
 *                double SG0[10]
 * Return Type  : void
 */
void psgolayp(const double A[10], double k, double n, double SG0[10])
{
  emxArray_real_T *hx;
  int i0;
  double ndbl;
  double apnd;
  double cdiff;
  double absa;
  emxArray_real_T *y;
  double absb;
  int loop_ub;
  emxArray_real_T *result;
  int vstride;
  int ia;
  int br;
  boolean_T empty_non_axis_sizes;
  int nm1d2;
  cell_wrap_0 reshapes[2];
  int b_k;
  emxArray_real_T *df;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int rankR;
  emxArray_real_T *b_df;
  int nb;
  int m;
  emxArray_real_T *b_hx;
  emxArray_real_T *fc;

  /* if round(n) ~= n  */
  /*     error(generatemsgid('MustBeInteger'),'Number of data points must be an integer.'), end */
  /* if round(k) ~= k */
  /*     error(generatemsgid('MustBeInteger'),'Polynomial order must be an integer.'), end */
  /* if k > n-1, error(generatemsgid('InvalidRange'),'It is silly to have a polynomial degree higher than # data.'), end */
  /*  compute the SG parameters for the last item.  Based on code from */
  /*  Diederick C. Niehorster <dcniehorster@hku.hk>.  Lots of magic ;-) */
  emxInit_real_T(&hx, 2);
  if (rtIsNaN(-n) || rtIsNaN(n)) {
    i0 = hx->size[0] * hx->size[1];
    hx->size[0] = 1;
    hx->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
    hx->data[0] = rtNaN;
  } else if (n < -n) {
    i0 = hx->size[0] * hx->size[1];
    hx->size[0] = 1;
    hx->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
  } else if ((rtIsInf(-n) || rtIsInf(n)) && (-n == n)) {
    i0 = hx->size[0] * hx->size[1];
    hx->size[0] = 1;
    hx->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
    hx->data[0] = rtNaN;
  } else if (floor(-n) == -n) {
    i0 = hx->size[0] * hx->size[1];
    hx->size[0] = 1;
    hx->size[1] = (int)floor(n - (-n)) + 1;
    emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
    loop_ub = (int)floor(n - (-n));
    for (i0 = 0; i0 <= loop_ub; i0++) {
      hx->data[hx->size[0] * i0] = -n + (double)i0;
    }
  } else {
    ndbl = floor((n - (-n)) + 0.5);
    apnd = -n + ndbl;
    cdiff = apnd - n;
    absa = fabs(-n);
    absb = fabs(n);
    if ((absa > absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = n;
    } else if (cdiff > 0.0) {
      apnd = -n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      vstride = (int)ndbl;
    } else {
      vstride = 0;
    }

    i0 = hx->size[0] * hx->size[1];
    hx->size[0] = 1;
    hx->size[1] = vstride;
    emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
    if (vstride > 0) {
      hx->data[0] = -n;
      if (vstride > 1) {
        hx->data[vstride - 1] = apnd;
        nm1d2 = (vstride - 1) / 2;
        for (b_k = 1; b_k < nm1d2; b_k++) {
          hx->data[b_k] = -n + (double)b_k;
          hx->data[(vstride - b_k) - 1] = apnd - (double)b_k;
        }

        if (nm1d2 << 1 == vstride - 1) {
          hx->data[nm1d2] = (-n + apnd) / 2.0;
        } else {
          hx->data[nm1d2] = -n + (double)nm1d2;
          hx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }
  }

  emxInit_real_T(&y, 2);
  i0 = y->size[0] * y->size[1];
  y->size[0] = hx->size[1];
  y->size[1] = (int)k;
  emxEnsureCapacity((emxArray__common *)y, i0, sizeof(double));
  loop_ub = hx->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    vstride = (int)k;
    for (ia = 0; ia < vstride; ia++) {
      y->data[i0 + y->size[0] * ia] = hx->data[hx->size[0] * i0];
    }
  }

  emxInit_real_T(&result, 2);
  if (!((int)(2.0 * n + 1.0) == 0)) {
    br = (int)(2.0 * n + 1.0);
  } else if (!((y->size[0] == 0) || (y->size[1] == 0))) {
    br = y->size[0];
  } else {
    br = (int)(2.0 * n + 1.0);
    if (!(br > 0)) {
      br = 0;
    }

    if (y->size[0] > br) {
      br = y->size[0];
    }
  }

  empty_non_axis_sizes = (br == 0);
  if (empty_non_axis_sizes || (!((int)(2.0 * n + 1.0) == 0))) {
    nm1d2 = 1;
  } else {
    nm1d2 = 0;
  }

  emxInitMatrix_cell_wrap_0(reshapes);
  i0 = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
  reshapes[0].f1->size[0] = br;
  reshapes[0].f1->size[1] = nm1d2;
  emxEnsureCapacity((emxArray__common *)reshapes[0].f1, i0, sizeof(double));
  loop_ub = br * nm1d2;
  for (i0 = 0; i0 < loop_ub; i0++) {
    reshapes[0].f1->data[i0] = 1.0;
  }

  if (empty_non_axis_sizes || (!((y->size[0] == 0) || (y->size[1] == 0)))) {
    nm1d2 = y->size[1];
  } else {
    nm1d2 = 0;
  }

  i0 = result->size[0] * result->size[1];
  result->size[0] = reshapes[0].f1->size[0];
  result->size[1] = reshapes[0].f1->size[1] + nm1d2;
  emxEnsureCapacity((emxArray__common *)result, i0, sizeof(double));
  loop_ub = reshapes[0].f1->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    vstride = reshapes[0].f1->size[0];
    for (ia = 0; ia < vstride; ia++) {
      result->data[ia + result->size[0] * i0] = reshapes[0].f1->data[ia +
        reshapes[0].f1->size[0] * i0];
    }
  }

  for (i0 = 0; i0 < nm1d2; i0++) {
    for (ia = 0; ia < br; ia++) {
      result->data[ia + result->size[0] * (i0 + reshapes[0].f1->size[1])] =
        y->data[ia + br * i0];
    }
  }

  emxFreeMatrix_cell_wrap_0(reshapes);
  nm1d2 = result->size[1];
  if ((!((result->size[0] == 0) || (result->size[1] == 0))) && (result->size[1] >
       1)) {
    vstride = result->size[0];
    for (ia = 0; ia + 1 <= vstride; ia++) {
      for (b_k = 1; b_k < nm1d2; b_k++) {
        result->data[ia + b_k * vstride] *= result->data[ia + (b_k - 1) *
          vstride];
      }
    }
  }

  eye(2.0 * n + 1.0, y);
  emxInit_real_T(&df, 2);
  emxInit_real_T1(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  if ((result->size[0] == 0) || (result->size[1] == 0) || ((y->size[0] == 0) ||
       (y->size[1] == 0))) {
    unnamed_idx_0 = (unsigned int)result->size[1];
    unnamed_idx_1 = (unsigned int)y->size[1];
    i0 = df->size[0] * df->size[1];
    df->size[0] = (int)unnamed_idx_0;
    df->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)df, i0, sizeof(double));
    loop_ub = (int)unnamed_idx_0 * (int)unnamed_idx_1;
    for (i0 = 0; i0 < loop_ub; i0++) {
      df->data[i0] = 0.0;
    }
  } else if (result->size[0] == result->size[1]) {
    lusolve(result, y, df);
  } else {
    xgeqp3(result, tau, jpvt);
    rankR = 0;
    if (result->size[0] < result->size[1]) {
      nm1d2 = result->size[0];
      vstride = result->size[1];
    } else {
      nm1d2 = result->size[1];
      vstride = result->size[0];
    }

    if (nm1d2 > 0) {
      ndbl = (double)vstride * fabs(result->data[0]) * 2.2204460492503131E-16;
      while ((rankR < nm1d2) && (fabs(result->data[rankR + result->size[0] *
               rankR]) >= ndbl)) {
        rankR++;
      }
    }

    nb = y->size[1];
    nm1d2 = result->size[1];
    vstride = y->size[1];
    i0 = df->size[0] * df->size[1];
    df->size[0] = nm1d2;
    df->size[1] = vstride;
    emxEnsureCapacity((emxArray__common *)df, i0, sizeof(double));
    loop_ub = nm1d2 * vstride;
    for (i0 = 0; i0 < loop_ub; i0++) {
      df->data[i0] = 0.0;
    }

    m = result->size[0];
    nm1d2 = y->size[1];
    br = result->size[0];
    vstride = result->size[1];
    if (br < vstride) {
      vstride = br;
    }

    for (ia = 0; ia + 1 <= vstride; ia++) {
      if (tau->data[ia] != 0.0) {
        for (b_k = 0; b_k + 1 <= nm1d2; b_k++) {
          ndbl = y->data[ia + y->size[0] * b_k];
          for (br = ia + 1; br + 1 <= m; br++) {
            ndbl += result->data[br + result->size[0] * ia] * y->data[br +
              y->size[0] * b_k];
          }

          ndbl *= tau->data[ia];
          if (ndbl != 0.0) {
            y->data[ia + y->size[0] * b_k] -= ndbl;
            for (br = ia + 1; br + 1 <= m; br++) {
              y->data[br + y->size[0] * b_k] -= result->data[br + result->size[0]
                * ia] * ndbl;
            }
          }
        }
      }
    }

    for (b_k = 0; b_k + 1 <= nb; b_k++) {
      for (br = 0; br + 1 <= rankR; br++) {
        df->data[(jpvt->data[br] + df->size[0] * b_k) - 1] = y->data[br +
          y->size[0] * b_k];
      }

      for (ia = rankR - 1; ia + 1 > 0; ia--) {
        df->data[(jpvt->data[ia] + df->size[0] * b_k) - 1] /= result->data[ia +
          result->size[0] * ia];
        for (br = 0; br + 1 <= ia; br++) {
          df->data[(jpvt->data[br] + df->size[0] * b_k) - 1] -= df->data
            [(jpvt->data[ia] + df->size[0] * b_k) - 1] * result->data[br +
            result->size[0] * ia];
        }
      }
    }
  }

  emxFree_int32_T(&jpvt);
  emxFree_real_T(&result);
  emxFree_real_T(&y);
  emxInit_real_T(&b_df, 2);
  i0 = b_df->size[0] * b_df->size[1];
  b_df->size[0] = df->size[1];
  b_df->size[1] = df->size[0];
  emxEnsureCapacity((emxArray__common *)b_df, i0, sizeof(double));
  loop_ub = df->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    vstride = df->size[1];
    for (ia = 0; ia < vstride; ia++) {
      b_df->data[ia + b_df->size[0] * i0] = df->data[i0 + df->size[0] * ia];
    }
  }

  i0 = df->size[0] * df->size[1];
  df->size[0] = b_df->size[0];
  df->size[1] = b_df->size[1];
  emxEnsureCapacity((emxArray__common *)df, i0, sizeof(double));
  loop_ub = b_df->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    vstride = b_df->size[0];
    for (ia = 0; ia < vstride; ia++) {
      df->data[ia + df->size[0] * i0] = b_df->data[ia + b_df->size[0] * i0];
    }
  }

  emxFree_real_T(&b_df);
  memset(&SG0[0], 0, 10U * sizeof(double));
  i0 = hx->size[0] * hx->size[1];
  hx->size[0] = 1;
  hx->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
  hx->data[0] = 1.0;
  br = 0;
  emxInit_real_T(&b_hx, 2);
  while (br <= (int)k - 1) {
    i0 = b_hx->size[0] * b_hx->size[1];
    b_hx->size[0] = 1;
    b_hx->size[1] = hx->size[1] + 1;
    emxEnsureCapacity((emxArray__common *)b_hx, i0, sizeof(double));
    loop_ub = hx->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_hx->data[b_hx->size[0] * i0] = hx->data[hx->size[0] * i0];
    }

    b_hx->data[b_hx->size[0] * hx->size[1]] = rt_powd_snf(n, 1.0 + (double)br);
    i0 = hx->size[0] * hx->size[1];
    hx->size[0] = 1;
    hx->size[1] = b_hx->size[1];
    emxEnsureCapacity((emxArray__common *)hx, i0, sizeof(double));
    loop_ub = b_hx->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      hx->data[hx->size[0] * i0] = b_hx->data[b_hx->size[0] * i0];
    }

    br++;
  }

  emxFree_real_T(&b_hx);

  /*  these are the SG filter coefficients */
  i0 = tau->size[0];
  tau->size[0] = hx->size[1];
  emxEnsureCapacity((emxArray__common *)tau, i0, sizeof(double));
  loop_ub = hx->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    tau->data[i0] = hx->data[hx->size[0] * i0];
  }

  emxFree_real_T(&hx);
  emxInit_real_T1(&fc, 1);
  if ((df->size[1] == 1) || (tau->size[0] == 1)) {
    i0 = fc->size[0];
    fc->size[0] = df->size[0];
    emxEnsureCapacity((emxArray__common *)fc, i0, sizeof(double));
    loop_ub = df->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      fc->data[i0] = 0.0;
      vstride = df->size[1];
      for (ia = 0; ia < vstride; ia++) {
        fc->data[i0] += df->data[i0 + df->size[0] * ia] * tau->data[ia];
      }
    }
  } else {
    b_k = df->size[1];
    unnamed_idx_0 = (unsigned int)df->size[0];
    i0 = fc->size[0];
    fc->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)fc, i0, sizeof(double));
    m = df->size[0];
    nm1d2 = fc->size[0];
    i0 = fc->size[0];
    fc->size[0] = nm1d2;
    emxEnsureCapacity((emxArray__common *)fc, i0, sizeof(double));
    for (i0 = 0; i0 < nm1d2; i0++) {
      fc->data[i0] = 0.0;
    }

    if (df->size[0] != 0) {
      nm1d2 = 0;
      while ((m > 0) && (nm1d2 <= 0)) {
        for (loop_ub = 1; loop_ub <= m; loop_ub++) {
          fc->data[loop_ub - 1] = 0.0;
        }

        nm1d2 = m;
      }

      br = 0;
      nm1d2 = 0;
      while ((m > 0) && (nm1d2 <= 0)) {
        nm1d2 = -1;
        i0 = br + b_k;
        for (vstride = br; vstride + 1 <= i0; vstride++) {
          if (tau->data[vstride] != 0.0) {
            ia = nm1d2;
            for (loop_ub = 0; loop_ub + 1 <= m; loop_ub++) {
              ia++;
              fc->data[loop_ub] += tau->data[vstride] * df->data[ia];
            }
          }

          nm1d2 += m;
        }

        br += b_k;
        nm1d2 = m;
      }
    }
  }

  emxFree_real_T(&tau);
  emxFree_real_T(&df);

  /*  now do the smoothing! */
  apnd = 2.0 * n + 1.0;
  for (br = 0; br < (int)(10.0 + (1.0 - apnd)); br++) {
    cdiff = apnd + (double)br;
    ndbl = cdiff - 2.0 * n;
    if (ndbl > cdiff) {
      i0 = 1;
    } else {
      i0 = (int)ndbl;
    }

    ndbl = 0.0;
    if (!(fc->size[0] < 1)) {
      for (b_k = 0; b_k + 1 <= fc->size[0]; b_k++) {
        ndbl += fc->data[b_k] * A[(i0 + b_k) - 1];
      }
    }

    SG0[(int)cdiff - 1] = ndbl;
  }

  emxFree_real_T(&fc);
}

/*
 * File trailer for psgolayp.c
 *
 * [EOF]
 */
