/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lusolve.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "psgolayp.h"
#include "lusolve.h"
#include "psgolayp_emxutil.h"
#include "colon.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *A
 *                const emxArray_real_T *B
 *                emxArray_real_T *X
 * Return Type  : void
 */
void lusolve(const emxArray_real_T *A, const emxArray_real_T *B, emxArray_real_T
             *X)
{
  emxArray_real_T *b_A;
  int n;
  int i1;
  int iy;
  emxArray_int32_T *ipiv;
  int nb;
  int u1;
  int j;
  int mmj;
  int c;
  int jA;
  int ix;
  int kAcol;
  double smax;
  int k;
  double s;
  emxInit_real_T(&b_A, 2);
  n = A->size[1];
  i1 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i1, sizeof(double));
  iy = A->size[0] * A->size[1];
  for (i1 = 0; i1 < iy; i1++) {
    b_A->data[i1] = A->data[i1];
  }

  emxInit_int32_T(&ipiv, 2);
  iy = A->size[1];
  eml_signed_integer_colon(iy, ipiv);
  if (!(A->size[1] < 1)) {
    iy = A->size[1] - 1;
    u1 = A->size[1];
    if (iy < u1) {
      u1 = iy;
    }

    for (j = 0; j + 1 <= u1; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        jA = -1;
      } else {
        jA = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(b_A->data[c]);
          for (k = 2; k <= mmj; k++) {
            ix++;
            s = fabs(b_A->data[ix]);
            if (s > smax) {
              jA = k - 1;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[c + jA] != 0.0) {
        if (jA != 0) {
          ipiv->data[j] = (j + jA) + 1;
          ix = j;
          iy = j + jA;
          for (k = 1; k <= n; k++) {
            smax = b_A->data[ix];
            b_A->data[ix] = b_A->data[iy];
            b_A->data[iy] = smax;
            ix += n;
            iy += n;
          }
        }

        i1 = c + mmj;
        for (jA = c + 1; jA + 1 <= i1; jA++) {
          b_A->data[jA] /= b_A->data[c];
        }
      }

      kAcol = n - j;
      jA = (c + n) + 1;
      iy = c + n;
      for (nb = 1; nb < kAcol; nb++) {
        smax = b_A->data[iy];
        if (b_A->data[iy] != 0.0) {
          ix = c + 1;
          i1 = mmj + jA;
          for (k = jA; k + 1 < i1; k++) {
            b_A->data[k] += b_A->data[ix] * -smax;
            ix++;
          }
        }

        iy += n;
        jA += n;
      }
    }
  }

  nb = B->size[1];
  i1 = X->size[0] * X->size[1];
  X->size[0] = B->size[0];
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, i1, sizeof(double));
  iy = B->size[0] * B->size[1];
  for (i1 = 0; i1 < iy; i1++) {
    X->data[i1] = B->data[i1];
  }

  for (iy = 0; iy + 1 < n; iy++) {
    if (ipiv->data[iy] != iy + 1) {
      kAcol = ipiv->data[iy] - 1;
      for (jA = 0; jA + 1 <= nb; jA++) {
        smax = X->data[iy + X->size[0] * jA];
        X->data[iy + X->size[0] * jA] = X->data[kAcol + X->size[0] * jA];
        X->data[kAcol + X->size[0] * jA] = smax;
      }
    }
  }

  emxFree_int32_T(&ipiv);
  if (B->size[1] != 0) {
    if (!((X->size[0] == 0) || (X->size[1] == 0))) {
      for (j = 1; j <= nb; j++) {
        iy = n * (j - 1) - 1;
        for (k = 1; k <= n; k++) {
          kAcol = n * (k - 1);
          if (X->data[k + iy] != 0.0) {
            for (jA = k + 1; jA <= n; jA++) {
              X->data[jA + iy] -= X->data[k + iy] * b_A->data[(jA + kAcol) - 1];
            }
          }
        }
      }
    }

    if (!((X->size[0] == 0) || (X->size[1] == 0))) {
      for (j = 1; j <= nb; j++) {
        iy = n * (j - 1) - 1;
        for (k = n; k > 0; k--) {
          kAcol = n * (k - 1) - 1;
          if (X->data[k + iy] != 0.0) {
            smax = X->data[k + iy];
            s = b_A->data[k + kAcol];
            X->data[k + iy] = smax / s;
            for (jA = 1; jA < k; jA++) {
              X->data[jA + iy] -= X->data[k + iy] * b_A->data[jA + kAcol];
            }
          }
        }
      }
    }
  }

  emxFree_real_T(&b_A);
}

/*
 * File trailer for lusolve.c
 *
 * [EOF]
 */
