/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgeqp3.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 22-Jan-2019 16:55:26
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "psgolayp.h"
#include "xgeqp3.h"
#include "xnrm2.h"
#include "xscal.h"
#include "psgolayp_emxutil.h"
#include "colon.h"

/* Function Declarations */
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : emxArray_real_T *A
 *                emxArray_real_T *tau
 *                emxArray_int32_T *jpvt
 * Return Type  : void
 */
void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T *jpvt)
{
  int m;
  int n;
  int itemp;
  int mn;
  int i2;
  emxArray_real_T *work;
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  int k;
  int nmi;
  int i;
  int i_i;
  int mmi;
  int ix;
  int pvt;
  double smax;
  int iy;
  double t;
  double absxk;
  double s;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  m = A->size[0];
  n = A->size[1];
  itemp = A->size[0];
  mn = A->size[1];
  if (itemp < mn) {
    mn = itemp;
  }

  i2 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i2, sizeof(double));
  eml_signed_integer_colon(A->size[1], jpvt);
  if (!((A->size[0] == 0) || (A->size[1] == 0))) {
    emxInit_real_T1(&work, 1);
    itemp = A->size[1];
    i2 = work->size[0];
    work->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)work, i2, sizeof(double));
    for (i2 = 0; i2 < itemp; i2++) {
      work->data[i2] = 0.0;
    }

    emxInit_real_T1(&vn1, 1);
    emxInit_real_T1(&vn2, 1);
    itemp = A->size[1];
    i2 = vn1->size[0];
    vn1->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)vn1, i2, sizeof(double));
    i2 = vn2->size[0];
    vn2->size[0] = vn1->size[0];
    emxEnsureCapacity((emxArray__common *)vn2, i2, sizeof(double));
    k = 1;
    for (nmi = 0; nmi + 1 <= n; nmi++) {
      vn1->data[nmi] = xnrm2(m, A, k);
      vn2->data[nmi] = vn1->data[nmi];
      k += m;
    }

    for (i = 0; i + 1 <= mn; i++) {
      i_i = i + i * m;
      nmi = n - i;
      mmi = (m - i) - 1;
      if (nmi < 1) {
        itemp = 0;
      } else {
        itemp = 1;
        if (nmi > 1) {
          ix = i;
          smax = fabs(vn1->data[i]);
          for (k = 2; k <= nmi; k++) {
            ix++;
            s = fabs(vn1->data[ix]);
            if (s > smax) {
              itemp = k;
              smax = s;
            }
          }
        }
      }

      pvt = (i + itemp) - 1;
      if (pvt + 1 != i + 1) {
        ix = m * pvt;
        iy = m * i;
        for (k = 1; k <= m; k++) {
          smax = A->data[ix];
          A->data[ix] = A->data[iy];
          A->data[iy] = smax;
          ix++;
          iy++;
        }

        itemp = jpvt->data[pvt];
        jpvt->data[pvt] = jpvt->data[i];
        jpvt->data[i] = itemp;
        vn1->data[pvt] = vn1->data[i];
        vn2->data[pvt] = vn2->data[i];
      }

      if (i + 1 < m) {
        t = A->data[i_i];
        absxk = 0.0;
        if (!(1 + mmi <= 0)) {
          smax = b_xnrm2(mmi, A, i_i + 2);
          if (smax != 0.0) {
            s = rt_hypotd_snf(A->data[i_i], smax);
            if (A->data[i_i] >= 0.0) {
              s = -s;
            }

            if (fabs(s) < 1.0020841800044864E-292) {
              pvt = 0;
              do {
                pvt++;
                xscal(mmi, 9.9792015476736E+291, A, i_i + 2);
                s *= 9.9792015476736E+291;
                t *= 9.9792015476736E+291;
              } while (!(fabs(s) >= 1.0020841800044864E-292));

              smax = b_xnrm2(mmi, A, i_i + 2);
              s = rt_hypotd_snf(t, smax);
              if (t >= 0.0) {
                s = -s;
              }

              absxk = (s - t) / s;
              xscal(mmi, 1.0 / (t - s), A, i_i + 2);
              for (k = 1; k <= pvt; k++) {
                s *= 1.0020841800044864E-292;
              }

              t = s;
            } else {
              absxk = (s - A->data[i_i]) / s;
              smax = 1.0 / (A->data[i_i] - s);
              xscal(mmi, smax, A, i_i + 2);
              t = s;
            }
          }
        }

        tau->data[i] = absxk;
        A->data[i_i] = t;
      } else {
        tau->data[i] = 0.0;
      }

      if (i + 1 < n) {
        t = A->data[i_i];
        A->data[i_i] = 1.0;
        i_ip1 = (i + (i + 1) * m) + 1;
        if (tau->data[i] != 0.0) {
          lastv = mmi + 1;
          itemp = i_i + mmi;
          while ((lastv > 0) && (A->data[itemp] == 0.0)) {
            lastv--;
            itemp--;
          }

          lastc = nmi - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            itemp = i_ip1 + (lastc - 1) * m;
            k = itemp;
            do {
              exitg1 = 0;
              if (k <= (itemp + lastv) - 1) {
                if (A->data[k - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  k++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }

        if (lastv > 0) {
          if (lastc != 0) {
            for (iy = 1; iy <= lastc; iy++) {
              work->data[iy - 1] = 0.0;
            }

            iy = 0;
            i2 = i_ip1 + m * (lastc - 1);
            itemp = i_ip1;
            while ((m > 0) && (itemp <= i2)) {
              ix = i_i;
              smax = 0.0;
              pvt = (itemp + lastv) - 1;
              for (k = itemp; k <= pvt; k++) {
                smax += A->data[k - 1] * A->data[ix];
                ix++;
              }

              work->data[iy] += smax;
              iy++;
              itemp += m;
            }
          }

          if (!(-tau->data[i] == 0.0)) {
            itemp = i_ip1 - 1;
            pvt = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if (work->data[pvt] != 0.0) {
                smax = work->data[pvt] * -tau->data[i];
                ix = i_i;
                i2 = lastv + itemp;
                for (k = itemp; k + 1 <= i2; k++) {
                  A->data[k] += A->data[ix] * smax;
                  ix++;
                }
              }

              pvt++;
              itemp += m;
            }
          }
        }

        A->data[i_i] = t;
      }

      for (nmi = i + 1; nmi + 1 <= n; nmi++) {
        itemp = (i + m * nmi) + 1;
        if (vn1->data[nmi] != 0.0) {
          smax = fabs(A->data[i + A->size[0] * nmi]) / vn1->data[nmi];
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          s = vn1->data[nmi] / vn2->data[nmi];
          s = smax * (s * s);
          if (s <= 1.4901161193847656E-8) {
            if (i + 1 < m) {
              smax = 0.0;
              if (!(mmi < 1)) {
                if (mmi == 1) {
                  smax = fabs(A->data[itemp]);
                } else {
                  s = 2.2250738585072014E-308;
                  pvt = itemp + mmi;
                  while (itemp + 1 <= pvt) {
                    absxk = fabs(A->data[itemp]);
                    if (absxk > s) {
                      t = s / absxk;
                      smax = 1.0 + smax * t * t;
                      s = absxk;
                    } else {
                      t = absxk / s;
                      smax += t * t;
                    }

                    itemp++;
                  }

                  smax = s * sqrt(smax);
                }
              }

              vn1->data[nmi] = smax;
              vn2->data[nmi] = vn1->data[nmi];
            } else {
              vn1->data[nmi] = 0.0;
              vn2->data[nmi] = 0.0;
            }
          } else {
            vn1->data[nmi] *= sqrt(smax);
          }
        }
      }
    }

    emxFree_real_T(&vn2);
    emxFree_real_T(&vn1);
    emxFree_real_T(&work);
  }
}

/*
 * File trailer for xgeqp3.c
 *
 * [EOF]
 */
