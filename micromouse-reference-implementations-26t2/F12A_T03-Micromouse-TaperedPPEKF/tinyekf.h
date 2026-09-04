/*
 * Extended Kalman Filter for embedded processors
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * MIT License
 */

#ifndef TINYEKF_H
#define TINYEKF_H

#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifndef _float_t
#define _float_t float
#endif

static void _mulmat(
        const _float_t * a,
        const _float_t * b,
        _float_t * c,
        const int arows,
        const int acols,
        const int bcols)
{
    for (int i = 0; i < arows; ++i) {
        for (int j = 0; j < bcols; ++j) {
            _float_t sum = 0;
            for (int k = 0; k < acols; ++k) {
                sum += a[i * acols + k] * b[k * bcols + j];
            }
            c[i * bcols + j] = sum;
        }
    }
}

static void _transpose(
        const _float_t * a, _float_t * at, const int m, const int n)
{
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            at[j * m + i] = a[i * n + j];
        }
    }
}

typedef struct {
    _float_t x[EKF_N];
    _float_t P[EKF_N * EKF_N];
} ekf_t;

typedef struct {
    _float_t fx[EKF_N];
    _float_t m0[EKF_N * EKF_N];
    _float_t m1[EKF_N * EKF_N];
    _float_t m2[EKF_N * EKF_N];
} ekf_scratch_t;

static void ekf_initialize(ekf_t * ekf, const _float_t pdiag[EKF_N])
{
    for (int i = 0; i < EKF_N; ++i) {
        for (int j = 0; j < EKF_N; ++j) {
            ekf->P[i * EKF_N + j] = (i == j) ? pdiag[i] : 0;
        }
        ekf->x[i] = 0;
    }
}

static void ekf_predict_diag(
        ekf_t * ekf,
        const _float_t fx[EKF_N],
        const _float_t F[EKF_N * EKF_N],
        const _float_t qdiag[EKF_N],
        ekf_scratch_t * scratch)
{
    memcpy(ekf->x, fx, EKF_N * sizeof(_float_t));

    _mulmat(F, ekf->P, scratch->m0, EKF_N, EKF_N, EKF_N);
    _transpose(F, scratch->m1, EKF_N, EKF_N);
    _mulmat(scratch->m0, scratch->m1, scratch->m2, EKF_N, EKF_N, EKF_N);
    memcpy(ekf->P, scratch->m2, EKF_N * EKF_N * sizeof(_float_t));
    for (int i = 0; i < EKF_N; ++i) {
        ekf->P[i * EKF_N + i] += qdiag[i];
    }
}

static bool ekf_update_scalar(ekf_t * ekf, int idx, _float_t z, _float_t r)
{
    if (idx < 0 || idx >= EKF_N) {
        return false;
    }

    const int row = idx * EKF_N;
    _float_t s = ekf->P[row + idx] + r;
    if (s <= 0.0f || !isfinite(s)) {
        return false;
    }

    _float_t innov = z - ekf->x[idx];
    if (!isfinite(innov)) {
        return false;
    }

    _float_t invs = 1.0f / s;
    for (int i = 0; i < EKF_N; ++i) {
        _float_t k = ekf->P[i * EKF_N + idx] * invs;
        ekf->x[i] += k * innov;
    }

    for (int i = 0; i < EKF_N; ++i) {
        _float_t ki = ekf->P[i * EKF_N + idx] * invs;
        for (int j = 0; j < EKF_N; ++j) {
            ekf->P[i * EKF_N + j] -= ki * ekf->P[row + j];
        }
    }

    return true;
}

#endif /* TINYEKF_H */
