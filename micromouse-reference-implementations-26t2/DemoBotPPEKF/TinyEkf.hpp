#pragma once

#include <math.h>
#include <string.h>

#ifndef EKF_N
#define EKF_N 3
#endif

struct EkfState {
    float x[EKF_N];
    float covariance[EKF_N * EKF_N];
};

struct EkfScratch {
    float predicted[EKF_N];
    float temporary0[EKF_N * EKF_N];
    float temporary1[EKF_N * EKF_N];
    float temporary2[EKF_N * EKF_N];
};

inline void ekfMultiply(
    const float* a, const float* b, float* result,
    uint8_t aRows, uint8_t aColumns, uint8_t bColumns) {
    for (uint8_t row = 0; row < aRows; ++row) {
        for (uint8_t column = 0; column < bColumns; ++column) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < aColumns; ++k) {
                sum += a[row * aColumns + k] * b[k * bColumns + column];
            }
            result[row * bColumns + column] = sum;
        }
    }
}

inline void ekfTranspose(
    const float* input, float* output, uint8_t rows, uint8_t columns) {
    for (uint8_t row = 0; row < rows; ++row) {
        for (uint8_t column = 0; column < columns; ++column) {
            output[column * rows + row] = input[row * columns + column];
        }
    }
}

inline void ekfInitialise(EkfState& ekf, const float diagonal[EKF_N]) {
    for (uint8_t row = 0; row < EKF_N; ++row) {
        ekf.x[row] = 0.0f;
        for (uint8_t column = 0; column < EKF_N; ++column) {
            ekf.covariance[row * EKF_N + column] =
                row == column ? diagonal[row] : 0.0f;
        }
    }
}

inline void ekfPredictDiagonal(
    EkfState& ekf,
    const float predicted[EKF_N],
    const float jacobian[EKF_N * EKF_N],
    const float processNoise[EKF_N],
    EkfScratch& scratch) {
    memcpy(ekf.x, predicted, sizeof(ekf.x));
    ekfMultiply(
        jacobian, ekf.covariance, scratch.temporary0,
        EKF_N, EKF_N, EKF_N);
    ekfTranspose(jacobian, scratch.temporary1, EKF_N, EKF_N);
    ekfMultiply(
        scratch.temporary0, scratch.temporary1, scratch.temporary2,
        EKF_N, EKF_N, EKF_N);
    memcpy(ekf.covariance, scratch.temporary2, sizeof(ekf.covariance));
    for (uint8_t i = 0; i < EKF_N; ++i) {
        ekf.covariance[i * EKF_N + i] += processNoise[i];
    }
}

inline bool ekfUpdateScalar(
    EkfState& ekf, uint8_t stateIndex, float measurement, float noise) {
    if (stateIndex >= EKF_N || !(noise > 0.0f)) return false;
    const uint8_t measurementRow = stateIndex * EKF_N;
    const float innovationVariance =
        ekf.covariance[measurementRow + stateIndex] + noise;
    if (!(innovationVariance > 0.0f) || !isfinite(innovationVariance)) {
        return false;
    }
    const float innovation = measurement - ekf.x[stateIndex];
    if (!isfinite(innovation)) return false;

    const float inverseVariance = 1.0f / innovationVariance;
    float gain[EKF_N];
    for (uint8_t i = 0; i < EKF_N; ++i) {
        gain[i] = ekf.covariance[i * EKF_N + stateIndex] * inverseVariance;
        ekf.x[i] += gain[i] * innovation;
    }

    float oldMeasurementRow[EKF_N];
    for (uint8_t j = 0; j < EKF_N; ++j) {
        oldMeasurementRow[j] = ekf.covariance[measurementRow + j];
    }
    for (uint8_t i = 0; i < EKF_N; ++i) {
        for (uint8_t j = 0; j < EKF_N; ++j) {
            ekf.covariance[i * EKF_N + j] -= gain[i] * oldMeasurementRow[j];
        }
    }
    return true;
}
