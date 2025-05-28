package com.espressif.ui.adapters;

public class KalmanFilter {
    private float q; // Noise covariance (process noise)
    private float r; // Noise covariance (measurement noise)
    private float x; // Estimated value
    private float p; // Error covariance
    private float k; // Kalman gain

    public KalmanFilter(float processNoise, float measurementNoise, float estimatedError, float initialValue) {
        this.q = processNoise;
        this.r = measurementNoise;
        this.p = estimatedError;
        this.x = initialValue;
    }

    public float update(float measurement) {
        // Prediction update
        p = p + q;

        // Measurement update
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;

        return x;
    }
}
