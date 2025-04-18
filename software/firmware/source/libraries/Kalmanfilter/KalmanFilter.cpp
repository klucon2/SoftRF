#include "KalmanFilter.h"
#include "Arduino.h"


KalmanFilter::KalmanFilter(float processNoise, float measurementNoise, float estimatedError, float initialValue) {
    setParameters(processNoise, measurementNoise, estimatedError, initialValue);
}

void KalmanFilter::setParameters(float processNoise, float measurementNoise, float estimatedError, float initialValue) {
    Q = processNoise;
    R = measurementNoise;
    P = estimatedError;
    X = initialValue;
}

float KalmanFilter::update(float measurement) {
    
    // Prediction update
    P = P + Q;

    // Measurement update
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    return X;
}

float KalmanFilter::getValue() const {
    return X;
}

void KalmanFilter::setValue(float newR) {
    R = newR-48;
    Serial.print("KalmanFilter: ");
    Serial.println(R);
}
