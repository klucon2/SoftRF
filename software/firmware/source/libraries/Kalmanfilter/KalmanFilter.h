#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    KalmanFilter(float processNoise = 1.0, float measurementNoise = 1.0, float estimatedError = 1.0, float initialValue = 0.0);

    void setParameters(float processNoise, float measurementNoise, float estimatedError, float initialValue);
    float update(float measurement);
    float getValue() const;
    void setValue(float newR);
    float R; // Measurement noise

private:
    float Q; // Process noise
    //float R; // Measurement noise
    float P; // Estimation error
    float K; // Kalman gain
    float X; // Current estimate
};

#endif
