// KalmanFilter.cpp

#include "KalmanFilter.h"
#include <Eigen/Dense>

// Predict the state and covariance
void predict(KalmanFilter& filter) {
    filter.x = filter.F * filter.x;
    filter.P = filter.F * filter.P * filter.F.transpose() + filter.Q;
}

// Update the state with the new measurement
void update(KalmanFilter& filter, const Eigen::VectorXd& z) {
    Eigen::MatrixXd S = filter.H * filter.P * filter.H.transpose() + filter.R;
    Eigen::MatrixXd K = filter.P * filter.H.transpose() * S.inverse();

    filter.x += K * (z - filter.H * filter.x);
    filter.P = (Eigen::MatrixXd::Identity(filter.x.size(), filter.x.size()) - K * filter.H) * filter.P;
}
