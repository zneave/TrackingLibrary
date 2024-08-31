#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

// KalmanFilter struct to hold the state and matrices
struct KalmanFilter {
    Eigen::VectorXd x;  // State vector
    Eigen::MatrixXd P;  // Covariance matrix
    Eigen::MatrixXd F;  // State transition matrix
    Eigen::MatrixXd H;  // Observation matrix
    Eigen::MatrixXd R;  // Measurement noise matrix
    Eigen::MatrixXd Q;  // Process noise matrix

    KalmanFilter(int dim_x, int dim_z) {
        x = Eigen::VectorXd::Zero(dim_x);
        P = Eigen::MatrixXd::Identity(dim_x, dim_x);
        F = Eigen::MatrixXd::Identity(dim_x, dim_x);
        H = Eigen::MatrixXd::Zero(dim_z, dim_x);
        R = Eigen::MatrixXd::Identity(dim_z, dim_z);
        Q = Eigen::MatrixXd::Identity(dim_x, dim_x);
    }
};

// Function declarations
void predict(KalmanFilter& filter);
void update(KalmanFilter& filter, const Eigen::VectorXd& z);

#endif // KALMAN_FILTER_H
