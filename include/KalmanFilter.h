#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include "filter.h"

class KalmanFilter : public Filter {
public:
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

    void predict() override {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd* R_ = nullptr, const Eigen::MatrixXd* H_ = nullptr) override {
        const Eigen::MatrixXd& R_used = R_ ? *R_ : R;
        const Eigen::MatrixXd& H_used = H_ ? *H_ : H;

        Eigen::VectorXd y = z - H_used * x;
        Eigen::MatrixXd S = H_used * P * H_used.transpose() + R_used;
        Eigen::MatrixXd K = P * H_used.transpose() * S.inverse();
        x += K * y;
        P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * H_used) * P;
    }

    Eigen::VectorXd get_state() const override {
        return x;
    }
};

#endif // KALMAN_FILTER_H
