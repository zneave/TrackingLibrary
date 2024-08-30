#ifndef OPTIMIZED_KALMAN_FILTER_H
#define OPTIMIZED_KALMAN_FILTER_H

#include <Eigen/Dense>

class OptimizedKalmanFilter {
public:
    OptimizedKalmanFilter(int dim_x, int dim_z, double pos_variance = 10, double pos_vel_covariance = 0, double vel_variance = 1, double q = 0.1, double r = 4);
    void predict();
    void update(const Eigen::VectorXd& detection_points_flatten, const Eigen::MatrixXd* R = nullptr, const Eigen::MatrixXd* H = nullptr);

private:
    int dim_z;
    Eigen::VectorXd x;
    Eigen::VectorXd pos_variance;
    Eigen::VectorXd pos_vel_covariance;
    Eigen::VectorXd vel_variance;
    double q_Q;
    Eigen::VectorXd default_r;
};

#endif // OPTIMIZED_KALMAN_FILTER_H
