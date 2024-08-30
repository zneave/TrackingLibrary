#include "include/OptimizedKalmanFilter.h"

OptimizedKalmanFilter::OptimizedKalmanFilter(int dim_x, int dim_z, double pos_variance, double pos_vel_covariance, double vel_variance, double q, double r)
    : dim_z(dim_z), x(Eigen::VectorXd::Zero(dim_x)), pos_variance(Eigen::VectorXd::Constant(dim_z, pos_variance)),
      pos_vel_covariance(Eigen::VectorXd::Constant(dim_z, pos_vel_covariance)), vel_variance(Eigen::VectorXd::Constant(dim_z, vel_variance)),
      q_Q(q), default_r(Eigen::VectorXd::Constant(dim_z, r)) {}

void OptimizedKalmanFilter::predict() {
    x.head(dim_z) += x.segment(dim_z, dim_z);
}

void OptimizedKalmanFilter::update(const Eigen::VectorXd& detection_points_flatten, const Eigen::MatrixXd* R, const Eigen::MatrixXd* H) {
    Eigen::VectorXd diagonal = (H != nullptr) ? Eigen::VectorXd(H->diagonal()) : Eigen::VectorXd::Ones(dim_z);
    Eigen::VectorXd one_minus_diagonal = Eigen::VectorXd::Ones(dim_z) - diagonal;

    Eigen::VectorXd kalman_r = R != nullptr ? R->diagonal() : default_r;
    Eigen::VectorXd error = (detection_points_flatten - x.head(dim_z)).cwiseProduct(diagonal);

    Eigen::VectorXd vel_var_plus_pos_vel_cov = pos_vel_covariance + vel_variance;
    Eigen::VectorXd added_variances = pos_variance.array() + pos_vel_covariance.array() + vel_var_plus_pos_vel_cov.array() + q_Q + kalman_r.array();

    Eigen::VectorXd kalman_r_over_added_variances = kalman_r.cwiseQuotient(added_variances);
    Eigen::VectorXd vel_var_plus_pos_vel_cov_over_added_variances = vel_var_plus_pos_vel_cov.cwiseQuotient(added_variances);

    Eigen::VectorXd added_variances_or_kalman_r = added_variances.cwiseProduct(one_minus_diagonal) + kalman_r.cwiseProduct(diagonal);

    x.head(dim_z) += (Eigen::VectorXd::Ones(dim_z) - kalman_r_over_added_variances).cwiseProduct(error);
    x.segment(dim_z, dim_z) += vel_var_plus_pos_vel_cov_over_added_variances.cwiseProduct(error);

    pos_variance = (Eigen::VectorXd::Ones(dim_z) - kalman_r_over_added_variances).cwiseProduct(added_variances_or_kalman_r);
    pos_vel_covariance = vel_var_plus_pos_vel_cov_over_added_variances.cwiseProduct(added_variances_or_kalman_r);
    vel_variance.array() += (-q_Q) + diagonal.array() * (vel_var_plus_pos_vel_cov_over_added_variances.array().square() * added_variances.array());
}
