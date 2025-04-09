#include "NoFilter.h"

NoFilter::NoFilter(int dim_x, int dim_z) : dim_z(dim_z), x(Eigen::VectorXd::Zero(dim_x)) {}

void NoFilter::predict() {
    // No prediction logic
}

void NoFilter::update(const Eigen::VectorXd& detection_points_flatten, const Eigen::MatrixXd* R, const Eigen::MatrixXd* H) {
    if (H != nullptr) {
        Eigen::VectorXd diagonal = H->diagonal();
        Eigen::VectorXd one_minus_diagonal = Eigen::VectorXd::Ones(dim_z) - diagonal;

        x.head(dim_z) = diagonal.asDiagonal() * detection_points_flatten + one_minus_diagonal.asDiagonal() * x.head(dim_z);
    } else {
        x.head(dim_z) = detection_points_flatten;
    }
}
