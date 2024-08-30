#include "include/FilterFactory.h"
#include "NoFilter.h"  // Include this if NoFilter is implemented in a separate file
#include <Eigen/Dense>

FilterPyKalmanFilterFactory::FilterPyKalmanFilterFactory(double R, double Q, double P)
    : R(R), Q(Q), P(P) {}

std::unique_ptr<Eigen::MatrixXd> FilterPyKalmanFilterFactory::create_filter(const Eigen::MatrixXd& initial_detection) {
    int num_points = initial_detection.rows();
    int dim_points = initial_detection.cols();
    int dim_z = dim_points * num_points;
    int dim_x = 2 * dim_z;

    auto filter = std::make_unique<Eigen::MatrixXd>(dim_x, dim_z);

    // Initialize matrices (example setup)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(dim_x, dim_x);
    double dt = 1.0;
    F.block(0, dim_z, dim_z, dim_z) = dt * Eigen::MatrixXd::Identity(dim_z, dim_z);

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(dim_z, dim_x);
    Eigen::MatrixXd R_mat = Eigen::MatrixXd::Identity(dim_z, dim_z) * R;
    Eigen::MatrixXd Q_mat = Eigen::MatrixXd::Identity(dim_x, dim_x) * Q;
    Q_mat.block(dim_z, dim_z, dim_z, dim_z) *= Q;

    Eigen::VectorXd x(dim_x);
    x.head(dim_z) = Eigen::Map<const Eigen::VectorXd>(initial_detection.data(), initial_detection.size());
    x.tail(dim_z).setZero();

    Eigen::MatrixXd P_mat = Eigen::MatrixXd::Identity(dim_x, dim_x);
    P_mat.block(dim_z, dim_z, dim_z, dim_z) *= P;

    // Setup filter struct or object here (custom implementation needed)
    // Assuming custom Kalman filter object or structure is used

    // Return constructed filter
    return filter;
}

std::unique_ptr<Eigen::MatrixXd> NoFilterFactory::create_filter(const Eigen::MatrixXd& initial_detection) {
    int num_points = initial_detection.rows();
    int dim_points = initial_detection.cols();
    int dim_z = dim_points * num_points;
    int dim_x = 2 * dim_z;

    auto no_filter = std::make_unique<NoFilter>(dim_x, dim_z);
    no_filter->x.head(dim_z) = Eigen::Map<const Eigen::VectorXd>(initial_detection.data(), initial_detection.size());

    return nullptr; // Replace with actual filter object if needed
}
