#include "KalmanFilter.h"
#include <memory>

// FilterPyKalmanFilterFactory class
class FilterPyKalmanFilterFactory {
public:
    FilterPyKalmanFilterFactory(double R, double Q, double P)
        : R(R), Q(Q), P(P) {}

    std::unique_ptr<KalmanFilter> create_filter(const Eigen::MatrixXd& initial_detection) {
        int num_points = initial_detection.rows();
        int dim_points = initial_detection.cols();
        int dim_z = dim_points * num_points;
        int dim_x = 2 * dim_z;

        auto filter = std::make_unique<KalmanFilter>(dim_x, dim_z);

        filter->F.block(0, dim_z, dim_z, dim_z) = Eigen::MatrixXd::Identity(dim_z, dim_z);
        filter->H = Eigen::MatrixXd::Identity(dim_z, dim_x);
        filter->R = Eigen::MatrixXd::Identity(dim_z, dim_z) * R;
        filter->Q = Eigen::MatrixXd::Identity(dim_x, dim_x) * Q;
        filter->Q.block(dim_z, dim_z, dim_z, dim_z) *= Q;
        filter->P.block(dim_z, dim_z, dim_z, dim_z) *= P;

        filter->x.head(dim_z) = Eigen::Map<const Eigen::VectorXd>(initial_detection.data(), initial_detection.size());
        filter->x.tail(dim_z).setZero();

        return filter;
    }

private:
    double R, Q, P;
};

// NoFilterFactory class
class NoFilterFactory {
public:
    std::unique_ptr<KalmanFilter> create_filter(const Eigen::MatrixXd& initial_detection) {
        int num_points = initial_detection.rows();
        int dim_points = initial_detection.cols();
        int dim_z = dim_points * num_points;
        int dim_x = 2 * dim_z;

        auto no_filter = std::make_unique<KalmanFilter>(dim_x, dim_z);
        no_filter->x.head(dim_z) = Eigen::Map<const Eigen::VectorXd>(initial_detection.data(), initial_detection.size());
        no_filter->x.tail(dim_z).setZero();

        return no_filter;
    }
};
