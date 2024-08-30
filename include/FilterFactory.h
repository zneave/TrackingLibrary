#ifndef FILTER_FACTORY_H
#define FILTER_FACTORY_H

#include <Eigen/Dense>
#include <memory>
#include "detection.h"
#include "tracked_object.h"

// Abstract base class
class FilterFactory {
public:
    virtual ~FilterFactory() = default;

    virtual std::unique_ptr<Eigen::MatrixXd> create_filter(const Eigen::MatrixXd& initial_detection) = 0;
};

// Derived class for Kalman Filter using FilterPy logic
class FilterPyKalmanFilterFactory : public FilterFactory {
public:
    FilterPyKalmanFilterFactory(double R = 4.0, double Q = 0.1, double P = 10.0);
    std::unique_ptr<Eigen::MatrixXd> create_filter(const Eigen::MatrixXd& initial_detection) override;

private:
    double R;
    double Q;
    double P;
};

// Derived class for NoFilter
class NoFilterFactory : public FilterFactory {
public:
    std::unique_ptr<Eigen::MatrixXd> create_filter(const Eigen::MatrixXd& initial_detection) override;
};

#endif // FILTER_FACTORY_H
