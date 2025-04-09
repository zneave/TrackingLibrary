#ifndef FILTER_FACTORY_H
#define FILTER_FACTORY_H

#include <memory>
#include "filter.h"
#include "KalmanFilter.h"
#include "NoFilter.h"

class FilterFactory {
public:
    virtual ~FilterFactory() = default;
    virtual std::shared_ptr<Filter> create(const Eigen::MatrixXd& initial_detection) const = 0;
};

class FilterPyKalmanFilterFactory : public FilterFactory {
public:
    FilterPyKalmanFilterFactory(double R = 4.0, double Q = 0.1, double P = 10.0);

    std::shared_ptr<Filter> create(const Eigen::MatrixXd& initial_detection) const override;

private:
    double R, Q, P;
};

class NoFilterFactory : public FilterFactory {
public:
    std::shared_ptr<Filter> create(const Eigen::MatrixXd& initial_detection) const override;
};

#endif // FILTER_FACTORY_H
