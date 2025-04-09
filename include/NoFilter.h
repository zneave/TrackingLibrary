#ifndef NO_FILTER_H
#define NO_FILTER_H

#include <Eigen/Dense>
#include "filter.h"

class NoFilter : public Filter {
public:
    NoFilter(int dim_x, int dim_z);

    void predict() override;

    void update(const Eigen::VectorXd& detection_points_flatten,
                const Eigen::MatrixXd* R = nullptr,
                const Eigen::MatrixXd* H = nullptr) override;

    Eigen::VectorXd get_state() const override;

private:
    int dim_z;
    Eigen::VectorXd x;
};

#endif // NO_FILTER_H
