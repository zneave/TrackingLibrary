#ifndef NO_FILTER_H
#define NO_FILTER_H

#include <Eigen/Dense>

class NoFilter {
public:
    NoFilter(int dim_x, int dim_z);
    void predict();
    void update(const Eigen::VectorXd& detection_points_flatten, const Eigen::MatrixXd* R = nullptr, const Eigen::MatrixXd* H = nullptr);

    Eigen::VectorXd x;

private:
    int dim_z;
};

#endif // NO_FILTER_H
