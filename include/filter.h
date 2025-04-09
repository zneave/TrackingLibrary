#ifndef FILTER_H
#define FILTER_H

#include <Eigen/Dense>

class Filter {
public:
    virtual ~Filter() = default;

    virtual void predict() = 0;

    virtual void update(const Eigen::VectorXd& z,
                        const Eigen::MatrixXd* R = nullptr,
                        const Eigen::MatrixXd* H = nullptr) = 0;

    virtual Eigen::VectorXd get_state() const = 0;
};

#endif // FILTER_H
