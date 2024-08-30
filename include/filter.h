// filter.h
#ifndef FILTER_H
#define FILTER_H

#include <Eigen/Dense>

// Declare the Filter class or struct here

class Filter {
public:
    // Your Filter class methods and members
    virtual void predict() = 0;
    virtual void update(const Eigen::VectorXd& detection_points_flatten, const Eigen::MatrixXd* R = nullptr, const Eigen::MatrixXd* H = nullptr) = 0;
    
    // Destructor, copy constructors, etc., as needed
    virtual ~Filter() = default;
};

#endif // FILTER_H
