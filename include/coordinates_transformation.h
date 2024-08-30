#ifndef COORDINATES_TRANSFORMATION_H
#define COORDINATES_TRANSFORMATION_H

#include <Eigen/Dense>
#include <tuple>

// Abstract base class representing a coordinate transformation
class CoordinatesTransformation {
public:
    virtual ~CoordinatesTransformation() = default;

    virtual Eigen::MatrixXd abs_to_rel(const Eigen::MatrixXd& points) const = 0;
    virtual Eigen::MatrixXd rel_to_abs(const Eigen::MatrixXd& points) const = 0;
};

// Abstract base class representing a method for finding CoordinatesTransformation between two sets of points
class TransformationGetter {
public:
    virtual ~TransformationGetter() = default;

    virtual std::tuple<bool, CoordinatesTransformation*> operator()(const Eigen::MatrixXd& curr_pts, const Eigen::MatrixXd& prev_pts) const = 0;
};

#endif // COORDINATES_TRANSFORMATION_H
