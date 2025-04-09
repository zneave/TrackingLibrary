#ifndef COORDINATES_TRANSFORMATION_H
#define COORDINATES_TRANSFORMATION_H

#include <Eigen/Dense>
#include <tuple>

class CoordinatesTransformation {
public:
    virtual Eigen::MatrixXd abs_to_rel(const Eigen::MatrixXd& points) const = 0;
    virtual Eigen::MatrixXd rel_to_abs(const Eigen::MatrixXd& points) const = 0;
    virtual ~CoordinatesTransformation() = default;
};

class TransformationGetter {
public:
    virtual std::tuple<bool, CoordinatesTransformation*> operator()(const Eigen::MatrixXd& curr_pts,
                                                                    const Eigen::MatrixXd& prev_pts) const = 0;

    virtual TransformationGetter* clone() const = 0;
    virtual ~TransformationGetter() = default;
};

#endif // COORDINATES_TRANSFORMATION_H
