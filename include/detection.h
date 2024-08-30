#ifndef DETECTION_H
#define DETECTION_H

#include <vector>
#include <memory>
#include <Eigen/Dense>

class CoordinatesTransformation;

class Detection {
public:
    Detection(const Eigen::MatrixXd& points, 
              const Eigen::VectorXd& scores = Eigen::VectorXd(), 
              const void* data = nullptr, 
              const int label = 0, 
              const Eigen::VectorXd& embedding = Eigen::VectorXd());

    void update_coordinate_transformation(const CoordinatesTransformation& coord_transform);

    const Eigen::MatrixXd& get_points() const { return points; }
    const Eigen::MatrixXd& get_absolute_points() const { return absolute_points; }
    const Eigen::VectorXd& get_scores() const { return scores; }
    const int get_label() const { return label; }

private:
    Eigen::MatrixXd points;
    Eigen::MatrixXd absolute_points;
    Eigen::VectorXd scores;
    void* data;  // Raw pointer for data, type depends on usage
    int label;
    Eigen::VectorXd embedding;
};

#endif // DETECTION_H
