// detection.cpp

#include "include/detection.h"
#include "coordinates_transformation.h"  // Include header for CoordinatesTransformation

Detection::Detection(const Eigen::MatrixXd& points, 
                     const Eigen::VectorXd& scores, 
                     const void* data, 
                     const int label, 
                     const Eigen::VectorXd& embedding)
    : points(points), 
      absolute_points(points), 
      scores(scores), 
      data(const_cast<void*>(data)), 
      label(label), 
      embedding(embedding) {}

void Detection::update_coordinate_transformation(const CoordinatesTransformation& coord_transform) {
    absolute_points = coord_transform.rel_to_abs(points);
}
