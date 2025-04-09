#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include "coordinates_transformation.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <optional>

class MotionEstimator {
public:
    MotionEstimator(int max_points = 200, int min_distance = 15, int block_size = 3,
                    TransformationGetter* transformations_getter = nullptr,
                    bool draw_flow = false, cv::Scalar flow_color = cv::Scalar(0, 0, 255), double quality_level = 0.01);

    std::optional<CoordinatesTransformation*> update(const cv::Mat& frame, const cv::Mat& mask = cv::Mat());

private:
    int max_points;
    int min_distance;
    int block_size;
    bool draw_flow;
    cv::Scalar flow_color;
    double quality_level;
    cv::Mat gray_prvs, gray_next, prev_pts;
    TransformationGetter* transformations_getter;
    TransformationGetter* transformations_getter_copy;

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> get_sparse_flow(const cv::Mat& gray_next, const cv::Mat& gray_prvs, const cv::Mat& prev_pts,
                                                                 int max_points, int min_distance, int block_size, const cv::Mat& mask, double quality_level);
};

#endif // MOTION_ESTIMATOR_H
