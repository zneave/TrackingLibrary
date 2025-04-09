#include "motion_estimator.h"
#include "homography_transformation.h"
#include <opencv2/core/eigen.hpp>

MotionEstimator::MotionEstimator(int max_points, int min_distance, int block_size,
                                 TransformationGetter* transformations_getter, bool draw_flow, cv::Scalar flow_color, double quality_level)
    : max_points(max_points), min_distance(min_distance), block_size(block_size), draw_flow(draw_flow), flow_color(flow_color), quality_level(quality_level)
{
    if (transformations_getter) {
        this->transformations_getter = transformations_getter;
        this->transformations_getter_copy = transformations_getter->clone();
    } else {
        this->transformations_getter = new HomographyTransformationGetter();
        this->transformations_getter_copy = this->transformations_getter->clone();
    }
}

std::optional<CoordinatesTransformation*> MotionEstimator::update(const cv::Mat& frame, const cv::Mat& mask) {
    cv::cvtColor(frame, gray_next, cv::COLOR_BGR2GRAY);
    if (gray_prvs.empty()) {
        gray_prvs = gray_next.clone();
    }

    Eigen::MatrixXd curr_pts, prev_pts_eig;
    std::tie(curr_pts, prev_pts_eig) = get_sparse_flow(gray_next, gray_prvs, prev_pts, max_points, min_distance, block_size, mask, quality_level);

    if (draw_flow) {
        for (int i = 0; i < curr_pts.rows(); ++i) {
            cv::Point p1(curr_pts(i, 0), curr_pts(i, 1));
            cv::Point p2(prev_pts_eig(i, 0), prev_pts_eig(i, 1));
            cv::line(frame, p1, p2, flow_color, 2);
            cv::circle(frame, p1, 3, flow_color, -1);
        }
    }

    bool update_prvs;
    CoordinatesTransformation* coord_transformations;
    std::tie(update_prvs, coord_transformations) = (*transformations_getter)(curr_pts, prev_pts_eig);

    if (update_prvs) {
        gray_prvs = gray_next.clone();
    }

    return std::optional<CoordinatesTransformation*>(coord_transformations);
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> MotionEstimator::get_sparse_flow(const cv::Mat& gray_next, const cv::Mat& gray_prvs, const cv::Mat& prev_pts,
                                                                              int max_points, int min_distance, int block_size, const cv::Mat& mask, double quality_level) {
    std::vector<cv::Point2f> curr_pts_cv, prev_pts_cv;
    cv::goodFeaturesToTrack(gray_prvs, prev_pts_cv, max_points, quality_level, min_distance, mask, block_size);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(gray_prvs, gray_next, prev_pts_cv, curr_pts_cv, status, err);

    // Fix: capture curr_pts_cv
    auto it = std::remove_if(curr_pts_cv.begin(), curr_pts_cv.end(),
                             [&status, &curr_pts_cv](const cv::Point2f& pt) {
                                 auto idx = &pt - &curr_pts_cv[0];
                                 return status[idx] == 0;
                             });
    curr_pts_cv.erase(it, curr_pts_cv.end());

    // Manual conversion to Eigen
    Eigen::MatrixXd curr_pts(curr_pts_cv.size(), 2);
    Eigen::MatrixXd prev_pts_eig(prev_pts_cv.size(), 2);

    for (size_t i = 0; i < curr_pts_cv.size(); ++i) {
        curr_pts(i, 0) = curr_pts_cv[i].x;
        curr_pts(i, 1) = curr_pts_cv[i].y;
    }
    for (size_t i = 0; i < prev_pts_cv.size(); ++i) {
        prev_pts_eig(i, 0) = prev_pts_cv[i].x;
        prev_pts_eig(i, 1) = prev_pts_cv[i].y;
    }

    return std::make_tuple(curr_pts, prev_pts_eig);
}
