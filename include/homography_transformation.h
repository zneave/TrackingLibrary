#ifndef HOMOGRAPHY_TRANSFORMATION_H
#define HOMOGRAPHY_TRANSFORMATION_H

#include "coordinates_transformation.h"
#include <opencv2/opencv.hpp>
#include <iostream>

class HomographyTransformation : public CoordinatesTransformation {
public:
    HomographyTransformation(const Eigen::Matrix3d& homography_matrix) 
        : homography_matrix(homography_matrix), inverse_homography_matrix(homography_matrix.inverse()) {}

    Eigen::MatrixXd abs_to_rel(const Eigen::MatrixXd& points) const override {
        Eigen::MatrixXd points_h = points;
        points_h.conservativeResize(points_h.rows(), points_h.cols() + 1);
        points_h.col(points_h.cols() - 1).setOnes();

        Eigen::MatrixXd transformed_points = (points_h * homography_matrix.transpose()).rowwise().hnormalized();
        return transformed_points.leftCols(2);
    }

    Eigen::MatrixXd rel_to_abs(const Eigen::MatrixXd& points) const override {
        Eigen::MatrixXd points_h = points;
        points_h.conservativeResize(points_h.rows(), points_h.cols() + 1);
        points_h.col(points_h.cols() - 1).setOnes();

        Eigen::MatrixXd transformed_points = (points_h * inverse_homography_matrix.transpose()).rowwise().hnormalized();
        return transformed_points.leftCols(2);
    }

private:
    Eigen::Matrix3d homography_matrix;
    Eigen::Matrix3d inverse_homography_matrix;
};

// Calculates HomographyTransformation between points using OpenCV's findHomography
class HomographyTransformationGetter : public TransformationGetter {
public:
    HomographyTransformationGetter(int method = cv::RANSAC, double ransac_reproj_threshold = 3.0, int max_iters = 2000, double confidence = 0.995, double proportion_points_used_threshold = 0.9)
        : method(method), ransac_reproj_threshold(ransac_reproj_threshold), max_iters(max_iters), confidence(confidence), proportion_points_used_threshold(proportion_points_used_threshold) {}

    std::tuple<bool, CoordinatesTransformation*> operator()(const Eigen::MatrixXd& curr_pts, const Eigen::MatrixXd& prev_pts) const override {
        if (curr_pts.rows() < 4 || prev_pts.rows() < 4) {
            std::cerr << "Not enough points to compute homography." << std::endl;
            return std::make_tuple(true, nullptr);
        }

        std::vector<cv::Point2f> curr_pts_cv(curr_pts.rows()), prev_pts_cv(prev_pts.rows());
        cv::eigen2cv(curr_pts, curr_pts_cv);
        cv::eigen2cv(prev_pts, prev_pts_cv);

        cv::Mat inlier_mask;
        cv::Mat homography_matrix_cv = cv::findHomography(prev_pts_cv, curr_pts_cv, method, ransac_reproj_threshold, inlier_mask, max_iters, confidence);
        Eigen::Matrix3d homography_matrix;
        cv::cv2eigen(homography_matrix_cv, homography_matrix);

        double proportion_points_used = static_cast<double>(cv::countNonZero(inlier_mask)) / static_cast<double>(inlier_mask.rows);
        bool update_prvs = proportion_points_used < proportion_points_used_threshold;

        return std::make_tuple(update_prvs, new HomographyTransformation(homography_matrix));
    }

private:
    int method;
    double ransac_reproj_threshold;
    int max_iters;
    double confidence;
    double proportion_points_used_threshold;
};

#endif // HOMOGRAPHY_TRANSFORMATION_H
