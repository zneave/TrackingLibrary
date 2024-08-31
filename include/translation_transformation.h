#ifndef TRANSLATION_TRANSFORMATION_H
#define TRANSLATION_TRANSFORMATION_H

#include "coordinates_transformation.h"
#include <Eigen/Dense>
#include <map>
#include <tuple>

struct RowVector2dComparator {
    bool operator()(const Eigen::RowVector2d& lhs, const Eigen::RowVector2d& rhs) const {
        if (lhs.x() == rhs.x()) {
            return lhs.y() < rhs.y();
        }
        return lhs.x() < rhs.x();
    }
};

class TranslationTransformation : public CoordinatesTransformation {
public:
    TranslationTransformation(const Eigen::VectorXd& movement_vector) : movement_vector(movement_vector) {}

    Eigen::MatrixXd abs_to_rel(const Eigen::MatrixXd& points) const override {
        return points.rowwise() + movement_vector.transpose();
    }

    Eigen::MatrixXd rel_to_abs(const Eigen::MatrixXd& points) const override {
        return points.rowwise() - movement_vector.transpose();
    }

private:
    Eigen::VectorXd movement_vector;
};

class TranslationTransformationGetter : public TransformationGetter {
public:
    TranslationTransformationGetter(double bin_size = 0.2, double proportion_points_used_threshold = 0.9)
        : bin_size(bin_size), proportion_points_used_threshold(proportion_points_used_threshold), data(Eigen::VectorXd::Zero(2)) {}

    std::tuple<bool, CoordinatesTransformation*> operator()(const Eigen::MatrixXd& curr_pts, const Eigen::MatrixXd& prev_pts) const override {
        Eigen::MatrixXd flow = curr_pts - prev_pts;
        flow = (flow.array() / bin_size).round() * bin_size;
        
        std::map<Eigen::RowVector2d, int, RowVector2dComparator> flow_count;
        for (int i = 0; i < flow.rows(); ++i) {
            Eigen::RowVector2d f = flow.row(i);
            flow_count[f]++;
        }

        auto max_flow = std::max_element(flow_count.begin(), flow_count.end(),
                                         [](const std::pair<Eigen::RowVector2d, int>& a, const std::pair<Eigen::RowVector2d, int>& b) {
                                             return a.second < b.second;
                                         });

        double proportion_points_used = static_cast<double>(max_flow->second) / static_cast<double>(prev_pts.rows());
        bool update_prvs = proportion_points_used < proportion_points_used_threshold;

        Eigen::VectorXd flow_mode = max_flow->first;
        if (update_prvs) {
            data = flow_mode;
        } else {
            flow_mode += data;
        }

        return std::make_tuple(update_prvs, new TranslationTransformation(flow_mode));
    }

private:
    double bin_size;
    double proportion_points_used_threshold;
    mutable Eigen::VectorXd data;
};

#endif // TRANSLATION_TRANSFORMATION_H
